"""One lazy worker, trailing-edge debounce, latest-only queues and versioned results."""
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
import threading
import time

from .model import uri_key
from .validation import Cancelled, ValidationResult, Validator, unavailable

@dataclass(frozen=True)
class Job:
    uri: str
    generation: int
    version: object
    context: object
    owner: object


class Diagnostics:
    def __init__(self, publish, validator=None):
        self.publish = publish
        self.validator = validator or Validator()
        self.condition = threading.Condition()
        self.pending = {}
        self.generations = {}
        self.sequence = 0
        self.identity = object()
        self.dependencies = {}
        self.dependency_versions = {}
        self.context = None
        self.buffers = {}
        self.invalidated = set()
        self.invalidate_all = False
        self.delay = 0.5
        self.active = None
        self.callbacks = 0
        self.paused = False
        self.closed = False
        self.thread = None

    def update(self, context, buffers, changed=(), immediate=False, all_documents=False, delay=0.5, disk_changes=()):
        with self.condition:
            if self.closed:
                return set()
            configuration_changed = self.context != context
            if configuration_changed:
                self.dependencies.clear()
                self.dependency_versions.clear()
            self.context, self.buffers = context, buffers
            self.delay = delay
            if not self.invalidate_all:
                self.invalidated.update(disk_changes)
                if len(self.invalidated) > 512:
                    self.invalidated.clear()
                    self.invalidate_all = True
            changed_keys = {uri_key(uri) for uri in changed}
            targets = set(buffers) if all_documents or configuration_changed else {uri for uri in buffers if uri_key(uri) in changed_keys}
            for uri, deps in self.dependencies.items():
                if any(uri_key(dep) in changed_keys for dep in deps) and uri in buffers:
                    targets.add(uri)
            # Pending/new checks have no complete dependency graph yet. Refresh
            # them on a header edit, rather than using an obsolete overlay.
            if any(Path(uri).suffix in ('.h', '.vexh') for uri in changed):
                targets.update(self.pending)
            if self.active:
                job, cancel = self.active
                unknown = job.uri not in self.dependency_versions or self.dependency_versions[job.uri] != job.version
                if job.uri in targets or configuration_changed or (unknown and changed):
                    cancel.set()
                    if job.uri in buffers:
                        targets.add(job.uri)
            for uri in list(self.generations):
                if uri not in buffers:
                    self.generations.pop(uri, None)
                    self.dependencies.pop(uri, None)
                    self.dependency_versions.pop(uri, None)
                    self.pending.pop(uri, None)
            due = time.monotonic() + (0 if immediate else delay)
            for uri in targets:
                if uri in buffers:
                    self.sequence += 1
                    self.generations[uri] = self.sequence
                    self.pending[uri] = due
            # One tiny (URI, deadline) entry per open document, never per revision.
            if self.pending and self.thread is None:
                self.thread = threading.Thread(target=self._work, name='ko-vex-diagnostics', daemon=True)
                self.thread.start()
            self.condition.notify_all()
            return targets

    def current(self, job):
        with self.condition:
            value = self.buffers.get(job.uri)
            return (not self.closed and job.owner is self.identity and value is not None and self.context == job.context and
                    value[1] == job.version and self.generations.get(job.uri) == job.generation)

    @contextmanager
    def suspend(self):
        """Do not contend with foreground rename/reference compiler operations."""
        with self.condition:
            self.paused = True
            if self.active:
                self.active[1].set()
                self.pending[self.active[0].uri] = time.monotonic()+self.delay
            while self.active:
                self.condition.wait()
        try:
            yield
        finally:
            with self.condition:
                self.paused = False
                self.condition.notify_all()

    def wait_idle(self, timeout=10):
        deadline = time.monotonic()+timeout
        with self.condition:
            while self.pending or self.active or self.callbacks:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    return False
                self.condition.wait(remaining)
            return True

    def close(self, wait=True):
        with self.condition:
            self.closed = True
            self.pending.clear()
            self.buffers = {}
            self.invalidated.clear()
            if self.active:
                self.active[1].set()
            self.condition.notify_all()
        if self.thread and wait:
            self.thread.join(timeout=2)
        elif self.thread is None:
            self.validator.close()

    def _work(self):
        try:
            while True:
                with self.condition:
                    while True:
                        if self.closed:
                            return
                        if self.pending and not self.paused:
                            uri = min(self.pending, key=self.pending.get)
                            remaining = self.pending[uri]-time.monotonic()
                            if remaining <= 0:
                                del self.pending[uri]
                                if uri not in self.buffers:
                                    continue
                                job = Job(uri, self.generations[uri], self.buffers[uri][1], self.context, self.identity)
                                buffers = dict(self.buffers)  # Shared immutable strings, not copies of token trees.
                                invalidated = None if self.invalidate_all else self.invalidated
                                self.invalidated, self.invalidate_all = set(), False
                                cancel = threading.Event()
                                self.active = job, cancel
                                break
                            self.condition.wait(remaining)
                        else:
                            self.condition.wait()  # No polling or timer churn while idle.
                result = None
                try:
                    if invalidated is None or invalidated:
                        self.validator.invalidate(invalidated)
                    result = self.validator.validate(uri, job.context, buffers, cancel)
                except Cancelled:
                    if not cancel.is_set():
                        result = ValidationResult([], frozenset(), {}, retry=True)
                except Exception as error:
                    result = ValidationResult([unavailable(None, str(error)[:1500])], frozenset(), {})
                finally:
                    del buffers
                with self.condition:
                    self.active = None
                    valid = self.current(job) and not cancel.is_set()
                    if valid and result is not None:
                        if result.retry:
                            self.dependency_versions.pop(uri, None)
                            self.pending[uri] = time.monotonic()+self.delay
                        else:
                            self.dependencies[uri] = result.dependencies
                            self.dependency_versions[uri] = job.version
                            self.callbacks += 1
                    self.condition.notify_all()
                if valid and result is not None and not result.retry:
                    try:
                        self.publish(job, result)
                    finally:
                        with self.condition:
                            self.callbacks -= 1
                            self.condition.notify_all()
        finally:
            self.validator.close()
