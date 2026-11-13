"""Disk history independent of Qt/Houdini. No automatic pruning, ever."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import uuid
from datetime import datetime, timezone


def data_root():
    override = os.environ.get("KO_AI_HOME")
    if override:
        return Path(override).expanduser().resolve()
    return Path(os.environ.get("LOCALAPPDATA", str(Path.home() / ".local/share"))) / "ko-ai"


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, ensure_ascii=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


class WorkspaceInUseError(RuntimeError):
    """Expected ownership conflict; render as inline UI, not a traceback."""


class History:
    """Immutable snapshots; metadata changes are atomic and cleanup is explicit.

    A per-open-panel OS lock prevents two Houdini sessions writing one workspace.
    Immutable checkpoint metadata also allows recovery after an index-write crash.
    """
    def __init__(self, workspace_id, root=None):
        if not workspace_id or any(c not in "0123456789abcdef" for c in workspace_id):
            raise ValueError("Invalid workspace id")
        self.root = Path(root or data_root()) / "workspaces" / workspace_id
        self.root.mkdir(parents=True, exist_ok=True)
        self.id = workspace_id
        self.index_path = self.root / "history.json"
        self._lock = (self.root / "workspace.lock").open("a+b")
        try:
            if os.fstat(self._lock.fileno()).st_size == 0:
                self._lock.write(b"0")
                self._lock.flush()
            self._lock.seek(0)
            if os.name == "nt":
                import msvcrt
                msvcrt.locking(self._lock.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(self._lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self._lock.close()
            raise WorkspaceInUseError("Ko AI is already open for this scene in another pane or Houdini session.")
        try:
            self.index = read_json(self.index_path) if self.index_path.exists() else {
                "version": 1, "id": workspace_id, "checkpoints": [], "deleted": []}
            self._recover()
        except Exception:
            self.close()
            raise

    def close(self):
        if self._lock and not self._lock.closed:
            self._lock.close()

    def _recover(self):
        known = {entry["id"] for entry in self.entries}
        deleted = set(self.index.get("deleted", []))
        for path in sorted((self.root / "checkpoints").glob("*/checkpoint.json")):
            record = read_json(path)
            if record["id"] not in known and record["id"] not in deleted:
                self.index["checkpoints"].append(record)
                known.add(record["id"])
        # A fully saved replacement can be recovered even if its index update
        # failed. Never change descendants' parent links to point at new content.
        records = {record["id"]: record for record in self.entries}
        for record in self.entries:
            replaced = records.get(record.get("replaces"))
            if replaced:
                replaced["replaced_by"] = record["id"]
        self.flush()

    @property
    def entries(self):
        return self.index["checkpoints"]

    def flush(self):
        atomic_json(self.index_path, self.index)

    def get(self, checkpoint_id):
        for record in self.entries:
            if record["id"] == checkpoint_id:
                return record
        raise ValueError("Checkpoint not found: " + str(checkpoint_id))

    def directory(self, checkpoint_id):
        if not checkpoint_id or any(c not in "0123456789abcdef" for c in checkpoint_id):
            raise ValueError("Invalid checkpoint id")
        return self.root / "checkpoints" / checkpoint_id

    def scene_path(self, checkpoint_id):
        record = self.get(checkpoint_id)
        return self.directory(checkpoint_id) / record["file"]

    def verify(self, checkpoint_id):
        path = self.scene_path(checkpoint_id)
        if not path.is_file() or digest(path) != self.get(checkpoint_id)["sha256"]:
            raise RuntimeError("Checkpoint is missing or damaged; switch aborted.")
        return path

    def commit(self, record):
        replaced = self.get(record["replaces"]) if record.get("replaces") else None
        if replaced and (replaced.get("pinned") or replaced.get("replaced_by")):
            raise ValueError("Cannot replace a pinned or already replaced version")
        self.directory(record["id"]).mkdir(parents=True, exist_ok=True)
        atomic_json(self.directory(record["id"]) / "checkpoint.json", record)
        self.entries.append(record)
        if replaced:
            replaced["replaced_by"] = record["id"]
        self.flush()

    def rename(self, checkpoint_id, label):
        if not label.strip():
            raise ValueError("A checkpoint needs a label")
        self.get(checkpoint_id)["label"] = label.strip()
        self.flush()

    def pin(self, checkpoint_id, pinned):
        self.get(checkpoint_id)["pinned"] = bool(pinned)
        self.flush()

    def prune_except(self, keep_id, include_pinned=False):
        """Explicit bulk cleanup. Publish tombstones before removing any files.

        The kept snapshot is verified first. An index-write failure leaves both
        the in-memory history and all snapshot directories untouched. Failed file
        deletions stay queued for the next explicit cleanup, never a background job.
        """
        import shutil
        self.verify(keep_id)
        removed = [r for r in self.entries if r["id"] != keep_id]
        if not include_pinned and any(r.get("pinned") for r in removed):
            raise ValueError("Cleanup includes pinned versions; explicit confirmation is required")
        pending = list(dict.fromkeys(self.index.get("pending_cleanup", []) + [r["id"] for r in removed]))
        if keep_id in pending:
            raise ValueError("Cannot remove the retained checkpoint")
        retained = dict(self.get(keep_id))
        retained["parent"] = None
        for field in ("replaces", "replaced_by", "based_on"):
            retained.pop(field, None)
        updated = {**self.index, "checkpoints": [retained],
                   "deleted": list(dict.fromkeys(self.index.get("deleted", []) + pending)),
                   "pending_cleanup": pending}
        atomic_json(self.index_path, updated)
        self.index = updated
        errors, remaining = [], []
        for checkpoint_id in pending:
            try:
                directory = self.directory(checkpoint_id)
                if directory.exists():
                    shutil.rmtree(directory)
            except OSError as exc:
                remaining.append(checkpoint_id)
                errors.append({"id": checkpoint_id, "error": str(exc)})
        completed = {**self.index, "pending_cleanup": remaining}
        atomic_json(self.index_path, completed)
        self.index = completed
        return {"kept": keep_id, "removed": [r["id"] for r in removed], "file_errors": errors}

    def delete(self, checkpoint_id, protected=()):
        """Only called by explicit cleanup. Each snapshot is self-contained."""
        import shutil
        record = self.get(checkpoint_id)
        if checkpoint_id in protected or record.get("pinned"):
            raise ValueError("Active, return-target, or pinned checkpoints cannot be deleted")
        # Tombstone first: crash recovery must never resurrect explicitly deleted work.
        self.index.setdefault("deleted", []).append(checkpoint_id)
        self.index["checkpoints"] = [r for r in self.entries if r["id"] != checkpoint_id]
        self.flush()
        shutil.rmtree(self.directory(checkpoint_id))


def graph_diff(before, after):
    a, b = before.get("nodes", {}), after.get("nodes", {})
    result = {"created": sorted(b.keys() - a.keys()), "deleted": sorted(a.keys() - b.keys()),
              "modified": {}, "partial": before.get("partial", False) or after.get("partial", False)}
    for path in sorted(a.keys() & b.keys()):
        if a[path] != b[path]:
            changes = {}
            for key in sorted(a[path].keys() | b[path].keys()):
                if a[path].get(key) != b[path].get(key):
                    changes[key] = {"before": a[path].get(key), "after": b[path].get(key)}
            result["modified"][path] = changes
    return result
