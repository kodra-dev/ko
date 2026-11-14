"""Editable checkpoint storage independent of Qt/Houdini; deletion is explicit."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import uuid
from datetime import datetime, timezone


from ..paths import data_root


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
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


class WorkspaceInUseError(RuntimeError):
    """Expected ownership conflict; render as inline UI, not a traceback."""


class History:
    """Self-contained scene versions, with atomic metadata publication.

    New scene/graph files are staged before the authoritative index is updated.
    A session OS lock prevents simultaneous writers to one workspace.
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
            raise WorkspaceInUseError("This checkpoint workspace is already open in another Houdini session.")
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
        if replaced and replaced.get("replaced_by"):
            raise ValueError("Cannot replace an already replaced version")
        self.directory(record["id"]).mkdir(parents=True, exist_ok=True)
        atomic_json(self.directory(record["id"]) / "checkpoint.json", record)
        records = [dict(r) for r in self.entries] + [record]
        if replaced:
            next(r for r in records if r["id"] == replaced["id"])["replaced_by"] = record["id"]
        updated = {**self.index, "checkpoints": records}
        atomic_json(self.index_path, updated)
        self.index = updated
        if replaced:
            replaced["replaced_by"] = record["id"]

    def update(self, record):
        """Publish an existing checkpoint without risking its previous saved files."""
        old = self.get(record["id"])
        updated = {**self.index, "checkpoints": [
            record if r["id"] == record["id"] else r for r in self.entries]}
        atomic_json(self.index_path, updated)
        self.index = updated
        self.metadata_warning = ""
        try:
            atomic_json(self.directory(record["id"]) / "checkpoint.json", record)
        except OSError as exc:
            # Index is authoritative. Keep old files if the recovery manifest
            # couldn't be synchronized; never report a committed save as failed.
            self.metadata_warning = "Scene saved; recovery metadata could not be refreshed: " + str(exc)
            return
        for field, fallback in (("file", None), ("graph_file", "graph.json"), ("preview", None)):
            previous = old.get(field, fallback)
            if previous and previous != record.get(field, fallback):
                try:
                    (self.directory(record["id"]) / previous).unlink(missing_ok=True)
                except OSError:
                    pass  # A locked obsolete file is harmless; never lose the new save.

    def rename(self, checkpoint_id, label):
        if not label.strip():
            raise ValueError("A checkpoint needs a label")
        self.update({**self.get(checkpoint_id), "label": label.strip()})

    def pin(self, checkpoint_id, pinned):
        self.get(checkpoint_id)["pinned"] = bool(pinned)
        self.flush()

    def _remove(self, removed_ids, retained):
        """Publish tombstones/reparenting before touching any checkpoint folders."""
        import shutil
        pending = list(dict.fromkeys(self.index.get("pending_cleanup", []) + list(removed_ids)))
        if set(pending) & {r["id"] for r in retained}:
            raise ValueError("Cannot remove a retained checkpoint")
        updated = {**self.index, "checkpoints": retained,
                   "deleted": list(dict.fromkeys(self.index.get("deleted", []) + list(removed_ids))),
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
        try:
            atomic_json(self.index_path, completed)
            self.index = completed
        except OSError as exc:
            errors.append({"error": "Deleted entries are committed; cleanup bookkeeping will retry: " + str(exc)})
        return {"removed": list(removed_ids), "file_errors": errors}

    def prune_except(self, keep_id, include_pinned=False):
        self.verify(keep_id)
        removed = [r for r in self.entries if r["id"] != keep_id]
        if not include_pinned and any(r.get("pinned") for r in removed):
            raise ValueError("Cleanup includes legacy pinned versions")
        retained = dict(self.get(keep_id))
        retained["parent"] = None
        for field in ("replaces", "replaced_by", "based_on"):
            retained.pop(field, None)
        return {"kept": keep_id, **self._remove([r["id"] for r in removed], [retained])}

    def descendants(self, checkpoint_id):
        self.get(checkpoint_id)
        children = {}
        for record in self.entries:
            children.setdefault(record.get("parent"), []).append(record["id"])
        found, pending = set(), list(children.get(checkpoint_id, ()))
        while pending:
            current = pending.pop()
            if current == checkpoint_id or current in found:
                continue
            found.add(current)
            pending.extend(children.get(current, ()))
        return sorted(found)

    def delete_descendants(self, checkpoint_id, protected=()):
        """Delete all children/grandchildren in one atomic index publication."""
        removed = self.descendants(checkpoint_id)
        if set(removed) & set(protected):
            raise ValueError("Cannot delete descendants containing the current checkpoint.")
        if not removed:
            return {"removed": [], "file_errors": []}
        retained = []
        for record in self.entries:
            if record["id"] in removed:
                continue
            item = dict(record)
            for field in ("replaces", "replaced_by", "based_on"):
                if item.get(field) in removed:
                    item.pop(field, None)
            retained.append(item)
        return self._remove(removed, retained)

    def delete(self, checkpoint_id, protected=()):
        """Delete one version only; its children move up one level."""
        record = self.get(checkpoint_id)
        if checkpoint_id in protected:
            raise ValueError("Cannot delete the current checkpoint. Visit another checkpoint first.")
        parent = record.get("parent")
        retained = []
        for entry in self.entries:
            if entry["id"] == checkpoint_id:
                continue
            item = dict(entry)
            if item.get("parent") == checkpoint_id:
                item["parent"] = parent
            for field in ("replaces", "replaced_by", "based_on"):
                if item.get(field) == checkpoint_id:
                    item.pop(field, None)
            retained.append(item)
        return self._remove([checkpoint_id], retained)


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
