"""All HOM calls live here and must run on Houdini's main/UI thread."""
from __future__ import annotations

import json
import shutil
from pathlib import Path
import uuid
from datetime import datetime

from .storage import History, atomic_json, digest, now, read_json

WORKSPACE_KEY = "loom.workspace"
CHECKPOINT_KEY = "loom.checkpoint"


def migrate_identity():
    """Translate historical HIP identity keys without changing their UUIDs."""
    root = _hou().node("/")
    for key, legacy in ((WORKSPACE_KEY, "ko_ai.workspace"), (CHECKPOINT_KEY, "ko_ai.checkpoint")):
        previous = root.userData(legacy)
        if previous and not root.userData(key):
            root.setUserData(key, previous)
        if previous is not None:
            root.destroyUserData(legacy, must_exist=False)


def _hou():
    import hou
    return hou


def _safe(call, default=None):
    try:
        return call()
    except Exception:
        return default


def node_record(node):
    parms = {}
    for parm in node.parms():
        try:
            keys = parm.keyframes()
            parms[parm.name()] = {"keys": [k.asCode() for k in keys]} if keys else parm.rawValue()
        except Exception as exc:
            parms[parm.name()] = {"unreadable": str(exc)}
    return {"type": node.type().name(), "parms": parms,
            "inputs": [{"input": c.inputIndex(), "source": c.inputNode().path() if c.inputNode() is not None else None,
                        "indirect": _safe(lambda: c.subnetIndirectInput().path()),
                        "output": c.outputIndex()} for c in node.inputConnections()],
            "position": list(node.position()), "color": list(node.color().rgb()),
            "comment": node.comment(),
            "display": _safe(lambda: node.isDisplayFlagSet()), "render": _safe(lambda: node.isRenderFlagSet()),
            "bypass": _safe(lambda: node.isBypassed())}


def graph_snapshot(limit=20000):
    hou = _hou()
    nodes = {}
    partial = False
    # This is a graph diff aid, not a lossless HIP serializer. No geometry cooks.
    stack = list(reversed(hou.node("/").children()))
    while stack:
        node = stack.pop()
        if len(nodes) >= limit:
            partial = True
            break
        nodes[node.path()] = node_record(node)
        stack.extend(reversed(node.children()))
    return {"nodes": nodes, "partial": partial, "frame": hou.frame(), "fps": hou.fps()}


def ui_context():
    hou = _hou()
    result = {"hip": hou.hipFile.path(), "houdini": hou.applicationVersionString(),
              "frame": hou.frame(), "selection": [n.path() for n in hou.selectedNodes()],
              "panes": [], "top_level": [{"path": n.path(), "type": n.type().name()}
                                              for n in hou.node("/").children()]}
    if not hou.isUIAvailable():
        return result
    for pane in hou.ui.paneTabs():
        if not _safe(pane.isCurrentTab, False):
            continue
        item = {"type": str(pane.type()), "name": pane.name()}
        pwd = _safe(lambda: pane.pwd().path())
        if pwd:
            item["network"] = pwd
        current = _safe(lambda: pane.currentNode().path())
        if current:
            item["node"] = current
        item["pinned"] = _safe(pane.isPin)
        if pane.type() == hou.paneTabType.NetworkEditor:
            bounds = _safe(pane.visibleBounds)
            if bounds and pwd:
                visible = [n.path() for n in pane.pwd().children()
                           if bounds.contains(n.position())]
                item["visible_nodes"] = visible[:100]
                item["visible_nodes_truncated"] = len(visible) > 100
        if pane.type() == hou.paneTabType.SceneViewer:
            viewport = _safe(pane.curViewport)
            if viewport:
                item["camera"] = _safe(lambda: viewport.camera().path())
                item["view_transform"] = _safe(lambda: list(viewport.viewTransform().asTuple()))
        result["panes"].append(item)
    return result


def capture_preview(directory):
    hou = _hou()
    if not hou.isUIAvailable():
        raise RuntimeError("Viewport previews require graphical Houdini")
    viewer = hou.ui.paneTabOfType(hou.paneTabType.SceneViewer)
    if viewer is None:
        raise RuntimeError("Open a Scene Viewer pane first")
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    settings = viewer.flipbookSettings().stash()
    settings.frameRange((hou.frame(), hou.frame()))
    prefix = "preview-" + uuid.uuid4().hex
    settings.output(str(directory / (prefix + ".$F4.png")).replace("\\", "/"))
    settings.outputToMPlay(False)
    settings.useResolution(True)
    settings.resolution((960, 540))
    viewer.flipbook(viewer.curViewport(), settings)
    images = sorted(directory.glob(prefix + ".*.png"))
    if not images:
        raise RuntimeError("Houdini did not produce a preview image")
    return images[0].name


def scene_change_token():
    """Conservative session-local change token, including non-graph scene edits.

    Houdini advances the root node's modification time for descendant edits,
    including user data and network items absent from our semantic graph diff.
    Capture AFTER save/load bookkeeping, not from the checkpoint's old timestamp.
    Never use hasUnsavedChanges alone: save-copy and headless sessions stay dirty.
    """
    hou = _hou()
    root = hou.node("/")
    return (root.sessionId(), root.modificationTime().isoformat(), hou.hipFile.path(),
            hou.frame(), hou.fps(), tuple(hou.playbar.frameRange()),
            tuple(hou.playbar.playbackRange()))


class Workspace:
    """Editable scene versions. The active ID stays stable when its scene is saved."""
    def __init__(self, root=None):
        self.storage_root = root
        self.history = None
        self.active = None
        self._closed = False
        self._writing = False
        self._pending_native_save = None
        self.save_warning = ""
        self.last_warning = ""
        self.refresh_binding()
        _hou().hipFile.addEventCallback(self._on_hip_event)

    def close(self):
        if not self._closed:
            self._closed = True
            self._pending_native_save = None
            if self._on_hip_event in _hou().hipFile.eventCallbacks():
                _hou().hipFile.removeEventCallback(self._on_hip_event)
            if self.history:
                self.history.close()

    def refresh_binding(self):
        hou = _hou()
        migrate_identity()
        workspace_id = hou.node("/").userData(WORKSPACE_KEY)
        if not workspace_id:
            workspace_id = uuid.uuid4().hex
            hou.node("/").setUserData(WORKSPACE_KEY, workspace_id)
        if self.history is None or self.history.id != workspace_id:
            if self.history:
                self.history.close()
            self.history = History(workspace_id, self.storage_root)
            self._pending_native_save = None
        checkpoint_id = hou.node("/").userData(CHECKPOINT_KEY)
        self.active = checkpoint_id if any(r["id"] == checkpoint_id for r in self.history.entries) else None

    def _on_hip_event(self, event_type, **kwargs):
        hou = _hou()
        if self._closed or self._writing:
            return
        if event_type == hou.hipFileEventType.AfterLoad:
            # Translate before the next native Save, not afterward (which would
            # dirty the just-saved document). Do not rebind an active agent turn.
            migrate_identity()
            return
        if event_type != hou.hipFileEventType.AfterSave:
            return
        if not hou.node("/").userData(CHECKPOINT_KEY) or not hou.node("/").userData(WORKSPACE_KEY):
            return
        try:
            self.refresh_binding()
            if self.active:
                self._pending_native_save = {
                    "identity": (self.history.id, self.active, hou.hipFile.path()),
                    "token": scene_change_token(), "graph": graph_snapshot(),
                    "context": ui_context(), "updated": now()}
                if hou.isUIAvailable():
                    from PySide6 import QtCore
                    QtCore.QTimer.singleShot(0, self.process_pending_save)
        except Exception as exc:
            self.save_warning = "Working HIP saved, but checkpoint synchronization failed: " + str(exc)

    def process_pending_save(self):
        pending = self._pending_native_save
        self._pending_native_save = None
        if not pending or self._closed:
            return
        hou = _hou()
        identity = (hou.node("/").userData(WORKSPACE_KEY),
                    hou.node("/").userData(CHECKPOINT_KEY), hou.hipFile.path())
        if identity != pending["identity"]:
            return  # Never apply a deferred Save to a different scene/version.
        try:
            # Copy the already saved working HIP, rather than mwrite -n: Houdini
            # marks the document dirty again when saving a copy via mwrite.
            self._copy_native_save(pending)
            self.save_warning = ""
        except Exception as exc:
            self.save_warning = "Working HIP saved, but its checkpoint could not be updated: " + str(exc)

    def _copy_native_save(self, pending):
        checkpoint_id = pending["identity"][1]
        source = Path(pending["identity"][2])
        old = self.history.get(checkpoint_id)
        directory = self.history.directory(checkpoint_id)
        generation = uuid.uuid4().hex
        filename = "scene-" + generation + source.suffix
        graph_file = "graph-" + generation + ".json"
        shutil.copyfile(source, directory / filename)
        if not (directory / filename).stat().st_size:
            raise RuntimeError("Working HIP copy is empty")
        atomic_json(directory / graph_file, pending["graph"])
        record = {**old, "file": filename, "graph_file": graph_file,
                  "source_hip": str(source), "context": pending["context"],
                  "updated": pending["updated"], "preview": None,
                  "sha256": digest(directory / filename), "bytes": (directory / filename).stat().st_size}
        self.history.update(record)
        self.last_warning = getattr(self.history, "metadata_warning", "")
        if scene_change_token() == pending["token"]:
            self._remember_baseline()

    def _remember_baseline(self):
        hou = _hou()
        baselines = getattr(hou.session, "_loom_version_baselines", None)
        if baselines is None:
            baselines = {}
            hou.session._loom_version_baselines = baselines
        baselines[self.history.id] = (self.active, scene_change_token())

    def has_live_changes(self):
        if self.active is None:
            return True
        baseline = getattr(_hou().session, "_loom_version_baselines", {}).get(self.history.id)
        return baseline != (self.active, scene_change_token())

    def _write(self, checkpoint_id, template, existing):
        """Stage new files, then publish their metadata. Never overwrite a valid HIP."""
        hou = _hou()
        directory = self.history.directory(checkpoint_id)
        directory.mkdir(parents=True, exist_ok=True)
        source = hou.hipFile.path()
        extension = Path(source).suffix.lower()
        if extension not in (".hip", ".hiplc", ".hipnc"):
            extension = ".hip"
        if hou.licenseCategory() == hou.licenseCategoryType.Indie:
            extension = ".hiplc"
        elif hou.licenseCategory() == hou.licenseCategoryType.Apprentice:
            extension = ".hipnc"
        generation = uuid.uuid4().hex
        filename = "scene-" + generation + extension
        graph_file = "graph-" + generation + ".json"
        previous = self.active
        record = {**template, "id": checkpoint_id, "updated": now(),
                  "source_hip": source, "file": filename, "graph_file": graph_file,
                  "context": ui_context(), "preview": None}
        self._writing = True
        try:
            atomic_json(directory / graph_file, graph_snapshot())
            if hou.node("/").userData(CHECKPOINT_KEY) != checkpoint_id:
                hou.node("/").setUserData(CHECKPOINT_KEY, checkpoint_id)
            destination = str(directory / filename).replace("\\", "/")
            if any(c in destination for c in ('"', '\n', '\r', '$', '`')):
                raise ValueError("Checkpoint storage path contains unsupported HScript characters")
            _, error = hou.hscript('mwrite -n "' + destination + '"')
            # mwrite sends non-fatal parameter-reference warnings to stderr too.
            # Do not cancel a visit after a successful save just for a warning;
            # unknown/mixed diagnostics remain fatal, as do missing/empty files.
            diagnostics = [line.strip() for line in error.splitlines() if line.strip()]
            write_warning = error.strip() if diagnostics else ""
            if any(not line.startswith("Warning:") for line in diagnostics):
                raise RuntimeError(error)
            path = directory / filename
            if not path.is_file() or path.stat().st_size == 0:
                raise RuntimeError("Houdini did not save the checkpoint")
            record["sha256"] = digest(path)
            record["bytes"] = path.stat().st_size
            if existing:
                self.history.update(record)
            else:
                self.history.commit(record)
        except Exception:
            if previous:
                if hou.node("/").userData(CHECKPOINT_KEY) != previous:
                    hou.node("/").setUserData(CHECKPOINT_KEY, previous)
            else:
                hou.node("/").destroyUserData(CHECKPOINT_KEY, must_exist=False)
            # Existing index/files remain valid. Partial staging files are not versions.
            raise
        finally:
            self._writing = False
        self.active = checkpoint_id
        self._pending_native_save = None
        self.last_warning = "\n".join(filter(None, (write_warning, getattr(self.history, "metadata_warning", ""))))
        self.save_warning = ""
        self._remember_baseline()
        return record

    def save_current(self, force=False):
        self.refresh_binding()
        if not self.active:
            raise ValueError("Create a checkpoint first")
        record = self.history.get(self.active)
        if not force and not self.has_live_changes():
            return record
        return self._write(self.active, record, existing=True)

    def checkpoint(self, label=None, preview=False, replaces=None, *, created_by="user"):
        if created_by not in ("user", "ai"):
            raise ValueError("created_by must be user or ai")
        if replaces:
            return self.replace(replaces, preview=preview)
        self.refresh_binding()
        # A new version branches from the up-to-date current version.
        if self.active:
            self.save_current()
        title = (label or "").strip() or datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        if created_by == "ai":
            while title.lower().startswith(("ai:", "before:", "after:")):
                title = title.split(":", 1)[1].strip()
            title = " ".join(title.split()[:6]) or datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            if len(title) > 40:
                title = title[:37].rsplit(" ", 1)[0] + "…" if " " in title[:37] else title[:39] + "…"
        template = {"parent": self.active,
                    "label": title,
                    "created": now(), "created_by": created_by}
        record = self._write(uuid.uuid4().hex, template, existing=False)
        if preview:
            return self.capture_current_preview()
        return record

    def replace(self, checkpoint_id, preview=False):
        self.refresh_binding()
        target = dict(self.history.get(checkpoint_id))
        if checkpoint_id == self.active:
            record = self.save_current()
        else:
            if self.active:
                self.save_current()
            record = self._write(checkpoint_id, target, existing=True)
        return self.capture_current_preview() if preview else record

    def capture_current_preview(self):
        """Save first, capture the current version, then attach only a matching still."""
        self.save_current(force=True)
        checkpoint_id = self.active
        workspace_id = self.history.id
        token = scene_change_token()
        preview = capture_preview(self.history.directory(checkpoint_id))
        if (_hou().node("/").userData(WORKSPACE_KEY) != workspace_id
                or _hou().node("/").userData(CHECKPOINT_KEY) != checkpoint_id):
            raise RuntimeError("Scene changed during preview capture")
        # Capturing may cook geometry or invoke scene callbacks; save their results too.
        if scene_change_token() != token:
            self.save_current(force=True)
        record = {**self.history.get(checkpoint_id), "preview": preview, "context": ui_context()}
        self.history.update(record)
        return record

    def visit(self, checkpoint_id):
        hou = _hou()
        self.refresh_binding()
        if checkpoint_id == self.active:
            return {"active": self.active, "unchanged": True, "updated_previous": False}
        # Verify the destination BEFORE saving anything or changing the scene.
        target = self.history.verify(checkpoint_id)
        target_record = dict(self.history.get(checkpoint_id))
        if not self.active:
            raise ValueError("Create a checkpoint for the current scene before visiting another one")
        previous = self.active
        changed = self.has_live_changes()
        self.last_warning = ""
        self.save_current()
        recovery_record = dict(self.history.get(previous))
        recovery_path = self.history.verify(previous)
        working_path = hou.hipFile.path()
        try:
            self._load(target, target_record, working_path)
        except Exception as exc:
            try:
                self._load(recovery_path, recovery_record, working_path)
            except Exception as recovery:
                raise RuntimeError(f"Switch failed: {exc}. Recovery also failed: {recovery}. "
                                   f"Recovery HIP: {recovery_path}") from exc
            raise RuntimeError(f"Switch failed; your current checkpoint was recovered: {exc}") from exc
        return {"active": checkpoint_id, "unchanged": False, "updated_previous": changed}

    def _load(self, path, record, working_path=None):
        hou = _hou()
        try:
            hou.hipFile.load(str(path), suppress_save_prompt=True, ignore_load_warnings=False)
        except hou.LoadWarning as warning:
            self.last_warning = "\n".join(filter(None, (self.last_warning, str(warning))))
        # A version switch must not undo Save As or redirect the next Ctrl+S.
        hou.hipFile.setName(working_path or record["source_hip"])
        hou.node("/").setUserData(WORKSPACE_KEY, self.history.id)
        hou.node("/").setUserData(CHECKPOINT_KEY, record["id"])
        migrate_identity()
        self.active = record["id"]
        self._remember_baseline()

    def delete(self, checkpoint_id):
        self.refresh_binding()
        return self.history.delete(checkpoint_id, protected=[self.active])

    def delete_descendants(self, checkpoint_id):
        self.refresh_binding()
        return self.history.delete_descendants(checkpoint_id, protected=[self.active])

    def cleanup(self):
        """Explicit one-action cleanup: save/verify current, then delete all others."""
        self.refresh_binding()
        keep = self.save_current(force=True) if self.active else self.checkpoint()
        self.history.verify(keep["id"])
        return self.history.prune_except(keep["id"], include_pinned=True)

    def graph(self, checkpoint_id):
        record = self.history.get(checkpoint_id)
        return read_json(self.history.directory(checkpoint_id) / record.get("graph_file", "graph.json"))

    def save_chat(self, messages, thread_id=None):
        atomic_json(self.history.root / "chat.json", {"messages": messages, "thread_id": thread_id})

    def load_chat(self):
        path = self.history.root / "chat.json"
        return read_json(path) if path.exists() else {"messages": [], "thread_id": None}
