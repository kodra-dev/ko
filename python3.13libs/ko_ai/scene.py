"""All HOM calls live here and must run on Houdini's main/UI thread."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import uuid

from .storage import History, atomic_json, digest, now, read_json

WORKSPACE_KEY = "ko_ai.workspace"
CHECKPOINT_KEY = "ko_ai.checkpoint"


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
            "inputs": [{"input": c.inputIndex(), "source": c.inputNode().path(),
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
    settings.output(str(directory / "preview.$F4.png").replace("\\", "/"))
    settings.outputToMPlay(False)
    settings.useResolution(True)
    settings.resolution((960, 540))
    viewer.flipbook(viewer.curViewport(), settings)
    images = sorted(directory.glob("preview.*.png"))
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
    def __init__(self, root=None):
        self.storage_root = root
        self.history = None
        self.active = None
        self.return_target = None
        self.refresh_binding()

    def close(self):
        if self.history:
            self.history.close()

    def refresh_binding(self):
        hou = _hou()
        workspace_id = hou.node("/").userData(WORKSPACE_KEY)
        if not workspace_id:
            workspace_id = uuid.uuid4().hex
            hou.node("/").setUserData(WORKSPACE_KEY, workspace_id)
        if self.history is None or self.history.id != workspace_id:
            self.close()
            self.history = History(workspace_id, self.storage_root)
            self.return_target = None
        checkpoint_id = hou.node("/").userData(CHECKPOINT_KEY)
        self.active = checkpoint_id if any(r["id"] == checkpoint_id for r in self.history.entries) else None

    def _remember_baseline(self):
        # hou.session attributes survive tool/module reloads, but are not written
        # into the scene's Python source. Unknown baselines are preserved safely.
        hou = _hou()
        baselines = getattr(hou.session, "_ko_ai_version_baselines", None)
        if baselines is None:
            baselines = {}
            hou.session._ko_ai_version_baselines = baselines
        baselines[self.history.id] = (self.active, scene_change_token())

    def has_live_changes(self):
        if self.active is None:
            return True
        baseline = getattr(_hou().session, "_ko_ai_version_baselines", {}).get(self.history.id)
        return baseline != (self.active, scene_change_token())

    def replace(self, checkpoint_id, preview=False):
        self.refresh_binding()
        target = self.history.get(checkpoint_id)
        if target.get("pinned"):
            raise ValueError("Unpin this version before replacing it")
        if target.get("replaced_by"):
            raise ValueError("Select the current version, not a replaced revision")
        if checkpoint_id == self.active and not self.has_live_changes():
            return target
        return self.checkpoint(target["label"], preview=preview, replaces=checkpoint_id)

    def checkpoint(self, label="Checkpoint", preview=False, replaces=None):
        hou = _hou()
        self.refresh_binding()
        replaced = self.history.get(replaces) if replaces else None
        if replaced and (replaced.get("pinned") or replaced.get("replaced_by")):
            raise ValueError("Cannot replace a pinned or already replaced version")
        checkpoint_id = uuid.uuid4().hex
        directory = self.history.directory(checkpoint_id)
        directory.mkdir(parents=True, exist_ok=False)
        source = hou.hipFile.path()
        # Preserve license-specific extension; Houdini may enforce hiplc/hipnc.
        extension = Path(source).suffix.lower()
        if extension not in (".hip", ".hiplc", ".hipnc"):
            extension = ".hip"
        if hou.licenseCategory() == hou.licenseCategoryType.Indie:
            extension = ".hiplc"
        elif hou.licenseCategory() == hou.licenseCategoryType.Apprentice:
            extension = ".hipnc"
        filename = "scene" + extension
        previous = self.active
        record = {"id": checkpoint_id, "parent": previous, "label": label.strip() or "Checkpoint",
                  "created": now(), "source_hip": source, "file": filename, "pinned": False,
                  "context": ui_context(), "preview": None}
        if replaced:
            record.update(parent=replaced.get("parent"), replaces=replaces,
                          based_on=previous)
        try:
            atomic_json(directory / "graph.json", graph_snapshot())
            if preview:
                record["preview"] = capture_preview(directory)
            hou.node("/").setUserData(CHECKPOINT_KEY, checkpoint_id)
            # mwrite -n saves a copy without changing the working filename/$HIP.
            destination = str(directory / filename).replace("\\", "/")
            if any(c in destination for c in ('"', '\n', '\r', '$', '`')):
                raise ValueError("Checkpoint storage path contains unsupported HScript characters")
            _, error = hou.hscript('mwrite -n "' + destination + '"')
            if error.strip():
                raise RuntimeError(error)
            path = directory / filename
            if not path.is_file() or path.stat().st_size == 0:
                raise RuntimeError("Houdini did not save the checkpoint")
            record["sha256"] = digest(path)
            record["bytes"] = path.stat().st_size
            self.history.commit(record)
        except Exception:
            if previous:
                hou.node("/").setUserData(CHECKPOINT_KEY, previous)
            else:
                hou.node("/").destroyUserData(CHECKPOINT_KEY, must_exist=False)
            # Leave any partial files for forensic recovery; never switch on failure.
            raise
        self.active = checkpoint_id
        self._remember_baseline()
        return record

    def visit(self, checkpoint_id):
        hou = _hou()
        self.refresh_binding()
        # Verify BEFORE preserving or loading; corruption must not affect live state.
        target = self.history.verify(checkpoint_id)
        target_record = self.history.get(checkpoint_id)
        changed = self.has_live_changes()
        if checkpoint_id == self.active and not changed:
            return {"active": self.active, "preserved": self.active,
                    "created_checkpoint": False, "unchanged": True}
        # Reuse the version being left if it is already safely checkpointed.
        # Only genuinely changed (or unknown) live states need an extra snapshot.
        preserved = (self.checkpoint("Auto-preserved before visiting " + target_record["label"])
                     if changed else self.history.get(self.active))
        recovery_path = self.history.verify(preserved["id"])
        self.return_target = preserved["id"]
        self.last_warning = ""
        try:
            self._load(target, target_record)
        except Exception as exc:
            # A failed load may have partially replaced the scene. Recover the safety copy.
            try:
                self._load(recovery_path, preserved)
            except Exception as recovery:
                raise RuntimeError(f"Switch failed: {exc}. Automatic recovery also failed: {recovery}. "
                                   f"Recovery HIP: {self.history.scene_path(preserved['id'])}") from exc
            raise RuntimeError(f"Switch failed; your preserved scene was recovered: {exc}") from exc
        self.active = checkpoint_id
        return {"active": checkpoint_id, "preserved": preserved["id"],
                "created_checkpoint": changed}

    def _load(self, path, record):
        hou = _hou()
        try:
            hou.hipFile.load(str(path), suppress_save_prompt=True, ignore_load_warnings=False)
        except hou.LoadWarning as warning:
            # Warnings mean the scene loaded, not that it failed. Surface in the UI.
            self.last_warning = str(warning)
        # Restore the logical source path, crucial for $HIP-relative dependencies.
        hou.hipFile.setName(record["source_hip"])
        hou.node("/").setUserData(WORKSPACE_KEY, self.history.id)
        hou.node("/").setUserData(CHECKPOINT_KEY, record["id"])
        self.active = record["id"]
        self._remember_baseline()

    def cleanup_plan(self):
        """Describe keep-current cleanup without saving or deleting anything."""
        self.refresh_binding()
        save_current = self.has_live_changes()
        keep_id = None if save_current else self.active
        entries = [r for r in self.history.entries if r["id"] != keep_id]
        return {"workspace": self.history.id, "active": self.active,
                "scene_token": scene_change_token(), "save_current": save_current,
                "keep_id": keep_id,
                "label": self.history.get(self.active)["label"] if self.active else "Current scene",
                "remove_ids": [r["id"] for r in entries],
                "pinned_count": sum(bool(r.get("pinned")) for r in entries),
                "bytes": sum(r.get("bytes", 0) for r in entries),
                "history_stamp": hashlib.sha256(json.dumps(self.history.index, sort_keys=True).encode()).hexdigest(),
                "pending_files": len(self.history.index.get("pending_cleanup", []))}

    def cleanup(self, plan):
        """Only invoked after explicit inline confirmation; selection is irrelevant."""
        self.refresh_binding()
        current_stamp = hashlib.sha256(json.dumps(self.history.index, sort_keys=True).encode()).hexdigest()
        if (plan["workspace"] != self.history.id or plan["active"] != self.active
                or plan["scene_token"] != scene_change_token() or plan["history_stamp"] != current_stamp):
            raise ValueError("Scene or history changed. Click Clean up again to review the updated plan.")
        # Complete a durable copy of unsaved manual work BEFORE any deletion.
        keep = self.checkpoint(plan["label"]) if plan["save_current"] else self.history.get(plan["keep_id"])
        result = self.history.prune_except(keep["id"], include_pinned=True)
        self.return_target = None
        return result

    def graph(self, checkpoint_id):
        return read_json(self.history.directory(checkpoint_id) / "graph.json")

    def save_chat(self, messages, thread_id=None):
        atomic_json(self.history.root / "chat.json", {"messages": messages, "thread_id": thread_id})

    def load_chat(self):
        path = self.history.root / "chat.json"
        return read_json(path) if path.exists() else {"messages": [], "thread_id": None}
