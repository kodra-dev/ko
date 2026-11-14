"""Package-wide reload coordination. Never reload a controller beneath live views."""
import importlib
from pathlib import Path
import sys

MODULES = (
    "paths", "checkpoints.storage", "window_state", "checkpoints.scene", "checkpoints.service", "checkpoints.actions",
    "checkpoints.ui", "checkpoints.panel", "checkpoints",
    "instructions", "local", "chat", "inspection", "network", "tools", "provider", "usage", "panel",
)
INTERFACES = {"agent": ("loom_workspace", "loom.pypanel"),
              "checkpoints": ("loom_checkpoints", "loom_checkpoints.pypanel")}


def _content(root):
    return getattr(root, "_content", None) or root


def _state(kind, root):
    if kind == "checkpoints" and hasattr(root, "selected_ids"):
        return {"selected": root.selected_ids(), "legacy": root.show_replaced_check.isChecked(),
                "columns": bytes(root.versions.header().saveState().toHex()).decode("ascii")}
    return {}


def _restore(kind, root, state):
    if kind == "checkpoints" and hasattr(root, "versions"):
        from PySide6 import QtCore, QtWidgets
        root.show_replaced_check.setChecked(state.get("legacy", False))
        if state.get("columns"):
            root.versions.header().restoreState(QtCore.QByteArray.fromHex(state["columns"].encode("ascii")))
        # Old saved UI state used Stretch/ResizeToContents: widths survive, fixed
        # resize policies do not. All columns must remain draggable after upgrade.
        root.versions.header().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Interactive)
        iterator = QtWidgets.QTreeWidgetItemIterator(root.versions)
        while iterator.value():
            item = iterator.value()
            item.setSelected(item.data(0, QtCore.Qt.ItemDataRole.UserRole) in state.get("selected", []))
            iterator += 1
    content = _content(root)
    if hasattr(content, "status"):
        content.status.setText("Loom + Checkpoints reloaded." +
                               (" Reconnecting automatically." if kind == "agent" else ""))


def _kind(pane):
    import hou
    import loom
    if pane.type() != hou.paneTabType.PythonPanel:
        return None
    interface = pane.activeInterface()
    if interface is not None:
        if interface.name() == "ko_ai_workspace":
            return "agent"
        return next((kind for kind, (name, _) in INTERFACES.items() if interface.name() == name), None)
    # Recover our unbound native interfaces without claiming unrelated Python Panels.
    if loom._is_loom_pane(pane):
        return "agent"
    root = pane.activeInterfaceRootWidget()
    names = getattr(hou.session, "_loom_checkpoint_pane_names", set())
    if (pane.name() in names or pane.label() == "Checkpoints"
            or (root is not None and hasattr(root, "versions") and hasattr(root, "workspace"))):
        return "checkpoints"
    return None


def _definition(kind):
    import hou
    import xml.etree.ElementTree as ET
    name, filename = INTERFACES[kind]
    path = Path(__file__).resolve().parents[2] / "python_panels" / filename
    script = ET.parse(path).find("interface/script").text
    compile(script, str(path), "exec")
    definition = hou.pypanel.interfaceByName(name)
    if definition is None:
        hou.pypanel.installFile(str(path))
        definition = hou.pypanel.interfaceByName(name)
    else:
        # Reinstalling a live interface file can invalidate Houdini's binding.
        definition.setScript(script)
    return definition


def reload_package():
    """Reload all loaded Loom features, preserving panes, chat and live HIP edits.

    No HIP load/save or new checkpoint is performed. Syntax and busy-state checks
    run before teardown. Optional features that haven't loaded stay unloaded.
    """
    import hou
    session = getattr(hou.session, "_loom_checkpoint_session", None)
    if session:
        session.require_idle()
    native, floating = [], []
    if hou.isUIAvailable():
        for pane in hou.ui.paneTabs():
            kind = _kind(pane)
            if kind:
                root = pane.activeInterfaceRootWidget()
                native.append({"kind": kind, "pane": pane, "root": root,
                               "state": _state(kind, root)})
    for kind, name in (("agent", "loom.panel"), ("checkpoints", "loom.checkpoints")):
        module = sys.modules.get(name)
        window = getattr(module, "_window", None) if module else None
        if window is not None:
            root = (getattr(window, "_loom_host", None) or getattr(window, "_ko_ai_host", None)) if kind == "agent" else getattr(window, "_loom_checkpoint_panel", None)
            if root is not None and not getattr(root, "_closed", False):
                floating.append({"kind": kind, "root": root, "visible": window.isVisible(),
                                 "state": _state(kind, root)})
    views = native + floating
    known_clients = {_content(view["root"]) for view in views if view["root"] is not None}
    if any(getattr(client, "busy", False) for client in known_clients):
        raise RuntimeError("Stop the active Loom turn before reloading the package.")
    if session and session.clients - known_clients:
        raise RuntimeError("Close standalone developer-created Loom widgets before reloading the package.")
    loaded = ["loom." + name for name in MODULES if "loom." + name in sys.modules]
    # A syntax error in an edited file must not close the working panes.
    for name in loaded:
        path = Path(sys.modules[name].__file__)
        compile(path.read_text(encoding="utf-8"), str(path), "exec")
    definitions = {kind: _definition(kind) for kind in {view["kind"] for view in native}}
    if session:
        session.workspace.process_pending_save()
    for view in views:
        content = _content(view["root"])
        if hasattr(content, "_save_chat"):
            content._save_chat()  # Fail before teardown if the transcript cannot be saved.
    # Recreate the real agent owner before any duplicate-pane notices.
    native.sort(key=lambda view: not bool(getattr(view["root"], "_content", None)))
    hou.session._loom_package_reloading = True
    try:
        for view in views:
            root = view["root"]
            from .window_state import flush
            flush(root)
            if root is not None and hasattr(root, "shutdown"):
                root.shutdown()
        agent_module = sys.modules.get("loom.panel")
        if agent_module:
            agent_module.close_panel()
        checkpoint_module = sys.modules.get("loom.checkpoints")
        if checkpoint_module:
            if hasattr(checkpoint_module, "close"):
                checkpoint_module.close()
            elif getattr(checkpoint_module, "_window", None) is not None:
                checkpoint_module._window.close()
        # The action registry preserves its registrations during module reload.
        # Do not load the agent for checkpoints alone.
        for name in loaded:
            importlib.reload(sys.modules[name])
        for view in sorted(views, key=lambda entry: not bool(getattr(entry["root"], "_content", None))):
            if "pane" in view:
                pane = view["pane"]
                definition = definitions[view["kind"]]
                active = pane.activeInterface()
                if active is None or active.id() != definition.id():
                    pane.setActiveInterface(definition)
                else:
                    pane.reloadActiveInterface()
                _restore(view["kind"], pane.activeInterfaceRootWidget(), view["state"])
            else:
                module = importlib.import_module("loom" if view["kind"] == "agent" else "loom.checkpoints")
                window = module.show()
                root = window._loom_host if view["kind"] == "agent" else window._loom_checkpoint_panel
                _restore(view["kind"], root, view["state"])
                if not view["visible"]:
                    window.hide()
    finally:
        hou.session._loom_package_reloading = False
    if hou.isUIAvailable():
        hou.ui.setStatusMessage("Reloaded Loom + Checkpoints.")
    return {"modules": loaded, "panes": len(native), "windows": len(floating)}
