"""Loom: a Houdini agent harness with independent scene checkpoints."""
__version__ = "0.1.0"


def show():
    from .panel import show_panel
    return show_panel()


def reload_modules():
    """Close the floating UI/transport before reloading; disk history survives."""
    import importlib
    import sys
    module = sys.modules.get(__name__ + ".panel")
    if module:
        module.close_panel()
    # Shared checkpoint controllers and action registries must survive agent reloads.
    for name in ("instructions", "tools", "provider", "usage", "panel"):
        module = sys.modules.get(__name__ + "." + name)
        if module:
            importlib.reload(module)


def _interface_definition():
    """Update in place: reinstalling an active .pypanel invalidates its binding."""
    import hou
    from pathlib import Path
    import xml.etree.ElementTree as ET
    path = Path(__file__).resolve().parents[2] / "python_panels/loom.pypanel"
    script = ET.parse(path).find("interface[@name='loom_workspace']/script").text
    compile(script, str(path), "exec")
    interface = hou.pypanel.interfaceByName("loom_workspace")
    if interface is None:
        hou.pypanel.installFile(str(path))
        interface = hou.pypanel.interfaceByName("loom_workspace")
    else:
        interface.setScript(script)
    return interface


def _remember_pane(pane):
    """Keep names, not invalidatable native pane objects, across module reloads."""
    import hou
    names = getattr(hou.session, "_loom_pane_names", None)
    if names is None:
        names = set()
        hou.session._loom_pane_names = names
    names.add(pane.name())


def _is_loom_pane(pane):
    import hou
    if pane.type() != hou.paneTabType.PythonPanel:
        return False
    interface = pane.activeInterface()
    if interface is not None:
        return interface.name() in ("loom_workspace", "ko_ai_workspace")
    if pane.name() in (getattr(hou.session, "_loom_pane_names", set())
                      | getattr(hou.session, "_ko_ai_pane_names", set())):
        return True
    root = pane.activeInterfaceRootWidget()
    # Older reloads can leave a valid-looking widget but no bound interface.
    if root is not None:
        try:
            if root.objectName() in ("loom_host", "loom_panel", "ko_ai_host", "ko_ai_panel"):
                return True
        except RuntimeError:
            pass
    return pane.label() in ("Loom", "Ko AI")


def reload():
    """Reload the Loom package, including all open Checkpoints and agent views."""
    import importlib
    import sys
    importlib.reload(sys.modules[__name__])
    from . import runtime
    return importlib.reload(runtime).reload_package()


def request_reload():
    """Menu entry point: defer until the menu closes, and keep errors non-modal."""
    from PySide6 import QtCore
    def run():
        try:
            reload()
        except Exception as exc:
            import hou
            import traceback
            hou.ui.setStatusMessage("Loom package reload failed: " + str(exc), severity=hou.severityType.Error)
            traceback.print_exc()
    QtCore.QTimer.singleShot(0, run)


def _reload_agent_pane():
    """Repair a legacy agent interface locally; user-facing reload is package-wide."""
    import hou
    if hou.isUIAvailable():
        tabs = hou.ui.paneTabs()
        candidates = [pane for pane in tabs if _is_loom_pane(pane)]
        if not candidates:
            # Migration/recovery for versions which lost their binding before we
            # recorded pane identity. Only infer when there is one unbound panel.
            unbound = [pane for pane in tabs if pane.type() == hou.paneTabType.PythonPanel
                       and pane.activeInterface() is None]
            if len(unbound) == 1:
                candidates = unbound
        # Prefer the real controller, including pre-host versions of the tool,
        # over a duplicate-pane notice or Houdini's old Python-error placeholder.
        def owns_controller(pane):
            root = pane.activeInterfaceRootWidget()
            if root is None:
                return False
            try:
                return (getattr(root, "_content", None) is not None
                        or (root.objectName() in ("loom_panel", "ko_ai_panel") and not getattr(root, "_closed", True)))
            except RuntimeError:
                return False
        candidates.sort(key=lambda pane: not owns_controller(pane))
        for pane in candidates:
            widget = pane.activeInterfaceRootWidget()
            if widget and getattr(widget, "busy", False):
                if hasattr(widget, "show_message"):
                    widget.show_message("Stop the active turn before reloading Loom.")
                elif hasattr(widget, "status"):
                    widget.status.setText("Stop the active turn before reloading Loom.")
                return pane
            definition = _interface_definition()
            _remember_pane(pane)
            if widget and hasattr(widget, "shutdown"):
                widget.shutdown()
            active = pane.activeInterface()
            if active is None or active.id() != definition.id():
                pane.setActiveInterface(definition)
            else:
                pane.reloadActiveInterface()
            return pane
    reload_modules()
    return show()


def toggle_pane():
    """Top-menu toggle; never close or move an existing docked pane."""
    import hou
    from .window_state import flush
    for pane in hou.ui.paneTabs():
        if _is_loom_pane(pane) and pane.floatingPanel() is not None:
            flush(pane.activeInterfaceRootWidget())
            pane.floatingPanel().close()
            return None
    return open_pane()


def open_pane():
    """Install/open the native Python Panel immediately, without a Houdini restart."""
    import hou
    if not hou.isUIAvailable():
        raise RuntimeError("The agent pane requires graphical Houdini")
    for pane in hou.ui.paneTabs():
        if _is_loom_pane(pane):
            if pane.activeInterface() is None or pane.activeInterface().name() == "ko_ai_workspace":
                _reload_agent_pane()
            pane.setIsCurrentTab()
            return pane
    module = __import__(__name__ + ".panel", fromlist=["close_panel"])
    module.close_panel()
    definition = _interface_definition()
    from .window_state import geometry, restore, track
    saved = geometry("agent")
    pane = hou.ui.curDesktop().createFloatingPaneTab(
        hou.paneTabType.PythonPanel, python_panel_interface=definition.name(), immediate=True, **saved)
    restore(pane, saved)
    _remember_pane(pane)
    track("agent", pane, pane.activeInterfaceRootWidget())
    return pane
