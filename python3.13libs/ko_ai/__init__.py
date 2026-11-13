"""Ko AI: a reloadable, versioned Houdini agent workspace."""
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
    for name in ("storage", "scene", "tools", "provider", "panel"):
        module = sys.modules.get(__name__ + "." + name)
        if module:
            importlib.reload(module)


def _interface_definition():
    """Update in place: reinstalling an active .pypanel invalidates its binding."""
    import hou
    from pathlib import Path
    import xml.etree.ElementTree as ET
    path = Path(__file__).resolve().parents[2] / "python_panels/ko_ai.pypanel"
    script = ET.parse(path).find("interface[@name='ko_ai_workspace']/script").text
    compile(script, str(path), "exec")
    interface = hou.pypanel.interfaceByName("ko_ai_workspace")
    if interface is None:
        hou.pypanel.installFile(str(path))
        interface = hou.pypanel.interfaceByName("ko_ai_workspace")
    else:
        interface.setScript(script)
    return interface


def _remember_pane(pane):
    """Keep names, not invalidatable native pane objects, across module reloads."""
    import hou
    names = getattr(hou.session, "_ko_ai_pane_names", None)
    if names is None:
        names = set()
        hou.session._ko_ai_pane_names = names
    names.add(pane.name())


def _is_ko_pane(pane):
    import hou
    if pane.type() != hou.paneTabType.PythonPanel:
        return False
    interface = pane.activeInterface()
    if interface is not None:
        return interface.name() == "ko_ai_workspace"
    if pane.name() in getattr(hou.session, "_ko_ai_pane_names", set()):
        return True
    root = pane.activeInterfaceRootWidget()
    # Older reloads can leave a valid-looking widget but no bound interface.
    if root is not None:
        try:
            if root.objectName() in ("ko_ai_host", "ko_ai_panel"):
                return True
        except RuntimeError:
            pass
    return pane.label() == "Ko AI"


def reload():
    """Reload the existing embedded panel, or reopen the floating workspace."""
    import hou
    if hou.isUIAvailable():
        tabs = hou.ui.paneTabs()
        candidates = [pane for pane in tabs if _is_ko_pane(pane)]
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
                        or (root.objectName() == "ko_ai_panel" and not getattr(root, "_closed", True)))
            except RuntimeError:
                return False
        candidates.sort(key=lambda pane: not owns_controller(pane))
        for pane in candidates:
            widget = pane.activeInterfaceRootWidget()
            if widget and getattr(widget, "busy", False):
                if hasattr(widget, "show_message"):
                    widget.show_message("Stop the active turn before reloading Ko AI.")
                elif hasattr(widget, "status"):
                    widget.status.setText("Stop the active turn before reloading Ko AI.")
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


def open_pane():
    """Install/open the native Python Panel immediately, without a Houdini restart."""
    import hou
    if not hou.isUIAvailable():
        raise RuntimeError("The agent pane requires graphical Houdini")
    for pane in hou.ui.paneTabs():
        if _is_ko_pane(pane):
            if pane.activeInterface() is None:
                reload()
            pane.setIsCurrentTab()
            return pane
    module = __import__(__name__ + ".panel", fromlist=["close_panel"])
    module.close_panel()
    definition = _interface_definition()
    pane = hou.ui.curDesktop().createFloatingPaneTab(
        hou.paneTabType.PythonPanel, python_panel_interface=definition.name(), immediate=True)
    _remember_pane(pane)
    return pane
