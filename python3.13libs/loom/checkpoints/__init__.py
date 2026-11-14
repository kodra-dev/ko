"""Scene checkpoints. Importing this package never imports the Loom agent."""

_window = None


def create_panel(parent=None):
    from PySide6 import QtWidgets
    from .panel import Panel
    import traceback
    # Keep startup/lock errors inline, rather than a native Python Panel traceback.
    try:
        return Panel(parent)
    except Exception as exc:
        host = QtWidgets.QWidget(parent)
        layout = QtWidgets.QVBoxLayout(host)
        label = QtWidgets.QLabel(str(exc))
        label.setWordWrap(True)
        layout.addWidget(label)
        details = QtWidgets.QPlainTextEdit(traceback.format_exc())
        details.setReadOnly(True)
        layout.addWidget(details)
        return host


def _remember_pane(pane):
    import hou
    names = getattr(hou.session, "_loom_checkpoint_pane_names", None)
    if names is None:
        names = set()
        hou.session._loom_checkpoint_pane_names = names
    names.add(pane.name())


def close():
    global _window
    window = _window
    if window is not None:
        window.close()
        _window = None


def toggle_pane():
    import hou
    from ..window_state import flush
    for pane in hou.ui.paneTabs():
        interface = pane.activeInterface() if pane.type() == hou.paneTabType.PythonPanel else None
        if interface is not None and interface.name() == "loom_checkpoints" and pane.floatingPanel() is not None:
            flush(pane.activeInterfaceRootWidget())
            pane.floatingPanel().close()
            return None
    return open_pane()


def open_pane():
    import hou
    from pathlib import Path
    if not hou.isUIAvailable():
        raise RuntimeError("Checkpoints requires graphical Houdini")
    name = "loom_checkpoints"
    for pane in hou.ui.paneTabs():
        if pane.type() == hou.paneTabType.PythonPanel:
            interface = pane.activeInterface()
            if interface is not None and interface.name() == name:
                pane.setIsCurrentTab()
                return pane
    if hou.pypanel.interfaceByName(name) is None:
        path = Path(__file__).resolve().parents[3] / "python_panels/loom_checkpoints.pypanel"
        hou.pypanel.installFile(str(path))
    from ..window_state import geometry, restore, track
    saved = geometry("checkpoints")
    pane = hou.ui.curDesktop().createFloatingPaneTab(
        hou.paneTabType.PythonPanel, python_panel_interface=name, immediate=True, **saved)
    restore(pane, saved)
    _remember_pane(pane)
    track("checkpoints", pane, pane.activeInterfaceRootWidget())
    return pane


def show():
    global _window
    from PySide6 import QtCore, QtWidgets
    import hou
    if _window is None:
        _window = QtWidgets.QDialog(hou.qt.mainWindow() if hou.isUIAvailable() else None)
        _window.setWindowTitle("Checkpoints")
        _window.resize(720, 700)
        layout = QtWidgets.QVBoxLayout(_window)
        panel = create_panel(_window)
        _window._loom_checkpoint_panel = panel
        layout.addWidget(panel)
        def finished(*_):
            global _window
            if hasattr(panel, "shutdown"):
                panel.shutdown()
            _window = None
        _window.finished.connect(finished)
        _window.setAttribute(QtCore.Qt.WidgetAttribute.WA_DeleteOnClose)
    _window.show()
    _window.raise_()
    return _window
