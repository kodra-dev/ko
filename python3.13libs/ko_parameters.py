"""Native Toggle Parameters menu action, independent of Loom and its provider.

Persist HOM FloatingPanel geometry, never Qt widget/frame/saveGeometry coordinates.
"""
import json
import math
import os
from pathlib import Path
import uuid

import hou
from hutil.Qt import QtCore, QtWidgets
from shiboken6 import isValid

PANE_NAME = "KoParameters"
MENU_SYMBOL = "h.ko_toggle_parameters"
LEGACY_SYMBOL = "h.tool:ko_restore_parameters"
SESSION_KEY = "_ko_parameters_controller"


def state_path():
    override = os.environ.get("KO_PARAMETERS_STATE")
    return Path(override) if override else Path(hou.homeHoudiniDirectory()) / "ko-parameters.json"


def _geometry(value):
    if not isinstance(value, dict):
        return {}
    result = {}
    for name in ("position", "size"):
        pair = value.get(name)
        if (not isinstance(pair, (list, tuple)) or len(pair) != 2
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in pair)):
            return {}
        result[name] = tuple(int(round(v)) for v in pair)
    return result if min(result["size"]) > 0 else {}


def _read():
    try:
        value = json.loads(state_path().read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def _write(value):
    path = state_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    staging = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        with staging.open("x", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(staging, path)
    finally:
        staging.unlink(missing_ok=True)


def _existing():
    candidates = []
    for pane in hou.ui.paneTabs():
        if pane.type() != hou.paneTabType.Parm or pane.floatingPanel() is None:
            continue
        # Never close a docked pane or a multi-tab/multi-pane floating workspace.
        if len(pane.floatingPanel().paneTabs()) != 1:
            continue
        if pane.name() in (PANE_NAME, "KoRestoreParameters", "FloatingPanel"):
            candidates.append(pane)
    return min(candidates, key=lambda p: (p.name() != PANE_NAME, p.name() != "KoRestoreParameters"), default=None)


class ParametersController(QtCore.QObject):
    def __init__(self):
        super().__init__(QtWidgets.QApplication.instance())
        self.pane = self.window = None
        self.last = None
        self.dirty = False
        self.poll = QtCore.QTimer(self)
        self.poll.setInterval(250)
        self.poll.timeout.connect(self.capture)
        self.writer = QtCore.QTimer(self)
        self.writer.setSingleShot(True)
        self.writer.timeout.connect(self.save)

    def alive(self):
        try:
            return (self.pane is not None and self.pane.type() == hou.paneTabType.Parm
                    and self.pane.floatingPanel() is not None)
        except hou.ObjectWasDeleted:
            return False  # A surviving Qt wrapper is NOT proof of a live HOM pane.

    def attach(self, pane):
        self.detach()
        self.pane = pane
        self.last = None
        self.poll.start()
        self.capture()  # Qt window may not exist until the next native idle boundary.

    def detach(self):
        self.poll.stop()
        self.writer.stop()
        if self.window is not None and isValid(self.window):
            self.window.removeEventFilter(self)
        self.window = self.pane = None

    def capture(self):
        if not self.alive():
            self.save()
            self.detach()
            return
        try:
            floating = self.pane.floatingPanel()
            if self.window is None or not isValid(self.window):
                self.window = floating.qtParentWindow()
                if self.window is not None and isValid(self.window):
                    self.window.installEventFilter(self)
            if (self.window is None or not isValid(self.window) or not self.window.isVisible()
                    or self.window.isMinimized() or self.window.isMaximized() or self.window.isFullScreen()):
                return
            value = {"position": [int(round(v)) for v in floating.position()],
                     "size": [int(round(v)) for v in floating.size()],
                     "node": self.pane.currentNode().path() if self.pane.currentNode() else "/obj",
                     "pinned": self.pane.isPin()}
            if _geometry(value) and value != self.last:
                self.last, self.dirty = value, True
                self.writer.start(150)
        except hou.ObjectWasDeleted:
            self.save()
            self.detach()

    def save(self):
        if self.dirty and self.last:
            try:
                _write(self.last)
                self.dirty = False
            except OSError:
                # Optional geometry persistence must not break native window closing.
                pass

    def flush(self):
        self.capture()
        self.save()

    def eventFilter(self, watched, event):
        if watched is self.window:
            if event.type() == QtCore.QEvent.Type.Close:
                self.flush()
                self.detach()  # Do not record teardown/recycled-window resize events.
            elif event.type() in (QtCore.QEvent.Type.Move, QtCore.QEvent.Type.Resize):
                self.capture()
        return False

    def open(self):
        saved = _read()
        geometry = _geometry(saved)
        # Pass the saved native coordinates at creation, exactly as Toggle Loom does.
        # Do not create at a default position then animate a Qt widget into place.
        pane = hou.ui.curDesktop().createFloatingPaneTab(hou.paneTabType.Parm, immediate=True, **geometry)
        pane.setName(PANE_NAME)
        pane.showNetworkControls(False)
        node = hou.node(saved.get("node", "/obj")) if isinstance(saved.get("node", "/obj"), str) else None
        if node is not None:
            pane.setCurrentNode(node, pick_node=False)
        pane.setPin(saved.get("pinned") is True)
        if geometry:
            floating = pane.floatingPanel()
            # Creation's position is documented as approximate. Native setters use
            # the same coordinate system as capture, without Qt frame/DPI conversion.
            floating.setSize(geometry["size"])
            floating.setPosition(geometry["position"])
        self.attach(pane)
        return pane

    def toggle(self):
        if not self.alive():
            pane = _existing()
            if pane is not None:
                self.attach(pane)
        if self.alive():
            floating = self.pane.floatingPanel()
            self.flush()
            self.detach()
            floating.close()
            return None
        return self.open()


def _controller():
    current = getattr(hou.session, SESSION_KEY, None)
    if not isinstance(current, ParametersController):
        if current is not None:
            current.flush()
            current.detach()
            current.deleteLater()
        current = ParametersController()
        setattr(hou.session, SESSION_KEY, current)
    return current


def _detach_legacy_tracker():
    tracker = getattr(hou.session, "_ko_parameter_tracker", None)
    if tracker is not None:
        window = getattr(tracker, "window", None)
        if window is not None and isValid(window):
            window.removeEventFilter(tracker)
        tracker.deleteLater()
        hou.session._ko_parameter_tracker = None
        hou.session._ko_parameter_restore_module = None


def configure_hotkey():
    """Keep custom menu assignments; migrate the old shelf key without stealing conflicts."""
    if not hou.hotkeys.hotkeyLabel(MENU_SYMBOL):
        return  # The new menu XML is discovered on the next Houdini restart.
    assigned = hou.hotkeys.assignments("h", MENU_SYMBOL)
    legacy = hou.hotkeys.assignments("h", LEGACY_SYMBOL)
    key = "Ctrl+Shift+P"
    conflicts = hou.hotkeys.findConflicts("h", MENU_SYMBOL, key)
    other = [c for c in conflicts if c.split("?")[-1] not in (MENU_SYMBOL, LEGACY_SYMBOL)]
    changed = False
    if not assigned and not other:
        changed = hou.hotkeys.addAssignment("h", MENU_SYMBOL, key)
    if assigned or changed:
        # Remove only the replaced tool's old requested key, not other user bindings.
        if key in legacy:
            changed = hou.hotkeys.removeAssignment("h", LEGACY_SYMBOL, key) or changed
    if changed:
        hou.hotkeys.saveOverrides()
    if not assigned and not changed and other:
        hou.ui.setStatusMessage("Toggle Parameters: Ctrl+Shift+P is occupied; assign a key in the Hotkey Manager.",
                                severity=hou.severityType.Warning)


def install():
    """Startup: adopt/track an existing panel and migrate the key; never open a window."""
    if not hou.isUIAvailable():
        return
    _detach_legacy_tracker()
    controller = _controller()
    if not controller.alive():
        pane = _existing()
        if pane is not None:
            controller.attach(pane)
    configure_hotkey()


def toggle_parameters():
    if not hou.isUIAvailable():
        raise RuntimeError("Toggle Parameters requires graphical Houdini")
    _detach_legacy_tracker()
    return _controller().toggle()
