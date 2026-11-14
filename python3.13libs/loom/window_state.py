"""Independent native floating-pane geometry preferences; no agent dependency."""
import math
from PySide6 import QtCore
from shiboken6 import isValid
from .checkpoints.storage import atomic_json, data_root, read_json


def _preferences():
    path = data_root() / "pane-geometry.json"
    try:
        value = read_json(path) if path.exists() else {}
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def geometry(kind):
    value = _preferences().get(kind)
    if not isinstance(value, dict):
        return {}
    result = {}
    for field in ("position", "size"):
        pair = value.get(field)
        if (not isinstance(pair, list) or len(pair) != 2
                or not all(isinstance(n, (int, float)) and not isinstance(n, bool) and math.isfinite(n) for n in pair)):
            return {}
        result[field] = tuple(int(round(n)) for n in pair)
    if min(result["size"]) <= 0:
        return {}
    return result


def restore(pane, saved):
    floating = pane.floatingPanel()
    if floating is not None and saved:
        # Creation's position parameter is only approximate in Houdini. Apply the
        # final geometry through the floating-panel API as well, never to docked tabs.
        floating.setSize(saved["size"])
        floating.setPosition(saved["position"])


def track(kind, pane, root):
    if pane is None or root is None or getattr(root, "_loom_geometry_tracker", None) is not None:
        return
    if pane.floatingPanel() is not None:
        root._loom_geometry_tracker = GeometryTracker(kind, pane, root)


def flush(root):
    tracker = getattr(root, "_loom_geometry_tracker", None)
    if tracker is not None:
        try:
            tracker.finish()
        except Exception:
            pass  # Optional UI state must never prevent native/controller teardown.


class GeometryTracker(QtCore.QObject):
    def __init__(self, kind, pane, root):
        super().__init__(root)
        self.kind, self.pane = kind, pane
        self.last = None
        self.dirty = False
        self.stopped = False
        # Native pane creation may precede its Qt floating window by an idle tick.
        self.window = None
        self.write_timer = QtCore.QTimer(self)
        self.write_timer.setSingleShot(True)
        self.write_timer.timeout.connect(self.save)
        self.poll = QtCore.QTimer(self)
        self.poll.timeout.connect(self.capture)
        self.poll.start(250)
        QtCore.QTimer.singleShot(0, self.capture)

    def capture(self):
        if self.stopped:
            return
        try:
            floating = self.pane.floatingPanel()
            if floating is not None and (self.window is None or not isValid(self.window)):
                self.window = floating.qtParentWindow()
                if self.window is not None and isValid(self.window):
                    self.window.installEventFilter(self)
            if (floating is None or self.window is None or not isValid(self.window) or not self.window.isVisible()
                    or self.window.isMinimized() or self.window.isMaximized()):
                return
            value = {"position": [int(round(v)) for v in floating.position()],
                     "size": [int(round(v)) for v in floating.size()]}
            if min(value["size"]) > 0 and value != self.last:
                self.last = value
                self.dirty = True
                self.write_timer.start(150)
        except Exception:
            pass  # Houdini may invalidate its pane before onDestroyInterface.

    def save(self):
        if self.dirty and self.last:
            try:
                saved = _preferences()
                saved[self.kind] = self.last
                atomic_json(data_root() / "pane-geometry.json", saved)
                self.dirty = False
            except Exception:
                pass  # Optional UI preference failure must not close/break a pane.

    def eventFilter(self, watched, event):
        if event.type() in (QtCore.QEvent.Type.Move, QtCore.QEvent.Type.Resize, QtCore.QEvent.Type.Close):
            self.capture()
            if event.type() == QtCore.QEvent.Type.Close:
                self.save()
        return False

    def finish(self):
        if self.stopped:
            return
        self.capture()
        self.save()
        self.stopped = True
        if isValid(self.poll):
            self.poll.stop()
        if isValid(self.write_timer):
            self.write_timer.stop()
        if self.window is not None and isValid(self.window):
            self.window.removeEventFilter(self)
