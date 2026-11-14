"""Native Open Editor window handoff for Houdini 22 parameter fields.

No native QAction/Python command callback was exposed in the inspected widgets.
Let Houdini handle the real key and resolve its target, then intercept the new
`Edit: /absolute/parm/path` window. This is a version-sensitive workaround.
"""
import time

from hutil.Qt import QtCore, QtWidgets
import hou
import shiboken6

NATIVE_COMMANDS = (("inputfield", "inputfield.editor"),
                   ("h.pane.textport", "h.pane.textport.editor"))


def parameter_panel(widget):
    """Recognize native parameter panels; never infer a target from selection."""
    while isinstance(widget, QtWidgets.QWidget):
        if widget.objectName() == "RE_Window" and widget.windowTitle() == "parmdialog":
            return widget
        widget = widget.parentWidget()
    return None


def parameter_from_editor(widget):
    if not isinstance(widget, QtWidgets.QWidget) or not widget.isWindow():
        return None
    if widget.objectName() != "RE_Window":
        return None
    title = widget.windowTitle()
    if not title.startswith("Edit: /"):
        return None
    parm = hou.parm(title[len("Edit: "):])
    if parm is None or parm.parmTemplate().type() != hou.parmTemplateType.String:
        return None
    # The bridge supports literal strings only. Leave other native editors alone.
    if parm.keyframes() or parm.isLocked():
        return None
    return parm


class EditorHandoff(QtCore.QObject):
    def __init__(self, launch=None):
        super().__init__(hou.qt.mainWindow())
        self.launch = launch or self._launch
        self.added_bindings = []
        self.suspended_bindings = []
        self.pending = None
        self.closed = False
        self.zed_key_down = False
        try:
            # Alt+E must reach native Open Editor so it can resolve the actual field.
            # Ctrl+E uses the same command but is never armed for a Zed handoff.
            # Add only; do not erase/reassign existing user bindings or save a keymap.
            for context, command in NATIVE_COMMANDS:
                for key in ("Ctrl+E", "Alt+E"):
                    if key not in hou.hotkeys.assignments(context, command):
                        if not hou.hotkeys.addAssignment(context, command, key):
                            raise RuntimeError("Cannot assign " + key + " to " + command)
                        self.added_bindings.append((context, command, key))
            # Our menu action competes with native field dispatch if bound to the
            # same key. Suspend only its two chords while this handoff owns them.
            context, command = "h.pane.parms", "h.pane.parms.ko_edit_in_zed"
            for key in ("Ctrl+E", "Alt+E"):
                if key in hou.hotkeys.assignments(context, command):
                    hou.hotkeys.removeAssignment(context, command, key)
                    self.suspended_bindings.append((context, command, key))
            QtWidgets.QApplication.instance().installEventFilter(self)
            hou.hipFile.addEventCallback(self.scene_event)
        except Exception:
            self.close()
            raise

    @staticmethod
    def _launch(parm):
        import ko_zed
        ko_zed.edit_parameter(parm)

    @staticmethod
    def warn(message):
        hou.ui.setStatusMessage("Zed shortcuts: " + message, severity=hou.severityType.Warning)

    def eventFilter(self, obj, event):
        if self.closed:
            return False
        try:
            kind = event.type()
            if kind == QtCore.QEvent.Type.KeyPress:
                if (event.key() == QtCore.Qt.Key.Key_E
                        and event.modifiers() == QtCore.Qt.KeyboardModifier.AltModifier
                        and parameter_panel(obj) is not None):
                    if event.isAutoRepeat():
                        return True
                    self.zed_key_down = True
                    self.request()
                    # Crucially, don't consume/remap the key: native Open Editor
                    # handles it in its own input-field/textport context.
            elif kind == QtCore.QEvent.Type.KeyRelease:
                if event.key() == QtCore.Qt.Key.Key_E and self.zed_key_down:
                    self.zed_key_down = False
                    if self.pending is not None:
                        # Alt shortcuts can resolve on key release. Don't let a
                        # long-held chord consume the target-resolution timeout.
                        self.pending["deadline"] = time.monotonic() + 1.5
            elif kind == QtCore.QEvent.Type.ApplicationDeactivate:
                self.cancel_pending()
            elif kind in (QtCore.QEvent.Type.Show, QtCore.QEvent.Type.WindowTitleChange):
                self.capture(obj)
        except Exception as error:
            # Never let an exception escape into Houdini's event dispatcher.
            self.warn(str(error))
        return False

    def request(self):
        if self.pending is not None:
            return
        pending = {"before": {w for w in QtWidgets.QApplication.topLevelWidgets() if w.isVisible()},
                   "deadline": time.monotonic() + 1.5, "editor": None}
        self.pending = pending
        QtCore.QTimer.singleShot(1600, lambda: self.expire(pending))

    def capture(self, widget):
        pending = self.pending
        if pending is None or pending["editor"] is not None:
            return
        if (time.monotonic() > pending["deadline"] and not self.zed_key_down) or widget in pending["before"]:
            return
        parm = parameter_from_editor(widget)
        if parm is None:
            return
        pending["editor"] = widget
        pending["parm"] = parm
        pending["dont_show"] = widget.testAttribute(QtCore.Qt.WidgetAttribute.WA_DontShowOnScreen)
        # The title-change event precedes native Show in tested Houdini 22.0.466.
        widget.setAttribute(QtCore.Qt.WidgetAttribute.WA_DontShowOnScreen, True)
        QtCore.QTimer.singleShot(0, lambda: self.handoff(pending))

    def handoff(self, pending):
        if self.closed or self.pending is not pending:
            return
        editor = pending["editor"]
        try:
            self.launch(pending["parm"])
            if shiboken6.isValid(editor):
                editor.close()
        except Exception as error:
            self.restore_editor(pending)
            self.warn(str(error) + "; native editor retained")
        finally:
            if shiboken6.isValid(editor):
                editor.setAttribute(QtCore.Qt.WidgetAttribute.WA_DontShowOnScreen, pending["dont_show"])
            self.pending = None

    @staticmethod
    def restore_editor(pending):
        editor = pending.get("editor")
        if editor is not None and shiboken6.isValid(editor):
            editor.setAttribute(QtCore.Qt.WidgetAttribute.WA_DontShowOnScreen, pending["dont_show"])
            editor.hide()
            editor.show()

    def expire(self, pending):
        if self.pending is pending:
            if self.zed_key_down or time.monotonic() < pending["deadline"]:
                QtCore.QTimer.singleShot(200, lambda: self.expire(pending))
                return
            self.pending = None
            self.restore_editor(pending)
            self.warn("No new literal-parameter editor identified; nothing sent to Zed. "
                      "Existing internal editor windows are not intercepted.")

    def cancel_pending(self):
        self.zed_key_down = False
        pending, self.pending = self.pending, None
        if pending is not None:
            self.restore_editor(pending)

    def scene_event(self, event):
        if event in (hou.hipFileEventType.BeforeLoad, hou.hipFileEventType.BeforeClear):
            self.cancel_pending()

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.scene_event in hou.hipFile.eventCallbacks():
            hou.hipFile.removeEventCallback(self.scene_event)
        QtWidgets.QApplication.instance().removeEventFilter(self)
        if self.pending is not None:
            self.restore_editor(self.pending)
            self.pending = None
        for context, command, key in self.added_bindings:
            hou.hotkeys.removeAssignment(context, command, key)
        self.added_bindings.clear()
        for context, command, key in self.suspended_bindings:
            hou.hotkeys.addAssignment(context, command, key)
        self.suspended_bindings.clear()
        self.deleteLater()


# Preserve the filter across the existing Reload Ko shelf action.
_handler = globals().get("_handler")


def enable(launch=None):
    """Enable once per UI session. Optional launch callback is for offline GUI tests."""
    global _handler
    if _handler is None:
        if hou.getPreference("ui.parm.external") == "1":
            raise RuntimeError("Turn off Preferences → Scripting → Use External Editor first. "
                               "This handoff needs Houdini's internal editor.")
        _handler = EditorHandoff(launch)
    return _handler


def direct_launch():
    """Cancel an armed request if a parameter-menu shortcut already launched Zed."""
    if _handler is not None and _handler.pending is not None and _handler.pending["editor"] is None:
        _handler.pending = None


def disable():
    global _handler
    if _handler is not None:
        _handler.close()
        _handler = None


def toggle():
    if _handler is None:
        enable()
        hou.ui.setStatusMessage("Zed shortcuts ON: Ctrl+E → internal editor; Alt+E → Zed")
    else:
        disable()
        hou.ui.setStatusMessage("Zed shortcuts OFF; added native bindings removed")
