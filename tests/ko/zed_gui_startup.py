"""Run only via scripts/zed_gui_test.ps1. Real GUI, hidden desktop, fake Zed launch."""
import ctypes
from ctypes import wintypes
import json
import os
import runpy
from unittest.mock import patch
from pathlib import Path
import sys
import time
import traceback

import hou
from PySide6 import QtCore, QtGui, QtTest, QtWidgets

RUN = Path(os.environ["KO_ZED_GUI_RUN"])
sys.path.insert(0, str(Path(os.environ["KO_ZED_GUI_REPO"]) / "python3.13libs"))
import ko_zed_shortcuts as shortcuts


def log(value):
    with (RUN / "gui.log").open("a", encoding="utf-8") as f:
        f.write(str(value) + "\n")


def desktop_name():
    user32 = ctypes.WinDLL("user32", use_last_error=True)
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.GetCurrentThreadId.restype = wintypes.DWORD
    user32.GetThreadDesktop.argtypes = [wintypes.DWORD]
    user32.GetThreadDesktop.restype = wintypes.HANDLE
    user32.GetUserObjectInformationW.argtypes = [wintypes.HANDLE, ctypes.c_int, wintypes.LPVOID,
                                               wintypes.DWORD, ctypes.POINTER(wintypes.DWORD)]
    name, needed = ctypes.create_unicode_buffer(256), wintypes.DWORD()
    handle = user32.GetThreadDesktop(kernel32.GetCurrentThreadId())
    if not user32.GetUserObjectInformationW(handle, 2, name, ctypes.sizeof(name), ctypes.byref(needed)):
        raise ctypes.WinError(ctypes.get_last_error())
    return name.value


class Test:
    def __init__(self):
        self.stage = 0
        self.calls = []
        self.launch_sources = []
        self.checks = []
        self.wait_deadlines = {}
        self.next = time.monotonic() + 4
        self.timer = QtCore.QTimer(hou.qt.mainWindow())
        self.timer.timeout.connect(self.tick)
        self.timer.start(150)

    def passed(self, name):
        self.checks.append(name)
        log("PASS " + name)

    def ready(self, condition, label):
        if condition:
            return True
        deadline = self.wait_deadlines.setdefault(label, time.monotonic() + 6)
        assert time.monotonic() < deadline, 'Timed out waiting for ' + label
        return False

    def click(self, x, y):
        self.window.raise_()
        self.window.activateWindow()
        QtTest.QTest.qWait(60)
        widget = self.window.childAt(x, y)
        QtTest.QTest.mouseClick(widget, QtCore.Qt.MouseButton.LeftButton,
                               pos=widget.mapFrom(self.window, QtCore.QPoint(x, y)))
        QtTest.QTest.qWait(60)
        assert shortcuts.parameter_panel(QtWidgets.QApplication.focusWidget()) is not None
        return widget

    def key(self, widget, internal=False):
        QtTest.QTest.keyClick(widget, QtCore.Qt.Key.Key_E,
                             QtCore.Qt.KeyboardModifier.ControlModifier if internal
                             else QtCore.Qt.KeyboardModifier.AltModifier)

    def editors(self):
        return [w for w in QtWidgets.QApplication.topLevelWidgets()
                if w.isVisible() and w.windowTitle().startswith("Edit: /")]

    def launch(self, parm):
        if getattr(self, "fail_launch", False):
            raise RuntimeError("intentional fake launch failure")
        self.calls.append((parm.path(), parm.unexpandedString()))
        self.launch_sources.append('native' if self.handler.pending is not None
                                   and self.handler.pending['editor'] is not None else 'menu')

    def finish(self, error=None):
        self.timer.stop()
        shortcuts.disable()
        (RUN / "result.json").write_text(json.dumps({
            "passed": error is None, "error": error, "stage": self.stage,
            "checks": self.checks, "desktop": desktop_name(), "houdini": hou.applicationVersionString()
        }, indent=2), encoding="utf-8")
        def quit_idle():
            hou.ui.removeEventLoopCallback(quit_idle)
            hou.exit(exit_code=1 if error else 0, suppress_save_prompt=True)
        hou.ui.addEventLoopCallback(quit_idle)

    def tick(self):
        if time.monotonic() < self.next:
            return
        self.next = time.monotonic() + 0.8
        try:
            assert desktop_name() == os.environ["KO_ZED_GUI_DESKTOP"]
            log("STAGE " + str(self.stage))
            if self.stage == 0:
                hou.setUpdateMode(hou.updateMode.Manual)
                hou.setPreference("ui.parm.external", "0")
                # Reproduce the user's setup: native Ctrl+E removed, menu Ctrl+E assigned.
                for context, command in shortcuts.NATIVE_COMMANDS:
                    if 'Ctrl+E' in hou.hotkeys.assignments(context, command):
                        hou.hotkeys.removeAssignment(context, command, 'Ctrl+E')
                assert hou.hotkeys.addAssignment('h.pane.parms', 'h.pane.parms.ko_edit_in_zed', 'Ctrl+E')
                import ko_zed
                ko_zed.edit_parameter = self.launch  # No real editor process, even if menu dispatch wins.
                self.original_bindings = [hou.hotkeys.assignments(c, a) for c, a in shortcuts.NATIVE_COMMANDS]
                self.geo = hou.node("/obj").createNode("geo", "zed_gui_test")
                self.wrangle = self.geo.createNode("attribwrangle")
                self.parm = self.wrangle.parm("snippet")
                self.parm.set("// initial code\nfloat foo = 1;")
                before = set(QtWidgets.QApplication.topLevelWidgets())
                self.pane = hou.ui.curDesktop().createFloatingPaneTab(hou.paneTabType.Parm)
                self.pane.setCurrentNode(self.wrangle)
                self.window = next(w for w in QtWidgets.QApplication.topLevelWidgets()
                                   if w not in before and w.isVisible())
                self.window.resize(850, 750)
                def save_keymap(*args):
                    assert [hou.hotkeys.assignments(c, a) for c, a in shortcuts.NATIVE_COMMANDS] == self.original_bindings
                    assert shortcuts._handler is None
                    self.passed('startup saves original keymap before runtime bindings')
                # Execute the real uiready script; stub ONLY its keymap write to ensure
                # this isolated test cannot touch the artist's Ko.keymap2.
                with patch.object(hou.hotkeys, 'saveAsKeymap', side_effect=save_keymap), patch.dict(os.environ, {'KO_ZED_SHORTCUTS': '1'}):
                    runpy.run_path(str(Path(os.environ['KO_ZED_GUI_REPO']) / 'python3.13libs/uiready.py'))
            elif self.stage == 1:
                self.handler = shortcuts._handler
                assert self.handler is not None, 'uiready did not enable shortcuts automatically'
                assert shortcuts.enable() is self.handler
                self.passed('automatic uiready activation')
                self.window.grab().save(str(RUN / "wrangle.png"))
                target = self.click(260, 270)
                # No focus change or clicking the label between typing and Alt+E.
                QtTest.QTest.keyClicks(target, "UNCOMMITTED")
                self.held_target = target
                QtTest.QTest.keyPress(target, QtCore.Qt.Key.Key_E, QtCore.Qt.KeyboardModifier.AltModifier)
            elif self.stage == 2:
                if not getattr(self, 'released_alt', False):
                    QtTest.QTest.keyRelease(self.held_target, QtCore.Qt.Key.Key_E, QtCore.Qt.KeyboardModifier.AltModifier)
                    self.released_alt = True
                    return  # Native keyhit/handoff may be queued until after Qt's callback returns.
                if not self.ready(len(self.calls) >= 1, 'first multiline handoff'):
                    return
                assert len(self.calls) == 1, self.calls
                assert self.calls[0][0] == self.parm.path()
                assert self.launch_sources == ['native'], self.launch_sources
                assert "UNCOMMITTED" in self.calls[0][1], self.calls
                assert not self.editors(), [w.windowTitle() for w in self.editors()]
                self.passed("focused multiline + uncommitted text + no visible internal editor")
                self.key(self.click(260, 270))
            elif self.stage == 3:
                if not self.ready(len(self.calls) >= 2, 'repeat multiline handoff'):
                    return
                assert len(self.calls) == 2, self.calls
                assert not self.editors()
                self.passed("repeat shortcut reuses/recaptures native editor")
                self.key(self.click(260, 270), internal=True)
            elif self.stage == 4:
                if not self.ready(bool(self.editors()), 'native multiline editor'):
                    return
                assert len(self.calls) == 2
                assert any(w.windowTitle() == "Edit: " + self.parm.path() for w in self.editors())
                self.passed("Ctrl+E keeps native multiline editor")
                for w in self.editors():
                    w.close()
                # A real native single-line string parameter. Hide the null's other controls
                # to make the test input's coordinates deterministic in isolated preferences.
                self.single_node = self.geo.createNode("null")
                group = self.single_node.parmTemplateGroup()
                for template in group.entries():
                    template.hide(True)
                    group.replace(template.name(), template)
                group.append(hou.StringParmTemplate("zed_test_string", "Test String", 1,
                                                    default_value=("original",)))
                self.single_node.setParmTemplateGroup(group)
                self.single = self.single_node.parm("zed_test_string")
                self.pane.setCurrentNode(self.single_node)
            elif self.stage == 5:
                self.window.grab().save(str(RUN / "single.png"))
                target = self.click(260, 100)
                QtTest.QTest.keyClicks(target, "SINGLE_PENDING")
                self.key(target)
            elif self.stage == 6:
                if not self.ready(len(self.calls) >= 3, 'single-line handoff'):
                    return
                assert len(self.calls) == 3, self.calls
                assert self.calls[-1][0] == self.single.path(), self.calls
                assert self.launch_sources[-1] == 'native', self.launch_sources
                assert "SINGLE_PENDING" in self.calls[-1][1], self.calls
                assert not self.editors()
                self.passed("focused single-line field + uncommitted text")
                self.key(self.click(260, 100), internal=True)
            elif self.stage == 7:
                if not self.ready(bool(self.editors()), 'native single-line editor'):
                    return
                assert len(self.calls) == 3
                assert any(w.windowTitle() == "Edit: " + self.single.path() for w in self.editors())
                self.passed("Ctrl+E keeps native single-line editor")
                for w in self.editors():
                    w.close()
                # Outside native parameter panels, Alt+E is not consumed.
                self.line = QtWidgets.QLineEdit(hou.qt.mainWindow())
                self.line.show()
                e = QtGui.QKeyEvent(QtCore.QEvent.Type.KeyPress, QtCore.Qt.Key.Key_E,
                                   QtCore.Qt.KeyboardModifier.AltModifier)
                assert not self.handler.eventFilter(self.line, e)
                self.line.close()
                self.passed("unrelated Qt field untouched")
                self.fail_launch = True
                self.key(self.click(260, 100))
            elif self.stage == 8:
                if not self.ready(bool(self.editors()), 'native fallback editor'):
                    return
                assert len(self.calls) == 3
                assert any(w.windowTitle() == "Edit: " + self.single.path() for w in self.editors())
                self.passed("failed handoff retains native editor")
                for w in self.editors():
                    w.close()
                shortcuts.disable()
                actual = [hou.hotkeys.assignments(c, a) for c, a in shortcuts.NATIVE_COMMANDS]
                assert actual == self.original_bindings, (actual, self.original_bindings)
                assert 'Ctrl+E' in hou.hotkeys.assignments('h.pane.parms', 'h.pane.parms.ko_edit_in_zed')
                assert not self.handler.pending
                self.passed("disable restores added bindings and removes filter")
                self.finish()
            self.stage += 1
        except Exception:
            error = traceback.format_exc()
            log(error)
            log('pending: ' + str(getattr(getattr(self, 'handler', None), 'pending', None)))
            log('visible windows: ' + str([(w.objectName(), w.windowTitle()) for w in QtWidgets.QApplication.topLevelWidgets() if w.isVisible()]))
            self.finish(error)


test = Test()
