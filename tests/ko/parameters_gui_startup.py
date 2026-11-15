"""Run only via scripts/parameters_gui_test.ps1: real HOM, hidden desktop, no AI."""
import ctypes
from ctypes import wintypes
import importlib
import json
import os
from pathlib import Path
import sys
import time
import traceback
import xml.etree.ElementTree as ET

import hou
from hutil.Qt import QtCore, QtTest, QtWidgets

RUN = Path(os.environ["KO_PARAMETERS_GUI_RUN"])
REPO = Path(os.environ["KO_PARAMETERS_GUI_REPO"])
sys.path.insert(0, str(REPO / "python3.13libs"))
import ko_parameters as parameters


def log(value):
    with (RUN / "gui.log").open("a", encoding="utf-8") as stream:
        stream.write(str(value) + "\n")


def desktop_name():
    user32, kernel32 = ctypes.WinDLL("user32", use_last_error=True), ctypes.WinDLL("kernel32")
    kernel32.GetCurrentThreadId.restype = wintypes.DWORD
    user32.GetThreadDesktop.argtypes = [wintypes.DWORD]
    user32.GetThreadDesktop.restype = wintypes.HANDLE
    user32.GetUserObjectInformationW.argtypes = [wintypes.HANDLE, ctypes.c_int, wintypes.LPVOID,
                                               wintypes.DWORD, ctypes.POINTER(wintypes.DWORD)]
    name, needed = ctypes.create_unicode_buffer(256), wintypes.DWORD()
    if not user32.GetUserObjectInformationW(user32.GetThreadDesktop(kernel32.GetCurrentThreadId()),
                                          2, name, ctypes.sizeof(name), ctypes.byref(needed)):
        raise ctypes.WinError(ctypes.get_last_error())
    return name.value


def geometry(pane):
    floating = pane.floatingPanel()
    return tuple(floating.position()), tuple(floating.size())


class Test:
    def __init__(self):
        self.checks = []
        self.next = time.monotonic() + 3
        self.steps = self.run()
        self.timer = QtCore.QTimer(hou.qt.mainWindow())
        self.timer.timeout.connect(self.tick)
        self.timer.start(150)

    def passed(self, label):
        self.checks.append(label)
        log("PASS " + label)

    def menu(self):
        assert hou.hotkeys.hotkeyLabel(parameters.MENU_SYMBOL) == "Toggle Parameters"
        source = ET.parse(REPO / "MainMenuCommon.xml").find(
            ".//scriptItem[@id='h.ko_toggle_parameters']/scriptCode").text
        exec(compile(source, "MainMenuCommon.xml:Toggle Parameters", "exec"), {})

    def pane(self):
        return next((p for p in hou.ui.paneTabs() if p.name() == parameters.PANE_NAME), None)

    def run(self):
        assert desktop_name() == os.environ["KO_PARAMETERS_GUI_DESKTOP"]
        assert desktop_name().lower() != "default"
        assert not any(name == "loom" or name.startswith("loom.") for name in sys.modules)
        # User's current keymap, read-only; all runtime overrides go to isolated prefs.
        assert hou.hotkeys.loadKeymap("Ko", str(REPO / "Ko.keymap2"))
        before = len(hou.ui.paneTabs())
        parameters.install()
        assert len(hou.ui.paneTabs()) == before, "Startup opened a window"
        self.passed("startup opens nothing, independent of Loom")
        # Adopt the same normal single-pane floating window seen in the user's log.
        floating = hou.ui.curDesktop().createFloatingPanel(hou.paneTabType.Parm,
            position=(150, 120), size=(650, 600), immediate=True)
        original = floating.paneTabOfType(hou.paneTabType.Parm)
        original.setName("FloatingPanel")
        original.showNetworkControls(False)
        geo = hou.node("/obj").createNode("geo", "parameters_test", run_init_scripts=False)
        original.setCurrentNode(geo, pick_node=False)
        original.setPin(True)
        yield
        parameters.install()
        assert parameters._controller().pane == original
        floating.setSize((650, 600))
        floating.setPosition((150, 120))
        yield
        expected = geometry(original)
        selection = tuple(hou.selectedNodes())
        self.menu()
        yield
        assert parameters._existing() is None, "Toggle did not close adopted floating window"
        self.menu()
        yield
        pane = self.pane()
        assert pane is not None and pane.pane() is None
        assert not pane.isShowingNetworkControls()
        assert pane.currentNode() == geo and pane.isPin()
        assert tuple(hou.selectedNodes()) == selection
        assert geometry(pane) == expected, repr(("adopted geometry", expected, geometry(pane)))
        self.passed("adopt existing, toggle close/reopen, native geometry, hidden bars, selection preserved")
        # Real X/native close, stale HOM + surviving Qt wrappers, repeated creation.
        pane.floatingPanel().setSize((740, 640))
        pane.floatingPanel().setPosition((480, 190))
        yield
        expected = geometry(pane)
        pane.floatingPanel().close()
        yield
        # Recreate the controller as well: the disk state, not old QObject memory, must suffice.
        old = parameters._controller()
        old.flush()
        old.detach()
        old.deleteLater()
        delattr(hou.session, parameters.SESSION_KEY)
        self.menu()
        yield
        pane = self.pane()
        assert geometry(pane) == expected, repr(("X close restore", expected, geometry(pane)))
        yield
        assert geometry(pane) == expected, "Window drifted after native idle"
        self.passed("X close and fresh-controller restore are exact and do not drift")
        # Repeated toggles must not accumulate size/frame/DPI offsets.
        for _ in range(3):
            self.menu()
            yield
            assert self.pane() is None
            self.menu()
            yield
            pane = self.pane()
            assert geometry(pane) == expected, repr((expected, geometry(pane)))
        self.passed("three repeated toggles have no geometry drift")
        window = pane.floatingPanel().qtParentWindow()
        parameters._controller().flush()
        window.showMaximized()
        yield
        self.menu()
        yield
        self.menu()
        yield
        pane = self.pane()
        assert geometry(pane) == expected, repr(("maximized close", expected, geometry(pane)))
        self.passed("maximized close retains last normal geometry")
        # Fresh menu module reload keeps geometry and can close the existing native pane.
        importlib.reload(parameters)
        self.menu()
        yield
        assert self.pane() is None
        self.menu()
        yield
        assert geometry(self.pane()) == expected
        self.passed("module reload does not orphan or duplicate the native pane")
        # Verify actual Ctrl+Shift+P dispatch, not just an assignment or exec'd menu body.
        assert hou.hotkeys.assignments("h", parameters.MENU_SYMBOL) == ("Ctrl+Shift+P",)
        main = hou.qt.mainWindow()
        main.raise_()
        main.activateWindow()
        QtTest.QTest.qWait(80)
        QtTest.QTest.keyClick(main, QtCore.Qt.Key.Key_P,
                             QtCore.Qt.KeyboardModifier.ControlModifier | QtCore.Qt.KeyboardModifier.ShiftModifier)
        yield
        assert self.pane() is None, "Ctrl+Shift+P did not close Parameters"
        QtTest.QTest.keyClick(main, QtCore.Qt.Key.Key_P,
                             QtCore.Qt.KeyboardModifier.ControlModifier | QtCore.Qt.KeyboardModifier.ShiftModifier)
        yield
        assert self.pane() is not None, "Ctrl+Shift+P did not open Parameters"
        assert geometry(self.pane()) == expected
        self.passed("actual Ctrl+Shift+P menu hotkey closes and opens")
        assert not any(name == "loom" or name.startswith("loom.") for name in sys.modules)

    def tick(self):
        if time.monotonic() < self.next:
            return
        self.next = time.monotonic() + 0.45
        try:
            next(self.steps)
        except StopIteration:
            self.finish()
        except Exception:
            self.finish(traceback.format_exc())

    def finish(self, error=None):
        self.timer.stop()
        log(error or "ALL CHECKS PASSED")
        (RUN / "result.json").write_text(json.dumps({"passed": error is None, "error": error,
            "checks": self.checks, "desktop": desktop_name(), "houdini": hou.applicationVersionString()}, indent=2))
        def quit_when_idle():
            hou.ui.removeEventLoopCallback(quit_when_idle)
            hou.exit(exit_code=0 if error is None else 1, suppress_save_prompt=True)
        hou.ui.addEventLoopCallback(quit_when_idle)


test = Test()
