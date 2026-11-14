"""Pure-Python checks for handoff guards/lifecycle; native GUI coverage is separate."""
import importlib.util
import os
import runpy
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]


class Widget:
    def __init__(self, name="RE_Window", title="", parent=None, window=True):
        self.name, self.title, self.parent, self.window = name, title, parent, window
        self.visible = True
        self.attributes = {}

    def objectName(self): return self.name
    def windowTitle(self): return self.title
    def parentWidget(self): return self.parent
    def isWindow(self): return self.window
    def isVisible(self): return self.visible
    def testAttribute(self, flag): return self.attributes.get(flag, False)
    def setAttribute(self, flag, value): self.attributes[flag] = value
    def close(self): self.visible = False
    def hide(self): self.visible = False
    def show(self): self.visible = True


class QObject:
    def __init__(self, *args): pass
    def deleteLater(self): pass


class ShortcutsTests(unittest.TestCase):
    def setUp(self):
        self.callbacks = []
        self.timers = []
        self.bindings = {
            ("inputfield", "inputfield.editor"): ["Ctrl+E"],
            ("h.pane.textport", "h.pane.textport.editor"): ["Alt+E", "Ctrl+E"],
            ("h.pane.parms", "h.pane.parms.ko_edit_in_zed"): ["Ctrl+E"],
        }
        self.app = NS(installEventFilter=Mock(), removeEventFilter=Mock())
        self.qtapp = NS(instance=lambda: self.app, topLevelWidgets=lambda: [])
        self.parms = {}
        self.hou = NS(
            qt=NS(mainWindow=lambda: Widget()),
            ui=NS(setStatusMessage=Mock()), severityType=NS(Warning="warning"),
            parm=lambda path: self.parms.get(path), parmTemplateType=NS(String="string"),
            getPreference=lambda name: "0",
            hipFileEventType=NS(BeforeLoad="load", BeforeClear="clear"),
            hipFile=NS(addEventCallback=self.callbacks.append, removeEventCallback=self.callbacks.remove,
                       eventCallbacks=lambda: tuple(self.callbacks)),
            hotkeys=NS(assignments=lambda c, a: list(self.bindings[(c, a)]),
                       addAssignment=self.add_binding, removeAssignment=self.remove_binding))
        fakeqt = NS(QtCore=NS(QObject=QObject, QTimer=NS(singleShot=lambda ms, cb: self.timers.append(cb)),
                              QEvent=NS(Type=NS(KeyPress=1, Show=2, WindowTitleChange=3, KeyRelease=4, ApplicationDeactivate=5)),
                              Qt=NS(WidgetAttribute=NS(WA_DontShowOnScreen="hidden"),
                                    Key=NS(Key_E=69), KeyboardModifier=NS(AltModifier=2, ControlModifier=1))),
                    QtGui=NS(), QtWidgets=NS(QWidget=Widget, QApplication=self.qtapp))
        spec = importlib.util.spec_from_file_location("zed_shortcut_under_test", ROOT / "python3.13libs/ko_zed_shortcuts.py")
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {"hou": self.hou, "hutil": NS(Qt=fakeqt), "hutil.Qt": fakeqt,
                                     "shiboken6": NS(isValid=lambda obj: True)}):
            spec.loader.exec_module(self.module)

    def add_binding(self, context, command, key):
        self.bindings[(context, command)].append(key)
        return True

    def remove_binding(self, context, command, key):
        self.bindings[(context, command)].remove(key)

    def parm(self, path="/obj/test/snippet", string=True, expression=False):
        parm = NS(parmTemplate=lambda: NS(type=lambda: "string" if string else "float"),
                  keyframes=lambda: [1] if expression else [], isLocked=lambda: False)
        self.parms[path] = parm
        return parm

    def test_only_native_parameter_ancestry_is_recognized(self):
        panel = Widget(title="parmdialog")
        child = Widget(name="RE_WindowDrawable", parent=panel, window=False)
        self.assertIs(self.module.parameter_panel(child), panel)
        self.assertIsNone(self.module.parameter_panel(Widget(title="networkeditor")))
        self.assertIsNone(self.module.parameter_panel(Widget(name="QLineEdit", title="parmdialog")))

    def test_exact_editor_path_not_selection_or_partial_title(self):
        parm = self.parm()
        self.assertIs(self.module.parameter_from_editor(Widget(title="Edit: /obj/test/snippet")), parm)
        for window in [Widget(title="Edit: /obj/missing/snippet"), Widget(title="Edit: /obj/test/snippet extra"),
                       Widget(name="fake", title="Edit: /obj/test/snippet"),
                       Widget(title="Edit: /obj/test/snippet", window=False)]:
            self.assertIsNone(self.module.parameter_from_editor(window))

    def test_expression_and_non_string_editors_are_left_native(self):
        for string, expression in [(False, False), (True, True)]:
            self.parm(string=string, expression=expression)
            self.assertIsNone(self.module.parameter_from_editor(Widget(title="Edit: /obj/test/snippet")))

    def test_enable_disable_preserves_existing_bindings(self):
        original = {key: list(value) for key, value in self.bindings.items()}
        handler = self.module.enable(Mock())
        self.assertIs(self.module.enable(), handler)
        self.assertEqual(len(self.callbacks), 1)
        self.assertEqual(self.bindings[("h.pane.parms", "h.pane.parms.ko_edit_in_zed")], [])
        self.assertEqual(self.bindings[("inputfield", "inputfield.editor")], ["Ctrl+E", "Alt+E"])
        self.module.disable()
        self.module.disable()
        self.assertEqual(self.bindings, original)
        self.assertEqual(self.callbacks, [])
        self.app.removeEventFilter.assert_called_once_with(handler)

    def test_only_alt_e_arms_handoff_ctrl_e_stays_native(self):
        handler = self.module.enable(Mock())
        panel = Widget(title="parmdialog")
        event = NS(type=lambda: 1, key=lambda: 69, modifiers=lambda: 1, isAutoRepeat=lambda: False)
        self.assertFalse(handler.eventFilter(panel, event))
        self.assertIsNone(handler.pending)
        event.modifiers = lambda: 2
        self.assertFalse(handler.eventFilter(panel, event))
        self.assertIsNotNone(handler.pending)

    def test_held_alt_e_waits_for_release_before_timing_out(self):
        handler = self.module.enable(Mock())
        handler.request()
        pending = handler.pending
        pending['deadline'] = -1
        handler.zed_key_down = True
        handler.expire(pending)
        self.assertIs(handler.pending, pending)
        event = NS(type=lambda: 4, key=lambda: 69)
        self.assertFalse(handler.eventFilter(Widget(title='parmdialog'), event))
        self.assertFalse(handler.zed_key_down)
        self.assertGreater(pending['deadline'], 0)

    def test_application_deactivation_cancels_pending_key_cycle(self):
        handler = self.module.enable(Mock())
        handler.request()
        handler.zed_key_down = True
        self.assertFalse(handler.eventFilter(Widget(), NS(type=lambda: 5)))
        self.assertIsNone(handler.pending)
        self.assertFalse(handler.zed_key_down)

    def test_existing_visible_editor_is_not_hijacked(self):
        self.parm()
        launch = Mock()
        handler = self.module.enable(launch)
        editor = Widget(title="Edit: /obj/test/snippet")
        pending = {"editor": None, "before": {editor}, "deadline": float("inf")}
        handler.pending = pending
        handler.capture(editor)
        self.assertIsNone(pending["editor"])
        self.assertTrue(editor.visible)
        launch.assert_not_called()

    def test_fresh_editor_handoff_uses_resolved_parm(self):
        parm = self.parm()
        launch = Mock()
        handler = self.module.enable(launch)
        editor = Widget(title="Edit: /obj/test/snippet")
        pending = {"editor": None, "before": set(), "deadline": float("inf")}
        handler.pending = pending
        handler.capture(editor)
        self.assertTrue(editor.testAttribute("hidden"))
        handler.handoff(pending)
        launch.assert_called_once_with(parm)
        self.assertFalse(editor.visible)
        self.assertFalse(editor.testAttribute("hidden"))
        self.assertIsNone(handler.pending)

    def test_launch_failure_restores_native_editor(self):
        self.parm()
        handler = self.module.enable(Mock(side_effect=RuntimeError("attach failed")))
        editor = Widget(title="Edit: /obj/test/snippet")
        pending = {"editor": None, "before": set(), "deadline": float("inf")}
        handler.pending = pending
        handler.capture(editor)
        handler.handoff(pending)
        self.assertTrue(editor.visible)
        self.assertFalse(editor.testAttribute("hidden"))
        self.assertIsNone(handler.pending)
        self.assertIn("native editor retained", self.hou.ui.setStatusMessage.call_args.args[0])

    def test_scene_load_cancels_pending_and_queued_handoff(self):
        self.parm()
        launch = Mock()
        handler = self.module.enable(launch)
        editor = Widget(title="Edit: /obj/test/snippet")
        pending = {"editor": None, "before": set(), "deadline": float("inf")}
        handler.pending = pending
        handler.capture(editor)
        handler.scene_event("load")
        handler.handoff(pending)
        launch.assert_not_called()
        self.assertIsNone(handler.pending)
        self.assertFalse(editor.testAttribute("hidden"))

    def test_external_editor_preference_is_not_silently_changed(self):
        self.hou.getPreference = lambda name: "1"
        with self.assertRaisesRegex(RuntimeError, "Use External Editor"):
            self.module.enable()
        self.assertIsNone(self.module._handler)
        self.assertEqual(self.callbacks, [])


class StartupTests(unittest.TestCase):
    def run_startup(self, ui=True, enabled="1", fail=False):
        order, scheduled = [], []
        hou = NS(isUIAvailable=lambda: ui, text=NS(expandString=lambda path: path),
                 hotkeys=NS(saveAsKeymap=lambda *args: order.append("save")),
                 ui=NS(setStatusMessage=Mock()), severityType=NS(Warning="warning"))
        enable = Mock(side_effect=RuntimeError("test startup failure") if fail else lambda: order.append("enable"))
        qt = NS(QtCore=NS(QTimer=NS(singleShot=lambda delay, callback: scheduled.append(callback))))
        with patch.dict(sys.modules, {"hou": hou, "hutil": NS(Qt=qt), "hutil.Qt": qt,
                                     "ko_zed_shortcuts": NS(enable=enable)}), patch.dict(os.environ, {"KO_ZED_SHORTCUTS": enabled}):
            runpy.run_path(str(ROOT / "python3.13libs/uiready.py"))
            self.assertEqual(order, ["save"])
            for callback in scheduled:
                callback()
        return order, scheduled, enable, hou

    def test_ui_startup_saves_before_deferred_enable(self):
        order, scheduled, enable, hou = self.run_startup()
        self.assertEqual(order, ["save", "enable"])
        self.assertEqual(len(scheduled), 1)
        enable.assert_called_once_with()
        hou.ui.setStatusMessage.assert_not_called()

    def test_startup_opt_out(self):
        order, scheduled, enable, hou = self.run_startup(enabled="0")
        self.assertEqual(scheduled, [])
        enable.assert_not_called()

    def test_headless_does_not_install_shortcuts(self):
        order, scheduled, enable, hou = self.run_startup(ui=False)
        self.assertEqual(scheduled, [])
        enable.assert_not_called()

    def test_startup_failure_warns_without_aborting(self):
        order, scheduled, enable, hou = self.run_startup(fail=True)
        self.assertIn("test startup failure", hou.ui.setStatusMessage.call_args.args[0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
