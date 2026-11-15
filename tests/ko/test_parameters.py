"""Portable Toggle Parameters logic tests; no Qt/Houdini process or artist prefs."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


class Deleted(Exception):
    pass


class QObject:
    def __init__(self, *args): pass
    def deleteLater(self): pass


class ParametersTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="ko-parameters-unit-")
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "parameters.json"
        self.enterContext(patch.dict(os.environ, {"KO_PARAMETERS_STATE": str(self.path)}))
        self.panes = []
        self.created = []
        self.bindings = {}
        self.conflicts = []
        self.hotkeys = NS(
            hotkeyLabel=lambda symbol: "Toggle Parameters",
            assignments=lambda context, symbol: self.bindings.get(symbol, []),
            findConflicts=lambda *args: self.conflicts,
            addAssignment=self.add_key, removeAssignment=self.remove_key, saveOverrides=Mock())
        self.hou = NS(ObjectWasDeleted=Deleted, paneTabType=NS(Parm="parm"),
                      isUIAvailable=lambda: True, session=NS(), node=lambda p: NS(path=lambda: p),
                      hotkeys=self.hotkeys, severityType=NS(Warning="warning"),
                      ui=NS(paneTabs=lambda: self.panes, curDesktop=lambda: NS(createFloatingPaneTab=self.create),
                            setStatusMessage=Mock()))
        self.events = NS(Move=1, Resize=2, Close=3)
        qt = NS(QtCore=NS(QObject=QObject, QTimer=Mock(), QEvent=NS(Type=self.events)),
                QtWidgets=NS(QApplication=NS(instance=lambda: None)))
        spec = importlib.util.spec_from_file_location("ko_parameters_under_test", ROOT / "python3.13libs/ko_parameters.py")
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {"hou": self.hou, "hutil": NS(Qt=qt), "hutil.Qt": qt,
                                     "shiboken6": NS(isValid=lambda obj: obj.valid)}):
            spec.loader.exec_module(self.module)

    def add_key(self, context, symbol, key):
        self.bindings.setdefault(symbol, []).append(key)
        return True

    def remove_key(self, context, symbol, key):
        self.bindings[symbol].remove(key)
        return True

    def pane(self, name="KoParameters", docked=False, tabs=1):
        window = Mock(valid=True)
        window.isVisible.return_value = True
        window.isMinimized.return_value = window.isMaximized.return_value = window.isFullScreen.return_value = False
        floating = Mock()
        floating.position.return_value = (-1200, 80)
        floating.size.return_value = (820, 740)
        floating.qtParentWindow.return_value = window
        pane = Mock(live=True)
        def get_type():
            if not pane.live:
                raise Deleted("closed")
            return "parm"
        pane.type.side_effect = get_type
        pane.name.return_value = name
        pane.floatingPanel.return_value = None if docked else floating
        pane.currentNode.return_value = NS(path=lambda: "/obj/example")
        pane.isPin.return_value = True
        floating.paneTabs.return_value = [pane] * tabs
        floating.close.side_effect = lambda: setattr(pane, "live", False)
        self.panes.append(pane)
        return pane

    def create(self, kind, **kwargs):
        self.created.append(kwargs)
        return self.pane()

    def test_toggle_closes_then_creates_at_saved_native_geometry(self):
        pane = self.pane()
        self.assertIsNone(self.module.toggle_parameters())
        pane.floatingPanel().close.assert_called_once()
        self.panes.clear()
        reopened = self.module.toggle_parameters()
        self.assertEqual(self.created, [{"immediate": True, "position": (-1200, 80), "size": (820, 740)}])
        reopened.floatingPanel().setPosition.assert_called_once_with((-1200, 80))
        reopened.floatingPanel().setSize.assert_called_once_with((820, 740))
        reopened.currentNode.assert_called()
        reopened.setCurrentNode.assert_called_once()
        self.assertEqual(reopened.setCurrentNode.call_args.kwargs, {"pick_node": False})
        reopened.showNetworkControls.assert_called_once_with(False)
        self.assertEqual(json.loads(self.path.read_text())["size"], [820, 740])

    def test_stale_hom_with_live_qt_wrapper_is_not_reused(self):
        pane = self.pane()
        controller = self.module._controller()
        controller.attach(pane)
        pane.live = False
        self.panes.clear()
        self.assertTrue(controller.window.valid)
        reopened = self.module.toggle_parameters()
        self.assertIsNot(reopened, pane)
        self.assertEqual(len(self.created), 1)

    def test_native_close_event_flushes_then_detaches_before_teardown(self):
        pane = self.pane()
        controller = self.module._controller()
        controller.attach(pane)
        window = controller.window
        pane.floatingPanel().position.return_value = (210, 130)
        controller.eventFilter(window, NS(type=lambda: self.events.Close))
        self.assertIsNone(controller.pane)
        self.assertEqual(json.loads(self.path.read_text())["position"], [210, 130])
        pane.floatingPanel().position.return_value = (0, 0)
        controller.eventFilter(window, NS(type=lambda: self.events.Resize))
        self.assertEqual(json.loads(self.path.read_text())["position"], [210, 130])

    def test_minimized_and_maximized_do_not_overwrite_normal_geometry(self):
        pane = self.pane()
        controller = self.module._controller()
        controller.attach(pane)
        controller.flush()
        before = self.path.read_bytes()
        pane.floatingPanel().size.return_value = (1920, 1080)
        controller.window.isMaximized.return_value = True
        controller.flush()
        self.assertEqual(self.path.read_bytes(), before)
        controller.window.isMaximized.return_value = False
        controller.window.isMinimized.return_value = True
        controller.flush()
        self.assertEqual(self.path.read_bytes(), before)

    def test_startup_adopts_legacy_without_opening_and_removes_old_filter(self):
        pane = self.pane("KoRestoreParameters")
        old = NS(window=Mock(valid=True), deleteLater=Mock())
        self.hou.session._ko_parameter_tracker = old
        self.module.install()
        self.assertEqual(self.created, [])
        self.assertIs(self.module._controller().pane, pane)
        old.window.removeEventFilter.assert_called_once_with(old)
        old.deleteLater.assert_called_once()
        self.assertIsNone(self.hou.session._ko_parameter_tracker)

    def test_docked_and_multi_tab_parameter_workspaces_are_untouched(self):
        docked = self.pane("KoParameters", docked=True)
        multi = self.pane("FloatingPanel", tabs=2)
        self.module.toggle_parameters()
        self.assertTrue(docked.live)
        multi.floatingPanel().close.assert_not_called()
        self.assertEqual(len(self.created), 1)

    def test_geometry_validation_and_corrupt_preferences(self):
        for value in ({}, {"position": [1, 2], "size": [0, 30]},
                      {"position": [float("nan"), 2], "size": [10, 30]},
                      {"position": [True, 2], "size": [10, 30]}):
            self.assertEqual(self.module._geometry(value), {})
        self.path.write_text("bad json")
        self.module.toggle_parameters()
        self.assertEqual(self.created, [{"immediate": True}])

    def test_atomic_write_failure_keeps_previous_state(self):
        self.path.write_text('{"previous": true}')
        before = self.path.read_bytes()
        with patch.object(self.module.os, "replace", side_effect=OSError("disk")):
            with self.assertRaises(OSError):
                self.module._write({"new": True})
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(list(self.path.parent.glob("*.tmp")), [])

    def test_shortcut_migration_only_replaces_old_own_key(self):
        self.bindings[self.module.LEGACY_SYMBOL] = ["Ctrl+Shift+P", "Alt+P"]
        self.conflicts = ["h?" + self.module.LEGACY_SYMBOL]
        self.module.configure_hotkey()
        self.assertEqual(self.bindings[self.module.MENU_SYMBOL], ["Ctrl+Shift+P"])
        self.assertEqual(self.bindings[self.module.LEGACY_SYMBOL], ["Alt+P"])
        self.hotkeys.saveOverrides.assert_called_once()

    def test_shortcut_conflicts_and_user_custom_menu_binding_are_preserved(self):
        self.conflicts = ["h.pane.parms?h.other_action"]
        self.module.configure_hotkey()
        self.assertNotIn(self.module.MENU_SYMBOL, self.bindings)
        self.hotkeys.saveOverrides.assert_not_called()
        self.hou.ui.setStatusMessage.assert_called_once()
        self.bindings[self.module.MENU_SYMBOL] = ["Alt+Shift+P"]
        self.module.configure_hotkey()
        self.assertEqual(self.bindings[self.module.MENU_SYMBOL], ["Alt+Shift+P"])

    def test_menu_points_to_module_and_obsolete_shelf_tool_is_removed(self):
        menu = ET.parse(ROOT / "MainMenuCommon.xml").find(".//scriptItem[@id='h.ko_toggle_parameters']")
        self.assertEqual(menu.findtext("label"), "Toggle Parameters")
        self.assertIn("ko_parameters.toggle_parameters()", menu.findtext("scriptCode"))
        shelf = ET.parse(ROOT / "toolbar/ko_network.shelf").getroot()
        self.assertIsNone(shelf.find("tool[@name='ko_restore_parameters']"))
        self.assertIsNone(shelf.find(".//memberTool[@name='ko_restore_parameters']"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
