"""Offscreen Qt + real HOM; no changes to an existing Houdini process."""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python3.13libs"))
import hou
from PySide6 import QtCore, QtGui, QtTest, QtWidgets
from ko_ai.panel import Panel, CompareDialog, TextInputDialog, create_panel

app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class FakeTransport:
    def __init__(self):
        self.replies = []
    def reply(self, request_id, result):
        self.replies.append((request_id, result))
    def reject(self, request_id, message):
        self.replies.append((request_id, {"error": message}))
    def close(self):
        pass


class UiTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="ko-ai-ui-")
        self.previous_home = os.environ.get("KO_AI_HOME")
        os.environ["KO_AI_HOME"] = self.temp.name
        hou.hipFile.clear(suppress_save_prompt=True)
        hou.hipFile.setName(str(Path(self.temp.name) / "working.hip"))
        self.panel = Panel()
        self.panel.resize(820, 900)
        self.panel.show()
        app.processEvents()

    def tearDown(self):
        self.panel.close()
        app.processEvents()
        hou.hipFile.clear(suppress_save_prompt=True)
        if self.previous_home is None:
            os.environ.pop("KO_AI_HOME", None)
        else:
            os.environ["KO_AI_HOME"] = self.previous_home
        self.temp.cleanup()

    def test_name_dialog_accepts_return_and_keypad_enter(self):
        for key, modifier in ((QtCore.Qt.Key.Key_Return, QtCore.Qt.KeyboardModifier.NoModifier),
                              (QtCore.Qt.Key.Key_Enter, QtCore.Qt.KeyboardModifier.KeypadModifier)):
            with self.subTest(key=key):
                dialog = TextInputDialog(self.panel, "Checkpoint", "Name this version:")
                try:
                    accepted = []
                    dialog.accepted.connect(lambda: accepted.append(True))
                    dialog.show()
                    app.processEvents()
                    QtTest.QTest.keyClicks(dialog.input, "Original")
                    override = QtGui.QKeyEvent(QtCore.QEvent.Type.ShortcutOverride, key, modifier)
                    QtWidgets.QApplication.sendEvent(dialog.input, override)
                    self.assertTrue(override.isAccepted())
                    QtTest.QTest.keyClick(dialog.input, key, modifier)
                    self.assertEqual(dialog.result(), QtWidgets.QDialog.DialogCode.Accepted)
                    self.assertEqual(dialog.input.text(), "Original")
                    self.assertEqual(len(accepted), 1)
                finally:
                    dialog.close()
                    dialog.deleteLater()

    def test_checkpoint_create_rename_and_cancel_using_keyboard(self):
        def answer(text, key):
            def respond():
                dialog = QtWidgets.QApplication.activeModalWidget()
                if isinstance(dialog, TextInputDialog):
                    dialog.input.setText(text)
                    QtTest.QTest.keyClick(dialog.input, key)
                elif dialog:
                    dialog.reject()
            QtCore.QTimer.singleShot(20, respond)
        answer("Original", QtCore.Qt.Key.Key_Return)
        self.panel.make_checkpoint()
        self.assertEqual(len(self.panel.workspace.history.entries), 1)
        self.assertEqual(self.panel.workspace.history.entries[0]["label"], "Original")
        self.panel.versions.topLevelItem(0).setSelected(True)
        answer("Renamed", QtCore.Qt.Key.Key_Enter)
        self.panel.rename_checkpoint()
        self.assertEqual(self.panel.workspace.history.entries[0]["label"], "Renamed")
        answer("Do not save", QtCore.Qt.Key.Key_Escape)
        self.panel.make_checkpoint()
        self.assertEqual(len(self.panel.workspace.history.entries), 1)

    def test_tree_diff_and_chat(self):
        geo = hou.node("/obj").createNode("geo", "candidate", run_init_scripts=False)
        box = geo.createNode("box", "box1")
        a = self.panel.workspace.checkpoint("Original")
        box.parm("sizex").set(3)
        b = self.panel.workspace.checkpoint("Wider candidate")
        self.panel.refresh_versions()
        self.assertEqual(self.panel.versions.topLevelItemCount(), 1)
        self.assertEqual(self.panel.versions.topLevelItem(0).childCount(), 1)
        dialog = CompareDialog(self.panel.workspace, a["id"], b["id"], self.panel)
        self.assertIn('/obj/candidate/box1', dialog.findChildren(QtWidgets.QPlainTextEdit)[0].toPlainText())
        dialog.close()
        self.panel._append("You", "Explain this box and propose a wider version.")
        self.panel._event({"method": "item/agentMessage/delta", "params": {"itemId": "x", "delta": "A wider "}})
        self.panel._event({"method": "item/agentMessage/delta", "params": {"itemId": "x", "delta": "box."}})
        self.panel._event({"method": "item/completed", "params": {"item": {
            "type": "agentMessage", "id": "x", "text": "A wider box."}}})
        self.assertEqual(self.panel.messages[-1]["text"], "A wider box.")
        self.panel.tabs.setCurrentIndex(2)
        app.processEvents()
        screenshot = Path(tempfile.gettempdir()) / "ko-ai-ui-smoke.png"
        self.assertTrue(self.panel.grab().save(str(screenshot)))
        print("UI screenshot:", screenshot)

    def test_duplicate_pane_shows_clean_message_and_retry_without_disrupting_owner(self):
        self.panel._append("You", "Keep my conversation")
        with patch.object(QtWidgets.QMessageBox, "warning") as warning:
            host = create_panel()
            try:
                self.assertIsNone(host._content)
                self.assertIn("doesn’t support two panes at once", host.message.text())
                self.assertIn("Retry", host.message.text())
                host.resize(650, 250)
                host.show()
                app.processEvents()
                host.grab().save(str(Path(tempfile.gettempdir()) / "ko-ai-duplicate-smoke.png"))
                self.assertFalse(self.panel._closed)
                self.assertEqual(self.panel.messages[-1]["text"], "Keep my conversation")
                host.retry()
                self.assertIsNone(host._content)
                self.panel.close()
                host.retry()
                self.assertIsNotNone(host._content)
                self.assertEqual(host._content.messages[-1]["text"], "Keep my conversation")
                warning.assert_not_called()
            finally:
                host.close()
                host.deleteLater()
                app.processEvents()

    def test_duplicate_native_interface_does_not_reload_first_controller(self):
        import ko_ai
        import xml.etree.ElementTree as ET
        source = ET.parse(Path(__file__).resolve().parents[2] / "python_panels/ko_ai.pypanel").find("interface/script").text
        namespace = {}
        exec(compile(source, "ko_ai.pypanel", "exec"), namespace)
        with patch.object(ko_ai, "reload_modules") as reload_modules:
            host = namespace["onCreateInterface"]()
            try:
                self.assertIsNone(host._content)
                self.assertIn("two panes", host.message.text())
                reload_modules.assert_not_called()
                self.assertFalse(self.panel._closed)
            finally:
                namespace["onDestroyInterface"]()
                host.deleteLater()
                app.processEvents()

    def test_reload_prefers_legacy_owner_over_failed_second_pane(self):
        import ko_ai
        from unittest.mock import Mock
        interface = Mock()
        interface.name.return_value = "ko_ai_workspace"
        owner = Mock()
        owner.type.return_value = hou.paneTabType.PythonPanel
        owner.activeInterface.return_value = interface
        owner.activeInterfaceRootWidget.return_value = self.panel
        notice_widget = Mock(_content=None, _closed=False, busy=False)
        notice_widget.objectName.return_value = "ko_ai_host"
        notice = Mock()
        notice.type.return_value = hou.paneTabType.PythonPanel
        notice.activeInterface.return_value = interface
        notice.activeInterfaceRootWidget.return_value = notice_widget
        with patch.object(hou, "isUIAvailable", return_value=True), \
             patch.object(hou, "ui", Mock(paneTabs=Mock(return_value=[notice, owner])), create=True), \
             patch.object(hou, "pypanel", Mock(interfaceByName=Mock(return_value=interface)), create=True) as pypanel:
            ko_ai.reload()
            pypanel.installFile.assert_not_called()
            interface.setScript.assert_called_once()
        owner.reloadActiveInterface.assert_called_once()
        notice.reloadActiveInterface.assert_not_called()

    def test_reload_repairs_pane_with_missing_interface_binding(self):
        import ko_ai
        from unittest.mock import Mock
        pane = Mock()
        pane.type.return_value = hou.paneTabType.PythonPanel
        pane.activeInterface.return_value = None
        pane.activeInterfaceRootWidget.return_value = self.panel
        definition = Mock()
        definition.name.return_value = "ko_ai_workspace"
        with patch.object(hou, "isUIAvailable", return_value=True), \
             patch.object(hou, "ui", Mock(paneTabs=Mock(return_value=[pane])), create=True), \
             patch.object(hou, "pypanel", Mock(interfaceByName=Mock(return_value=definition)), create=True) as pypanel:
            ko_ai.reload()
            pypanel.installFile.assert_not_called()
        pane.setActiveInterface.assert_called_once_with(definition)
        pane.reloadActiveInterface.assert_not_called()

    def test_reload_recovers_single_unbound_panel_from_older_tool(self):
        import ko_ai
        from unittest.mock import Mock
        pane = Mock()
        pane.name.return_value = "legacy_unbound_ko_ai"
        pane.label.return_value = "Python Panel"
        pane.type.return_value = hou.paneTabType.PythonPanel
        pane.activeInterface.return_value = None
        pane.activeInterfaceRootWidget.return_value = None
        definition = Mock()
        definition.name.return_value = "ko_ai_workspace"
        with patch.object(hou, "isUIAvailable", return_value=True), \
             patch.object(hou, "ui", Mock(paneTabs=Mock(return_value=[pane])), create=True), \
             patch.object(hou, "pypanel", Mock(interfaceByName=Mock(return_value=definition)), create=True):
            ko_ai.reload()
        pane.setActiveInterface.assert_called_once_with(definition)

    def test_action_errors_are_inline_not_popup_alerts(self):
        def fail():
            raise ValueError("Select one checkpoint to compare")
        with patch.object(QtWidgets.QMessageBox, "warning") as warning, \
             patch.object(QtWidgets.QMessageBox, "information") as information, \
             patch("traceback.print_exc") as print_exc:
            self.panel._guard(fail)
            self.assertEqual(self.panel.status.text(), "Select one checkpoint to compare")
            self.assertIn("ValueError", self.panel.error_details.toPlainText())
            warning.assert_not_called()
            information.assert_not_called()
            print_exc.assert_not_called()

    def test_cleanup_needs_no_selection_and_preserves_unsaved_current_scene(self):
        geo = hou.node("/obj").createNode("geo", "candidate", run_init_scripts=False)
        box = geo.createNode("box", "box1")
        original = self.panel.workspace.checkpoint("Original")
        box.parm("sizex").set(3)
        updated = self.panel.workspace.replace(original["id"])
        self.panel.workspace.history.pin(original["id"], True)
        self.panel.workspace.return_target = original["id"]
        box.parm("sizex").set(7)
        self.panel.refresh_versions()
        self.panel.versions.clearSelection()
        with patch.object(QtWidgets.QMessageBox, "warning") as warning:
            self.panel.clean_up()
            self.assertFalse(self.panel.cleanup_box.isHidden())
            self.assertEqual(len(self.panel.workspace.history.entries), 2)
            self.assertIn("1 pinned", self.panel.cleanup_summary.text())
            self.panel.tabs.setCurrentIndex(2)
            app.processEvents()
            self.panel.grab().save(str(Path(tempfile.gettempdir()) / "ko-ai-cleanup-smoke.png"))
            self.panel.cancel_cleanup()
            self.assertEqual(len(self.panel.workspace.history.entries), 2)
            self.panel.clean_up()
            self.panel.confirm_cleanup()
            warning.assert_not_called()
        history = self.panel.workspace.history
        self.assertEqual(len(history.entries), 1)
        self.assertEqual(history.entries[0]["label"], "Original")
        self.assertTrue(history.verify(self.panel.workspace.active).exists())
        self.assertEqual(box.parm("sizex").eval(), 7)
        self.assertFalse(history.directory(original["id"]).exists())
        self.assertFalse(history.directory(updated["id"]).exists())
        self.assertIsNone(self.panel.workspace.return_target)
        self.assertNotIn("replaced_by", history.entries[0])
        self.panel.close()
        self.panel = Panel()
        self.assertEqual(len(self.panel.workspace.history.entries), 1)

    def test_cleanup_unchanged_current_reuses_checkpoint(self):
        geo = hou.node("/obj").createNode("geo", "candidate", run_init_scripts=False)
        box = geo.createNode("box", "box1")
        original = self.panel.workspace.checkpoint("Original")
        box.parm("sizex").set(3)
        current = self.panel.workspace.checkpoint("Wide")
        self.panel.clean_up()
        self.panel.confirm_cleanup()
        self.assertEqual(self.panel.workspace.active, current["id"])
        self.assertEqual(len(self.panel.workspace.history.entries), 1)
        self.assertFalse(self.panel.workspace.history.directory(original["id"]).exists())

    def test_cleanup_refuses_stale_preview_and_save_failure_deletes_nothing(self):
        from ko_ai.scene import Workspace
        geo = hou.node("/obj").createNode("geo", "candidate", run_init_scripts=False)
        box = geo.createNode("box", "box1")
        original = self.panel.workspace.checkpoint("Original")
        box.parm("sizex").set(3)
        current = self.panel.workspace.checkpoint("Wide")
        self.panel.clean_up()
        box.parm("sizex").set(7)
        self.panel._guard(self.panel.confirm_cleanup)
        self.assertIn("Scene or history changed", self.panel.status.text())
        self.assertEqual(len(self.panel.workspace.history.entries), 2)
        self.panel.clean_up()
        with patch.object(Workspace, "checkpoint", side_effect=OSError("disk full")):
            self.panel._guard(self.panel.confirm_cleanup)
        self.assertIn("disk full", self.panel.status.text())
        self.assertEqual(len(self.panel.workspace.history.entries), 2)
        self.assertTrue(self.panel.workspace.history.verify(original["id"]).exists())
        self.assertTrue(self.panel.workspace.history.verify(current["id"]).exists())
        self.assertEqual(box.parm("sizex").eval(), 7)

    def test_tool_request_runs_on_ui_and_read_only_is_enforced(self):
        fake = FakeTransport()
        self.panel.transport = fake
        self.panel._turn_workspace = self.panel.workspace.history.id
        self.panel._set_busy(True)
        self.panel._event({"id": 91, "method": "item/tool/call", "params": {
            "tool": "ko_scene", "arguments": {"operation": "context", "path": "/", "query": "", "offset": 0, "limit": 1}}})
        self.assertTrue(fake.replies[-1][1]["success"])
        self.panel._event({"id": 92, "method": "item/tool/call", "params": {
            "tool": "ko_propose", "arguments": {"label": "test", "steps_json": "[]"}}})
        self.assertFalse(fake.replies[-1][1]["success"])
        self.panel._event({"id": 93, "method": "item/commandExecution/requestApproval", "params": {}})
        self.assertEqual(fake.replies[-1][1], {"decision": "decline"})
        self.panel._set_busy(False)

    def test_replace_selected_updates_normal_list_and_exposes_old_revision(self):
        geo = hou.node("/obj").createNode("geo", "candidate", run_init_scripts=False)
        box = geo.createNode("box", "box1")
        original = self.panel.workspace.checkpoint("Original")
        self.panel.refresh_versions()
        self.panel.versions.topLevelItem(0).setSelected(True)
        box.parm("sizex").set(7)
        with patch.object(QtWidgets.QMessageBox, "question", return_value=QtWidgets.QMessageBox.StandardButton.Yes):
            self.panel.replace_checkpoint()
        self.assertEqual(self.panel.versions.topLevelItemCount(), 1)
        self.assertEqual(len(self.panel.workspace.history.entries), 2)
        active = self.panel.workspace.active
        self.assertNotEqual(active, original["id"])
        self.assertEqual(self.panel.selected_ids(), [active])
        self.assertIn("Original", self.panel.versions.topLevelItem(0).text(0))
        self.assertNotIn("replaced revision", self.panel.versions.topLevelItem(0).text(0))
        self.panel.show_replaced_check.setChecked(True)
        self.assertEqual(self.panel.versions.topLevelItemCount(), 2)
        self.assertTrue(any("replaced revision" in self.panel.versions.topLevelItem(i).text(0)
                            for i in range(2)))
        self.panel.workspace.visit(original["id"])
        self.assertEqual(hou.node("/obj/candidate/box1").parm("sizex").eval(), 1)
        self.panel.workspace.visit(active)
        self.assertEqual(hou.node("/obj/candidate/box1").parm("sizex").eval(), 7)
        self.assertEqual(len(self.panel.workspace.history.entries), 2)

    def test_reload_releases_workspace_lock_and_preserves_chat(self):
        self.panel._append("You", "Remember this conversation")
        root = self.panel.workspace.history.root
        self.panel.close()
        self.panel = Panel()
        self.assertEqual(self.panel.workspace.history.root, root)
        self.assertEqual(self.panel.messages[-1]["text"], "Remember this conversation")

    def test_scene_change_mid_turn_does_not_rebind_or_mix_chat(self):
        self.panel._append("You", "Old-scene conversation")
        old = self.panel.workspace.history.id
        self.panel._set_busy(True)
        hou.hipFile.clear(suppress_save_prompt=True)
        with self.assertRaisesRegex(RuntimeError, "scene changed"):
            self.panel._ensure_binding()
        self.assertEqual(self.panel.workspace.history.id, old)
        self.panel._set_busy(False)
        self.panel._ensure_binding()
        self.assertNotEqual(self.panel.workspace.history.id, old)
        self.assertEqual(self.panel.messages, [])

    def test_model_switch_is_persisted_and_sent_on_each_turn(self):
        models = {"data": [
            {"model": "test-model-a", "displayName": "Test A", "isDefault": True},
            {"model": "test-model-b", "displayName": "Test B", "isDefault": False}]}
        self.panel._models(models)
        self.panel.model.setCurrentIndex(1)
        self.panel._remember_model()
        requests = []
        self.panel._request = lambda method, params, callback: requests.append((method, params))
        self.panel.thread_id = "test-thread"
        self.panel._turn_user_text = "Explain the scene"
        self.panel._start_turn()
        self.assertEqual(requests[-1][1]["model"], "test-model-b")
        self.panel.model.setCurrentIndex(0)
        self.panel._start_turn()
        self.assertEqual(requests[-1][1]["model"], "test-model-a")
        self.panel.close()
        self.panel = Panel()
        self.panel._models(models)
        self.assertEqual(self.panel.model.currentData(), "test-model-b")

    def test_proposals_survive_panel_reload(self):
        proposal = self.panel.tools.propose("Create a geo", [
            {"op": "create", "parent": "/obj", "type": "geo", "name": "test"}])
        self.panel.close()
        self.panel = Panel()
        self.assertIn(proposal["proposal"], self.panel.tools.proposals)
        self.assertEqual(self.panel.proposal_list.count(), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
