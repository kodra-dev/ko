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
from loom.panel import Panel, CompareDialog, TextInputDialog, create_panel
from loom.checkpoints.panel import Panel as CheckpointPanel

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
        self.temp = tempfile.TemporaryDirectory(prefix="loom-ui-")
        self.previous_home = os.environ.get("LOOM_HOME")
        os.environ["LOOM_HOME"] = self.temp.name
        hou.hipFile.clear(suppress_save_prompt=True)
        hou.hipFile.setName(str(Path(self.temp.name) / "working.hip"))
        self.panel = Panel()
        self.checkpoints = CheckpointPanel()
        self.checkpoints.resize(720, 700)
        self.checkpoints.show()
        self.panel.resize(820, 900)
        self.panel.show()
        app.processEvents()

    def tearDown(self):
        self.checkpoints.close()
        self.panel.close()
        app.processEvents()
        hou.hipFile.clear(suppress_save_prompt=True)
        if self.previous_home is None:
            os.environ.pop("LOOM_HOME", None)
        else:
            os.environ["LOOM_HOME"] = self.previous_home
        self.temp.cleanup()

    def test_compact_usage_is_right_aligned_persisted_and_thread_scoped(self):
        self.assertEqual(self.panel.allow_edits.text(), "Edit scene")
        self.assertTrue(any(b.text() == "Context" for b in self.panel.findChildren(QtWidgets.QPushButton)))
        self.assertTrue(self.panel.usage_label.alignment() & QtCore.Qt.AlignmentFlag.AlignRight)
        row = self.panel.usage_label.parentWidget().layout().itemAt(
            self.panel.usage_label.parentWidget().layout().count() - 1).layout()
        self.assertIs(row.itemAt(row.count() - 1).widget(), self.panel.usage_label)
        self.assertIsNone(row.itemAt(row.count() - 2).widget())  # stretch before rightmost label
        self.panel.thread_id = "usage-thread"
        self.panel._turn_usage_model = "gpt-5"
        counts = {"inputTokens": 100000, "cachedInputTokens": 40000,
                  "outputTokens": 4000, "reasoningOutputTokens": 2000, "totalTokens": 104000}
        event = {"method": "thread/tokenUsage/updated", "params": {
            "threadId": "usage-thread", "turnId": "turn-a", "tokenUsage": {
                "total": counts, "last": counts, "modelContextWindow": 742857}}}
        self.panel._event(event)
        self.panel._event(event)
        self.assertEqual(self.panel.usage_label.text(), "≈$0.12 C14%")
        self.assertEqual(self.panel.messages, [])
        self.panel._save_chat()
        self.panel._load_chat()
        self.assertEqual(self.panel.usage_label.text(), "≈$0.12 C14%")
        self.panel.new_chat()
        self.assertEqual(self.panel.usage_label.text(), "≈$0.00 C—%")
        self.panel._event(event)  # delayed notification from the old conversation
        self.assertEqual(self.panel.usage_label.text(), "≈$0.00 C—%")

    def test_usage_model_is_frozen_for_turn_and_tracks_rerouting(self):
        self.panel.thread_id = "usage-thread"
        self.panel._turn_usage_model = "gpt-5"
        self.panel._event({"method": "turn/started", "params": {"turn": {"id": "turn-a"}}})
        self.panel._turn_usage_model = "other-model"
        counts = {"inputTokens": 100000, "cachedInputTokens": 40000,
                  "outputTokens": 4000, "totalTokens": 104000}
        event = {"method": "thread/tokenUsage/updated", "params": {
            "threadId": "usage-thread", "turnId": "turn-a", "tokenUsage": {
                "total": counts, "last": counts, "modelContextWindow": 742857}}}
        self.panel._event(event)
        self.assertEqual(self.panel.usage_label.text(), "≈$0.12 C14%")
        self.panel._event({"method": "model/rerouted", "params": {
            "threadId": "usage-thread", "turnId": "turn-a", "toModel": "gpt-5.4"}})
        doubled = {key: value * 2 for key, value in counts.items()}
        event["params"]["tokenUsage"]["total"] = doubled
        self.panel._event(event)
        self.assertEqual(self.panel.usage_label.text(), "≈$0.34 C14%")

    def test_composer_single_click_enter_shift_enter_and_keypad(self):
        from unittest.mock import Mock
        self.checkpoints.activateWindow()
        self.checkpoints.versions.setFocus()
        app.processEvents()
        self.assertFalse(self.panel.prompt.hasFocus())
        QtTest.QTest.mouseClick(self.panel.prompt.viewport(), QtCore.Qt.MouseButton.LeftButton)
        app.processEvents()
        self.assertTrue(self.panel.prompt.hasFocus())
        self.panel.send_prompt = Mock()
        QtTest.QTest.keyClicks(app.focusWidget(), "First")
        QtTest.QTest.keyClick(self.panel.prompt, QtCore.Qt.Key.Key_Return, QtCore.Qt.KeyboardModifier.ShiftModifier)
        QtTest.QTest.keyClicks(app.focusWidget(), "Second")
        self.assertEqual(self.panel.prompt.toPlainText(), "First\nSecond")
        self.panel.send_prompt.assert_not_called()
        for key in (QtCore.Qt.Key.Key_Return, QtCore.Qt.Key.Key_Enter):
            override = QtGui.QKeyEvent(QtCore.QEvent.Type.ShortcutOverride, key, QtCore.Qt.KeyboardModifier.NoModifier)
            app.sendEvent(self.panel.prompt, override)
            self.assertTrue(override.isAccepted())
            QtTest.QTest.keyClick(self.panel.prompt, key)
        self.assertEqual(self.panel.send_prompt.call_count, 2)
        self.assertEqual(self.panel.prompt.toPlainText(), "First\nSecond")
        # Return while an IME composition is active must not accidentally send.
        app.sendEvent(self.panel.prompt, QtGui.QInputMethodEvent("composing", []))
        QtTest.QTest.keyClick(self.panel.prompt, QtCore.Qt.Key.Key_Return)
        self.assertEqual(self.panel.send_prompt.call_count, 2)

    def test_chat_markdown_and_collapsed_activity_are_not_a_flat_log(self):
        self.panel._append("You", "What is my shortcut?")
        self.panel._append("Tool", '{"keys":["Alt+E"]}', summary="Inspected local hotkeys", success=True)
        self.panel._append("Tool", '{"path":"/obj/cube"}', summary="Inspected scene", success=True)
        self.panel._append("Loom", "**Alt+E** invokes `PolyExtrude`.\n\n- Your live keymap\n- Geometry viewer context")
        app.processEvents()
        cards = self.panel.transcript.cards
        self.assertEqual(len(cards), 3)
        self.assertEqual(len(self.panel.messages), 4)  # grouping is presentation only
        self.assertEqual(cards[1].title.text(), "2 actions")
        self.assertFalse(cards[1].body.isVisible())
        cards[1].toggle.click()
        self.assertTrue(cards[1].body.isVisible())
        self.assertIn('Alt+E', cards[1].body.toPlainText())
        self.assertIn("Alt+E", cards[2].body.toPlainText())
        self.assertNotIn("**", cards[2].body.toPlainText())
        self.assertFalse(cards[2].body.openLinks())
        self.assertIsNone(cards[2].body.loadResource(2, QtCore.QUrl("file:///not-allowed.png")))
        first = cards[0]
        self.panel._event({"method": "item/agentMessage/delta", "params": {"itemId": "stream", "delta": "More"}})
        self.assertIs(self.panel.transcript.cards[0], first)

    def test_failed_tool_is_visible_with_expandable_actual_error(self):
        self.panel.transport = FakeTransport()
        self.panel._turn_workspace = self.panel.workspace.history.id
        self.panel._set_busy(True)
        self.panel._event({"id": 71, "method": "item/tool/call", "params": {
            "tool": "loom_scene", "arguments": {"operation": "node", "path": "/does_not_exist"}}})
        self.assertFalse(self.panel.transport.replies[-1][1]["success"])
        self.assertFalse(self.panel.messages[-1]["success"])
        card = self.panel.transcript.cards[-1]
        self.assertIn("Tool failed", card.title.text())
        self.assertTrue(card.toggle.isChecked())
        self.assertIn("Node not found", card.body.toPlainText())

    def test_tidy_button_freezes_scope_preserves_draft_and_requires_permission(self):
        box = self._cube()
        self.panel.ready = True
        self.panel._request = lambda *args: None
        self.panel.prompt.setPlainText("Unsent draft")
        with self.assertRaises(PermissionError):
            self.panel.tidy_network()
        self.panel.allow_edits.setChecked(True)
        self.panel._set_busy(False)  # Refresh controls after the mocked connection became ready.
        with patch.object(hou, "selectedNodes", return_value=(box,)):
            self.panel.tidy_button.click()
        self.assertTrue(self.panel.busy)
        self.assertFalse(self.panel.tidy_button.isEnabled())
        self.assertEqual(self.panel.tools.tidy_scope.nodes, {box})
        self.assertEqual(self.panel.prompt.toPlainText(), "Unsent draft")
        self.assertIn("Tidy Network", self.panel.messages[-1]["text"])
        self.panel._set_busy(False)
        self.assertIsNone(self.panel.tools.tidy_scope)

    def _start_test_turn(self, allowed=False):
        self.panel.ready = True
        self.panel.thread_id = "fake-thread"
        self.panel._resumed = True
        self.panel._request = lambda *args: None
        self.panel.allow_edits.setChecked(allowed)
        self.panel.prompt.setPlainText("Set the cube Size X to 3")
        self.panel.send_prompt()
        self.panel.transport = FakeTransport()

    def _send_size_proposal(self, request_id=72, **extra):
        import json
        self.panel._event({"id": request_id, "method": "item/tool/call", "params": {
            "tool": "loom_propose", "arguments": {"label": "Wider cube",
                "steps_json": json.dumps([{"op": "set", "path": "/obj/cube/box1", "parm": "sizex", "value": 3}]),
                **extra}}})

    def _cube(self):
        return hou.node("/obj").createNode("geo", "cube", run_init_scripts=False).createNode("box", "box1")

    def test_answer_propose_only_does_not_apply_even_if_model_claims_permission(self):
        box = self._cube()
        self._start_test_turn()
        self._send_size_proposal(allow_scene_edits=True)
        self.assertFalse(self.panel.transport.replies[-1][1]["success"])
        self.assertEqual(box.parm("sizex").eval(), 1)
        self.assertEqual(len(self.panel.workspace.history.entries), 0)
        self.assertEqual(self.panel.tools.proposals, {})
        self.assertFalse(hasattr(self.panel, "proposal_review"))

    def test_allowed_edits_apply_with_checkpoints_without_a_second_confirmation(self):
        box = self._cube()
        self._start_test_turn(allowed=True)
        self.assertFalse(self.panel.allow_edits.isEnabled())
        with patch.object(QtWidgets.QMessageBox, "question") as confirm:
            self._send_size_proposal()
            confirm.assert_not_called()
        self.assertEqual(box.parm("sizex").eval(), 3)
        self.assertEqual(len(self.panel.workspace.history.entries), 2)
        self.assertTrue(self.panel.transport.replies[-1][1]["success"])
        self.assertEqual(next(iter(self.panel.tools.proposals.values()))["status"], "applied")
        self.assertFalse(hasattr(self.panel, "proposal_review"))

    def test_legacy_proposal_calls_from_resumed_threads_keep_the_same_edit_grant(self):
        import json
        box = self._cube()
        self._start_test_turn(allowed=True)
        self.panel._event({"id": 74, "method": "item/tool/call", "params": {
            "tool": "ko_propose", "arguments": {"label": "Legacy conversation edit",
                "steps_json": json.dumps([{"op": "set", "path": "/obj/cube/box1", "parm": "sizex", "value": 3}])}}})
        self.assertEqual(box.parm("sizex").eval(), 3)
        self.assertTrue(self.panel.transport.replies[-1][1]["success"])
        self.assertEqual(next(iter(self.panel.tools.proposals.values()))["status"], "applied")

    def test_auto_apply_failure_revokes_grant_for_rest_of_turn(self):
        box = self._cube()
        self._start_test_turn(allowed=True)
        with patch.object(self.panel.tools, "apply", side_effect=OSError("cannot save checkpoint")) as apply:
            self._send_size_proposal()
            self.assertFalse(self.panel.transport.replies[-1][1]["success"])
            self.assertFalse(self.panel._turn_allow_edits)
            self._send_size_proposal(request_id=73)
            self.assertEqual(apply.call_count, 1)
        self.assertEqual(box.parm("sizex").eval(), 1)

    def test_new_workspace_starts_off_but_reopening_restores_its_edit_preference(self):
        self.panel.allow_edits.setChecked(True)
        hou.hipFile.clear(suppress_save_prompt=True)
        self.panel._ensure_binding()
        self.assertFalse(self.panel.allow_edits.isChecked())
        self.panel.allow_edits.setChecked(True)
        self.panel.close()
        self.panel = Panel()
        self.assertTrue(self.panel.allow_edits.isChecked())

    def test_auto_connect_resolves_last_model_without_starting_inference(self):
        from loom import panel as panel_module
        from loom.checkpoints.storage import atomic_json, data_root
        atomic_json(data_root() / "preferences.json", {"model": "last-model"})
        self.panel.close()
        class MockProvider:
            def __init__(self):
                import queue
                self.events = queue.Queue()
                self.requests = []
                self.started = 0
            def start(self):
                self.started += 1
            def request(self, method, params):
                self.requests.append((method, params))
                response = {
                    "initialize": {},
                    "account/read": {"account": {"type": "chatgpt", "planType": "test"}},
                    "model/list": {"data": [
                        {"model": "default-model", "displayName": "Default", "isDefault": True},
                        {"model": "last-model", "displayName": "Last", "isDefault": False}]}}[method]
                request_id = len(self.requests)
                self.events.put({"id": request_id, "result": response})
                return request_id
            def notify(self, method, params=None):
                pass
            def close(self):
                pass
        fake = MockProvider()
        # This override only enables a mock: no subprocess or real provider exists.
        with patch.dict(os.environ, {"LOOM_BLOCK_PROVIDER": "0"}), \
             patch.object(panel_module.provider, "CodexTransport", return_value=fake):
            self.panel = Panel()
            app.processEvents()
            self.panel._poll()
            self.assertEqual(fake.started, 1)
            self.assertTrue(self.panel.ready)
            self.assertEqual(self.panel.model.currentData(), "last-model")
            self.assertEqual([method for method, _ in fake.requests], ["initialize", "account/read", "model/list"])
            self.assertEqual(self.panel.messages, [])
            self.panel._auto_connect()
            self.assertEqual(fake.started, 1)

    def test_closing_agent_before_deferred_connection_never_starts_provider(self):
        from loom import panel as panel_module
        self.panel.close()
        with patch.dict(os.environ, {"LOOM_BLOCK_PROVIDER": "0"}), \
             patch.object(panel_module.provider, "CodexTransport") as transport:
            self.panel = Panel()
            self.panel.close()
            app.processEvents()
            self.panel._auto_connect()
            transport.assert_not_called()

    def test_checkpoints_only_never_creates_a_provider_even_if_not_offline_blocked(self):
        from loom import panel as panel_module
        self.panel.close()
        with patch.dict(os.environ, {"LOOM_BLOCK_PROVIDER": "0"}), \
             patch.object(panel_module.provider, "CodexTransport") as transport:
            extra = CheckpointPanel()
            try:
                extra.show()
                app.processEvents()
                extra.refresh_versions()
                transport.assert_not_called()
            finally:
                extra.close()

    def test_edit_preferences_are_separate_per_workspace_and_survive_save_as(self):
        from loom.checkpoints.scene import WORKSPACE_KEY
        self.panel.allow_edits.setChecked(True)
        first_id = self.panel.workspace.history.id
        first = Path(self.temp.name) / "first.hip"
        hou.hipFile.save(str(first))
        renamed = Path(self.temp.name) / "renamed.hip"
        hou.hipFile.save(str(renamed))
        self.panel._poll()
        self.assertEqual(hou.node("/").userData(WORKSPACE_KEY), first_id)
        self.assertTrue(self.panel.allow_edits.isChecked())
        hou.hipFile.clear(suppress_save_prompt=True)
        self.panel._poll()
        second_id = self.panel.workspace.history.id
        self.assertNotEqual(first_id, second_id)
        self.assertFalse(self.panel.allow_edits.isChecked())
        self.panel.allow_edits.setChecked(True)
        self.panel.allow_edits.setChecked(False)
        hou.hipFile.load(str(renamed), suppress_save_prompt=True)
        self.panel._poll()
        self.assertTrue(self.panel.allow_edits.isChecked())
        self.assertEqual(self.panel.workspace.history.id, first_id)
        saved = __import__("json").loads((self.panel.workspace.history.root / "preferences.json").read_text())
        self.assertTrue(saved["allow_scene_edits"])

    def test_floating_geometry_preferences_are_independent_and_validated(self):
        from loom.window_state import geometry
        from loom.checkpoints.storage import atomic_json, data_root
        path = data_root() / "pane-geometry.json"
        atomic_json(path, {"agent": {"position": [-300, 120], "size": [910, 760]},
                           "checkpoints": {"position": [350, 210], "size": [780, 640]}})
        self.assertEqual(geometry("agent"), {"position": (-300, 120), "size": (910, 760)})
        self.assertEqual(geometry("checkpoints"), {"position": (350, 210), "size": (780, 640)})
        atomic_json(path, {"agent": {"position": [True, 0], "size": [0, -1]}})
        self.assertEqual(geometry("agent"), {})
        path.write_text("invalid JSON")
        self.assertEqual(geometry("agent"), {})

    def test_menu_toggles_do_not_close_docked_tabs(self):
        import loom
        from unittest.mock import Mock
        dock = Mock()
        dock.type.return_value = hou.paneTabType.PythonPanel
        dock.floatingPanel.return_value = None
        interface = Mock()
        dock.activeInterface.return_value = interface
        with patch.object(hou, "ui", Mock(paneTabs=Mock(return_value=[dock])), create=True):
            interface.name.return_value = "loom_workspace"
            with patch.object(loom, "open_pane", return_value=dock) as open_pane:
                self.assertIs(loom.toggle_pane(), dock)
                open_pane.assert_called_once()
            interface.name.return_value = "loom_checkpoints"
            with patch.object(loom.checkpoints, "open_pane", return_value=dock) as open_pane:
                self.assertIs(loom.checkpoints.toggle_pane(), dock)
                open_pane.assert_called_once()
        dock.close.assert_not_called()

    def test_optional_geometry_flush_cannot_block_controller_close(self):
        from loom.window_state import flush
        from unittest.mock import Mock
        root = Mock()
        root._loom_geometry_tracker.finish.side_effect = RuntimeError("deleted native window")
        flush(root)  # must not propagate into onDestroyInterface and skip shutdown
        root._loom_geometry_tracker.finish.assert_called_once()

    def test_chat_has_larger_text_no_role_headers_and_hover_icon_actions(self):
        self.panel._append("You", "A message")
        self.panel._append("Loom", "A reply")
        app.processEvents()
        card = self.panel.transcript.cards[-1]
        self.assertFalse(card.heading.isVisible())
        self.assertFalse(self.panel.transcript.cards[0].heading.isVisible())
        self.assertGreaterEqual(card.body.font().pointSizeF(), 11)
        self.assertGreaterEqual(self.panel.prompt.font().pointSizeF(), 11)
        self.assertTrue(card.actions.isHidden())
        event = QtGui.QEnterEvent(QtCore.QPointF(2, 2), QtCore.QPointF(2, 2), QtCore.QPointF(2, 2))
        app.sendEvent(card, event)
        self.assertTrue(card.actions.isVisible())
        self.assertEqual(card.copy_button.text(), "")
        self.assertFalse(card.copy_button.icon().isNull())
        app.sendEvent(card, QtCore.QEvent(QtCore.QEvent.Type.Leave))
        self.assertTrue(card.actions.isHidden())
        labels = [label.text() for label in self.panel.findChildren(QtWidgets.QLabel)]
        self.assertFalse(any("Enter to send" in label or "Scene edits allowed" in label for label in labels))

    def test_checkpoint_columns_are_interactive_and_ai_origin_is_a_small_icon(self):
        a = self.panel.workspace.checkpoint("Working")
        b = self.panel.workspace.checkpoint("Extrude faces", created_by="ai")
        self.checkpoints.refresh_versions(select_id=b["id"])
        header = self.checkpoints.versions.header()
        for column in range(3):
            self.assertEqual(header.sectionResizeMode(column), QtWidgets.QHeaderView.ResizeMode.Interactive)
        header.resizeSection(0, 950)
        self.checkpoints.refresh_versions(select_id=b["id"])
        self.assertEqual(header.sectionSize(0), 950)
        item = self.checkpoints.versions.selectedItems()[0]
        self.assertEqual(item.text(0), "Extrude faces")
        self.assertFalse(item.icon(0).isNull())
        self.assertIn("Created by Loom (AI)", item.toolTip(0))
        self.assertEqual(self.checkpoints.versions.columnCount(), 3)

    def test_delete_descendant_menu_protects_current_but_allows_current_parent(self):
        a = self.panel.workspace.checkpoint("Parent")
        b = self.panel.workspace.checkpoint("Child")
        self.checkpoints.refresh_versions(select_id=a["id"])
        menu = self.checkpoints._menu()
        action = next(action for action in menu.actions() if action.text() == "Delete descendant checkpoints")
        self.assertFalse(action.isEnabled())
        menu.deleteLater()
        self.panel.workspace.visit(a["id"])
        self.checkpoints.refresh_versions(select_id=a["id"])
        menu = self.checkpoints._menu()
        action = next(action for action in menu.actions() if action.text() == "Delete descendant checkpoints")
        self.assertTrue(action.isEnabled())
        action.trigger()
        self.assertEqual([record["id"] for record in self.panel.workspace.history.entries], [a["id"]])
        menu.deleteLater()

    def test_geometry_is_inspected_without_a_dialog_and_spinner_marks_work(self):
        box = self._cube()
        self._start_test_turn()
        self.assertTrue(self.panel.spinner.timer.isActive())
        with patch.object(QtWidgets.QMessageBox, "question") as question:
            self.panel._event({"id": 81, "method": "item/tool/call", "params": {
                "tool": "loom_scene", "arguments": {"operation": "geometry", "path": box.path()}}})
            question.assert_not_called()
        self.assertTrue(self.panel.transport.replies[-1][1]["success"])
        self.panel._set_busy(False)
        self.assertFalse(self.panel.spinner.timer.isActive())

    def test_stop_is_processed_during_cooperative_inspection(self):
        self._start_test_turn()
        def inspect(*args):
            QtCore.QTimer.singleShot(0, self.panel.interrupt)
            self.panel.tools.progress()
            return {}
        with patch.object(self.panel.tools, "dispatch", side_effect=inspect):
            fake = self.panel.transport
            self.panel._event({"id": 82, "method": "item/tool/call", "params": {
                "tool": "loom_scene", "arguments": {"operation": "geometry", "path": "/"}}})
        self.assertFalse(fake.replies[-1][1]["success"])
        self.assertIn("cancelled", fake.replies[-1][1]["contentItems"][0]["text"])
        self.assertFalse(self.panel.busy)
        self.assertFalse(self.panel._handling_tool)

    def test_viewport_image_is_forwarded_not_written_into_chat_as_base64(self):
        self._start_test_turn()
        image = "data:image/png;base64,TEST"
        with patch.object(self.panel.tools, "dispatch", return_value={"width": 960, "_image_url": image}):
            self.panel._event({"id": 83, "method": "item/tool/call", "params": {
                "tool": "loom_scene", "arguments": {"operation": "viewport"}}})
        content = self.panel.transport.replies[-1][1]["contentItems"]
        self.assertEqual(content[1], {"type": "inputImage", "imageUrl": image})
        self.assertNotIn("base64", self.panel.messages[-1]["text"])
        self.assertEqual(self.panel.messages[-1]["summary"], "Captured viewport")

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
        self.checkpoints.make_checkpoint()
        self.assertEqual(len(self.panel.workspace.history.entries), 1)
        self.assertRegex(self.panel.workspace.history.entries[0]["label"], r"^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}$")
        self.checkpoints.versions.topLevelItem(0).setSelected(True)
        answer("Renamed", QtCore.Qt.Key.Key_Enter)
        self.checkpoints.activateWindow()
        self.checkpoints.versions.setFocus()
        app.processEvents()
        QtTest.QTest.keyClick(self.checkpoints.versions, QtCore.Qt.Key.Key_F2)
        self.assertEqual(self.panel.workspace.history.entries[0]["label"], "Renamed")
        answer("Do not rename", QtCore.Qt.Key.Key_Escape)
        self.checkpoints.rename_checkpoint()
        self.assertEqual(len(self.panel.workspace.history.entries), 1)
        self.assertEqual(self.panel.workspace.history.entries[0]["label"], "Renamed")

    def test_tree_diff_and_chat(self):
        geo = hou.node("/obj").createNode("geo", "candidate", run_init_scripts=False)
        box = geo.createNode("box", "box1")
        a = self.panel.workspace.checkpoint("Original")
        b = self.panel.workspace.checkpoint("Wider candidate")
        box.parm("sizex").set(3)
        self.panel.workspace.save_current()
        self.checkpoints.refresh_versions()
        self.assertEqual(self.checkpoints.versions.topLevelItemCount(), 1)
        self.assertEqual(self.checkpoints.versions.topLevelItem(0).childCount(), 1)
        dialog = CompareDialog(self.panel.workspace, a["id"], b["id"], self.panel)
        self.assertIn('/obj/candidate/box1', dialog.findChildren(QtWidgets.QPlainTextEdit)[0].toPlainText())
        dialog.close()
        self.panel._append("You", "Explain this box and propose a wider version.")
        self.panel._event({"method": "item/agentMessage/delta", "params": {"itemId": "x", "delta": "A wider "}})
        self.panel._event({"method": "item/agentMessage/delta", "params": {"itemId": "x", "delta": "box."}})
        self.panel._event({"method": "item/completed", "params": {"item": {
            "type": "agentMessage", "id": "x", "text": "A wider box."}}})
        self.assertEqual(self.panel.messages[-1]["text"], "A wider box.")
        self.checkpoints.setFocus()
        app.processEvents()
        screenshot = Path(tempfile.gettempdir()) / "loom-ui-smoke.png"
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
                host.grab().save(str(Path(tempfile.gettempdir()) / "loom-duplicate-smoke.png"))
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
        import loom
        import xml.etree.ElementTree as ET
        source = ET.parse(Path(__file__).resolve().parents[2] / "python_panels/loom.pypanel").find("interface/script").text
        namespace = {}
        exec(compile(source, "loom.pypanel", "exec"), namespace)
        with patch.object(loom, "reload_modules") as reload_modules:
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
        import loom
        from unittest.mock import Mock
        interface = Mock()
        interface.name.return_value = "loom_workspace"
        owner = Mock()
        owner.type.return_value = hou.paneTabType.PythonPanel
        owner.activeInterface.return_value = interface
        owner.activeInterfaceRootWidget.return_value = self.panel
        notice_widget = Mock(_content=None, _closed=False, busy=False)
        notice_widget.objectName.return_value = "loom_host"
        notice = Mock()
        notice.type.return_value = hou.paneTabType.PythonPanel
        notice.activeInterface.return_value = interface
        notice.activeInterfaceRootWidget.return_value = notice_widget
        with patch.object(hou, "isUIAvailable", return_value=True), \
             patch.object(hou, "ui", Mock(paneTabs=Mock(return_value=[notice, owner])), create=True), \
             patch.object(hou, "pypanel", Mock(interfaceByName=Mock(return_value=interface)), create=True) as pypanel:
            loom._reload_agent_pane()
            pypanel.installFile.assert_not_called()
            interface.setScript.assert_called_once()
        owner.reloadActiveInterface.assert_called_once()
        notice.reloadActiveInterface.assert_not_called()

    def test_reload_repairs_pane_with_missing_interface_binding(self):
        import loom
        from unittest.mock import Mock
        pane = Mock()
        pane.type.return_value = hou.paneTabType.PythonPanel
        pane.activeInterface.return_value = None
        pane.activeInterfaceRootWidget.return_value = self.panel
        definition = Mock()
        definition.name.return_value = "loom_workspace"
        with patch.object(hou, "isUIAvailable", return_value=True), \
             patch.object(hou, "ui", Mock(paneTabs=Mock(return_value=[pane])), create=True), \
             patch.object(hou, "pypanel", Mock(interfaceByName=Mock(return_value=definition)), create=True) as pypanel:
            loom._reload_agent_pane()
            pypanel.installFile.assert_not_called()
        pane.setActiveInterface.assert_called_once_with(definition)
        pane.reloadActiveInterface.assert_not_called()

    def test_reload_recovers_single_unbound_panel_from_older_tool(self):
        import loom
        from unittest.mock import Mock
        pane = Mock()
        pane.name.return_value = "legacy_unbound_loom"
        pane.label.return_value = "Python Panel"
        pane.type.return_value = hou.paneTabType.PythonPanel
        pane.activeInterface.return_value = None
        pane.activeInterfaceRootWidget.return_value = None
        definition = Mock()
        definition.name.return_value = "loom_workspace"
        with patch.object(hou, "isUIAvailable", return_value=True), \
             patch.object(hou, "ui", Mock(paneTabs=Mock(return_value=[pane])), create=True), \
             patch.object(hou, "pypanel", Mock(interfaceByName=Mock(return_value=definition)), create=True):
            loom._reload_agent_pane()
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

    def test_cleanup_is_direct_saves_current_and_needs_no_selection(self):
        geo = hou.node("/obj").createNode("geo", "candidate", run_init_scripts=False)
        box = geo.createNode("box", "box1")
        original = self.panel.workspace.checkpoint("Original")
        current = self.panel.workspace.checkpoint("Alternative")
        box.parm("sizex").set(7)
        self.checkpoints.refresh_versions()
        self.checkpoints.versions.clearSelection()
        with patch.object(QtWidgets.QMessageBox, "question") as question:
            self.checkpoints.clean_up()
            question.assert_not_called()
        history = self.panel.workspace.history
        self.assertEqual([r["id"] for r in history.entries], [current["id"]])
        self.assertEqual(history.get(current["id"])["label"], "Alternative")
        self.assertEqual(box.parm("sizex").eval(), 7)
        self.assertFalse(history.directory(original["id"]).exists())
        self.assertTrue(history.verify(current["id"]).exists())


    def test_cleanup_unchanged_current_keeps_same_id(self):
        self.panel.workspace.checkpoint("Original")
        current = self.panel.workspace.checkpoint("Wide")
        self.checkpoints.clean_up()
        self.assertEqual(self.panel.workspace.active, current["id"])
        self.assertEqual(len(self.panel.workspace.history.entries), 1)


    def test_cleanup_save_failure_deletes_nothing(self):
        from loom.checkpoints.scene import Workspace
        original = self.panel.workspace.checkpoint("Original")
        current = self.panel.workspace.checkpoint("Wide")
        with patch.object(Workspace, "save_current", side_effect=OSError("disk full")):
            self.checkpoints._guard(self.checkpoints.clean_up)
        self.assertIn("disk full", self.checkpoints.status.text())
        self.assertEqual(len(self.panel.workspace.history.entries), 2)
        self.assertTrue(self.panel.workspace.history.verify(original["id"]).exists())
        self.assertTrue(self.panel.workspace.history.verify(current["id"]).exists())


    def test_tool_request_runs_on_ui_and_read_only_is_enforced(self):
        fake = FakeTransport()
        self.panel.transport = fake
        self.panel._turn_workspace = self.panel.workspace.history.id
        self.panel._set_busy(True)
        self.panel._event({"id": 91, "method": "item/tool/call", "params": {
            "tool": "loom_scene", "arguments": {"operation": "context", "path": "/", "query": "", "offset": 0, "limit": 1}}})
        self.assertTrue(fake.replies[-1][1]["success"])
        self.panel._event({"id": 92, "method": "item/tool/call", "params": {
            "tool": "loom_propose", "arguments": {"label": "test", "steps_json": "[]"}}})
        self.assertFalse(fake.replies[-1][1]["success"])
        self.panel._event({"id": 93, "method": "item/commandExecution/requestApproval", "params": {}})
        self.assertEqual(fake.replies[-1][1], {"decision": "decline"})
        self.panel._set_busy(False)

    def test_replace_selected_updates_same_id_without_hidden_revision(self):
        geo = hou.node("/obj").createNode("geo", "candidate", run_init_scripts=False)
        box = geo.createNode("box", "box1")
        original = self.panel.workspace.checkpoint("Original")
        self.checkpoints.refresh_versions()
        self.checkpoints.versions.topLevelItem(0).setSelected(True)
        box.parm("sizex").set(7)
        with patch.object(QtWidgets.QMessageBox, "question", return_value=QtWidgets.QMessageBox.StandardButton.Yes):
            self.checkpoints.replace_checkpoint()
        self.assertEqual(self.checkpoints.versions.topLevelItemCount(), 1)
        self.assertEqual(len(self.panel.workspace.history.entries), 1)
        self.assertEqual(self.panel.workspace.active, original["id"])
        self.assertEqual(self.checkpoints.selected_ids(), [original["id"]])
        self.assertIn("Original", self.checkpoints.versions.topLevelItem(0).text(0))
        self.assertNotIn("replaced_by", self.panel.workspace.history.get(original["id"]))


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

    def test_two_panes_share_controller_and_survive_either_close_order(self):
        self.assertIs(self.panel.workspace, self.checkpoints.workspace)
        self.checkpoints.make_checkpoint()
        key = self.checkpoints.workspace.active
        self.panel.close()
        self.checkpoints.make_checkpoint()
        self.assertEqual(len(self.checkpoints.workspace.history.entries), 2)
        self.panel = Panel()
        self.assertIs(self.panel.workspace, self.checkpoints.workspace)
        self.checkpoints.close()
        self.panel.workspace.history.verify(key)
        self.checkpoints = CheckpointPanel()
        self.assertIs(self.panel.workspace, self.checkpoints.workspace)

    def test_checkpoint_menu_extensions_and_selection_specific_compare(self):
        from loom.checkpoints.actions import register_action, unregister_action
        self.checkpoints.make_checkpoint()
        self.checkpoints.versions.topLevelItem(0).setSelected(True)
        calls = []
        register_action("test.naming", "Suggest name", lambda *args: calls.append(args))
        try:
            menu = self.checkpoints._menu()
            labels = [action.text() for action in menu.actions()]
            self.assertIn("Compare with current scene", labels)
            self.assertNotIn("Compare checkpoints", labels)
            self.assertNotIn("Pin / unpin", labels)
            next(a for a in menu.actions() if a.text() == "Suggest name").trigger()
            self.assertEqual(calls[0][1], [self.panel.workspace.active])
            menu.deleteLater()
            self.checkpoints.make_checkpoint()
            self.checkpoints.versions.topLevelItem(0).setSelected(True)
            menu = self.checkpoints._menu()
            self.assertIn("Compare checkpoints", [a.text() for a in menu.actions()])
            self.assertNotIn("Suggest name", [a.text() for a in menu.actions()])
            menu.deleteLater()
        finally:
            unregister_action("test.naming")

    def test_checkpoint_changes_blocked_during_agent_turn(self):
        self.checkpoints.make_checkpoint()
        key = self.panel.workspace.active
        self.panel._set_busy(True)
        self.checkpoints._guard(self.checkpoints.make_checkpoint)
        self.assertIn("Stop the active Loom turn", self.checkpoints.status.text())
        self.assertEqual(len(self.panel.workspace.history.entries), 1)
        self.assertEqual(self.panel.workspace.active, key)
        self.checkpoints._refresh_if_changed()
        self.assertFalse(self.checkpoints.checkpoint_button.isEnabled())
        self.assertFalse(self.checkpoints.capture_button.isEnabled())
        self.panel._set_busy(False)


    def test_new_checkpoint_is_sole_selection_and_keyboard_current_item(self):
        self.checkpoints.make_checkpoint()
        first = self.panel.workspace.active
        self.assertEqual(self.checkpoints.selected_ids(), [first])
        self.checkpoints.make_checkpoint()
        second = self.panel.workspace.active
        self.assertNotEqual(first, second)
        self.assertEqual(self.checkpoints.selected_ids(), [second])
        self.assertEqual(self.checkpoints.versions.currentItem().data(0, QtCore.Qt.ItemDataRole.UserRole), second)
        self.assertFalse(any(w.text() == "Reload tool" for w in self.panel.findChildren(QtWidgets.QPushButton)))

    def test_capture_is_top_bar_action_and_always_targets_current(self):
        from loom.checkpoints.panel import CURRENT_ROLE
        self.assertFalse(self.checkpoints.capture_button.isEnabled())
        self.checkpoints.make_checkpoint()
        original = self.panel.workspace.active
        self.checkpoints.make_checkpoint()
        current = self.panel.workspace.active
        self.checkpoints.versions.clearSelection()
        first = self.checkpoints.versions.topLevelItem(0)
        first.setSelected(True)  # Not the current checkpoint.
        self.assertFalse(first.data(0, CURRENT_ROLE))
        self.assertTrue(first.child(0).data(0, CURRENT_ROLE))
        self.assertEqual(first.child(0).text(0), self.panel.workspace.history.get(current)["label"])
        self.assertEqual(self.checkpoints.versions.headerItem().text(1), "Last updated")
        menu = self.checkpoints._menu()
        labels = [a.text() for a in menu.actions()]
        self.assertNotIn("Capture preview…", labels)
        self.assertTrue(next(a for a in menu.actions() if a.text() == "Delete checkpoint").isEnabled())
        menu.deleteLater()
        def capture(directory):
            self.assertEqual(directory, self.panel.workspace.history.directory(current))
            (directory / "preview-test.png").write_bytes(b"fake preview")
            return "preview-test.png"
        with patch("loom.checkpoints.scene.capture_preview", side_effect=capture):
            self.checkpoints.capture_button.click()
        self.assertEqual(self.panel.workspace.history.get(current)["preview"], "preview-test.png")
        self.assertIsNone(self.panel.workspace.history.get(original)["preview"])

    def test_delete_current_disabled_and_parent_deletion_keeps_children(self):
        self.checkpoints.make_checkpoint()
        parent = self.panel.workspace.active
        self.checkpoints.make_checkpoint()
        current = self.panel.workspace.active
        self.checkpoints.versions.topLevelItem(0).child(0).setSelected(True)
        menu = self.checkpoints._menu()
        self.assertFalse(next(a for a in menu.actions() if a.text() == "Delete checkpoint").isEnabled())
        menu.deleteLater()
        self.checkpoints.versions.clearSelection()
        self.checkpoints.versions.topLevelItem(0).setSelected(True)
        self.checkpoints.delete_selected()
        self.assertEqual(self.panel.workspace.active, current)
        self.assertIsNone(self.panel.workspace.history.get(current)["parent"])
        self.assertFalse(self.panel.workspace.history.directory(parent).exists())
        self.assertEqual(self.checkpoints.versions.topLevelItemCount(), 1)
        self.assertFalse(any(w.text() == "Return" for w in self.checkpoints.findChildren(QtWidgets.QPushButton)))

    def test_instruction_files_follow_workspace_and_edits_refresh_next_turn(self):
        from loom.instructions import instruction_paths, load_instructions
        shared, local = instruction_paths(self.panel.workspace)
        self.assertEqual(shared.name, "LOOM.md")
        local.write_text("Use meters.\n", encoding="utf-8")
        self.assertIn("Use meters.", load_instructions(self.panel.workspace))
        old = self.panel.workspace.history.id
        hou.hipFile.setName(str(Path(self.temp.name) / "renamed.hip"))
        self.panel._ensure_binding()
        self.assertEqual(self.panel.workspace.history.id, old)
        self.assertEqual(instruction_paths(self.panel.workspace)[1], local)
        self.panel.ready = True
        self.panel.thread_id = "existing-thread"
        self.panel._resumed = True
        requests = []
        self.panel._request = lambda method, params, callback: requests.append((method, params))
        self.panel.prompt.setPlainText("Explain")
        self.panel.send_prompt()
        self.assertEqual(requests[-1][0], "thread/resume")
        self.assertIn("Use meters.", requests[-1][1]["developerInstructions"])
        self.panel._set_busy(False)
        self.panel._resumed = True
        self.panel.prompt.setPlainText("Continue")
        self.panel.send_prompt()
        self.assertEqual(requests[-1][0], "turn/start")
        self.panel._set_busy(False)
        local.write_text("Use centimeters.\n", encoding="utf-8")
        self.panel.prompt.setPlainText("Continue")
        self.panel.send_prompt()
        self.assertEqual(requests[-1][0], "thread/resume")
        self.assertIn("Use centimeters.", requests[-1][1]["developerInstructions"])
        self.panel._set_busy(False)

    def test_old_edit_records_survive_reload_without_review_ui(self):
        proposal = self.panel.tools.propose("Create a geo", [
            {"op": "create", "parent": "/obj", "type": "geo", "name": "test"}])
        self.panel.close()
        self.panel = Panel()
        self.assertIn(proposal["proposal"], self.panel.tools.proposals)
        self.assertFalse(hasattr(self.panel, "proposal_list"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
