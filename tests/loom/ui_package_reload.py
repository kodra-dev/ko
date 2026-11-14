"""Offscreen package-wide reload tests in their own disposable process."""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python3.13libs"))
import hou
from PySide6 import QtWidgets
import loom
from loom import checkpoints

app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class ReloadTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="loom-reload-test-")
        self.old_home = os.environ.get("LOOM_HOME")
        os.environ["LOOM_HOME"] = self.temp.name
        hou.hipFile.clear(suppress_save_prompt=True)
        hou.hipFile.setName(str(Path(self.temp.name) / "working.hip"))

    def tearDown(self):
        import loom.panel
        loom.panel.close_panel()
        checkpoints.close()
        app.processEvents()
        hou.hipFile.clear(suppress_save_prompt=True)
        if self.old_home is None:
            os.environ.pop("LOOM_HOME", None)
        else:
            os.environ["LOOM_HOME"] = self.old_home
        self.temp.cleanup()

    def test_both_features_reload_with_live_edits_and_selection_preserved(self):
        agent_window = loom.show()
        checkpoint_window = checkpoints.show()
        agent = agent_window._loom_host._content
        panel = checkpoint_window._loom_checkpoint_panel
        geo = hou.node("/obj").createNode("geo", "reload_test", run_init_scripts=False)
        box = geo.createNode("box")
        panel.make_checkpoint()
        panel.make_checkpoint()
        selected = panel.selected_ids()
        ids = [r["id"] for r in panel.workspace.history.entries]
        workspace_id = panel.workspace.history.id
        active = panel.workspace.active
        box.parm("sizex").set(7)  # Reload must neither save nor discard these edits.
        agent.new_chat("Reload selection")
        conversation_id = agent.conversation_id
        agent._append("You", "PACKAGE_RELOAD_MARKER")
        agent.prompt.setPlainText("PACKAGE_DRAFT_MARKER")
        from loom.checkpoints.actions import register_action
        callback = lambda *_: None
        register_action("test.reload", "Test extension", callback)
        old_types = (type(agent), type(panel), type(panel.workspace))
        path = hou.hipFile.path()
        result = loom.reload()
        self.assertEqual(result["windows"], 2)
        agent = loom.show()._loom_host._content
        panel = checkpoints.show()._loom_checkpoint_panel
        self.assertNotEqual(old_types, (type(agent), type(panel), type(panel.workspace)))
        self.assertIs(agent.workspace, panel.workspace)
        self.assertEqual(panel.workspace.history.id, workspace_id)
        self.assertEqual(panel.workspace.active, active)
        self.assertEqual([r["id"] for r in panel.workspace.history.entries], ids)
        self.assertEqual(panel.selected_ids(), selected)
        self.assertEqual(hou.hipFile.path(), path)
        self.assertEqual(box.parm("sizex").eval(), 7)
        self.assertTrue(panel.workspace.has_live_changes())
        self.assertEqual(agent.messages[-1]["text"], "PACKAGE_RELOAD_MARKER")
        self.assertEqual(agent.conversation_id, conversation_id)
        self.assertEqual(agent.prompt.toPlainText(), "PACKAGE_DRAFT_MARKER")
        self.assertEqual(len(hou.hipFile.eventCallbacks()), 1)
        from loom.checkpoints.actions import registered_actions, unregister_action
        self.assertIs(next(a for a in registered_actions() if a.id == "test.reload").callback, callback)
        unregister_action("test.reload")

    def test_busy_turn_refuses_reload_without_closing_either_view(self):
        agent = loom.show()._loom_host._content
        panel = checkpoints.show()._loom_checkpoint_panel
        agent._set_busy(True)
        try:
            with self.assertRaisesRegex(RuntimeError, "Stop the active Loom turn"):
                loom.reload()
            self.assertFalse(agent._closed)
            self.assertFalse(panel._closed)
            self.assertIs(loom.show()._loom_host._content, agent)
        finally:
            agent._set_busy(False)

    def test_syntax_preflight_failure_keeps_working_panes_alive(self):
        agent = loom.show()._loom_host._content
        panel = checkpoints.show()._loom_checkpoint_panel
        # Patch the source-read preflight, not importlib itself.
        with patch.object(Path, "read_text", return_value="def broken(:"):
            with self.assertRaises(SyntaxError):
                loom.reload()
        self.assertFalse(agent._closed)
        self.assertFalse(panel._closed)
        self.assertIs(agent.workspace, panel.workspace)

    def test_checkpoints_only_reload_does_not_open_agent(self):
        panel = checkpoints.show()._loom_checkpoint_panel
        panel.make_checkpoint()
        active = panel.workspace.active
        old_type = type(panel)
        loom.reload()
        panel = checkpoints.show()._loom_checkpoint_panel
        self.assertIsNot(type(panel), old_type)
        self.assertEqual(panel.workspace.active, active)
        self.assertIsNone(getattr(sys.modules.get("loom.panel"), "_window", None))
        self.assertEqual(len(panel.session.clients), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
