"""Independent extension registration and agent-free imports."""
from pathlib import Path
import os
import subprocess
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python3.13libs"))
from loom.checkpoints import actions


class ExtensionTests(unittest.TestCase):
    def tearDown(self):
        actions.unregister_action("test.naming")

    def test_registration_is_idempotent_and_needs_no_pane(self):
        first, second = lambda *_: None, lambda *_: None
        actions.register_action("test.naming", "Suggest name", first)
        actions.register_action("test.naming", "Suggest name", second)
        matches = [a for a in actions.registered_actions() if a.id == "test.naming"]
        self.assertEqual(len(matches), 1)
        self.assertIs(matches[0].callback, second)
        actions.unregister_action("test.naming")
        self.assertFalse(any(a.id == "test.naming" for a in actions.registered_actions()))

    def test_checkpoints_only_package_reload_does_not_import_agent(self):
        script = """
import os, sys, tempfile
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
from PySide6 import QtWidgets
import loom
from loom import checkpoints
app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
with tempfile.TemporaryDirectory(prefix='loom-independent-reload-') as home:
    os.environ['LOOM_HOME'] = home
    window = checkpoints.show()
    try:
        window._loom_checkpoint_panel.make_checkpoint()
        loom.reload()
        assert 'loom.panel' not in sys.modules
        assert 'loom.provider' not in sys.modules
        assert 'loom.tools' not in sys.modules
        assert checkpoints.show()._loom_checkpoint_panel.workspace.active
    finally:
        checkpoints.close()
        app.processEvents()
"""
        env = dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[2] / "python3.13libs"))
        result = subprocess.run([sys.executable, "-c", script], env=env, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_checkpoint_import_does_not_load_agent(self):
        script = """
import sys
import loom.checkpoints.panel
assert 'loom.panel' not in sys.modules
assert 'loom.provider' not in sys.modules
assert 'loom.tools' not in sys.modules
"""
        env = dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[2] / "python3.13libs"))
        result = subprocess.run([sys.executable, "-c", script], env=env, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
