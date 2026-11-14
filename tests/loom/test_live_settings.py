"""Read-only settings retrieval and tool-host launch regression guards."""
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python3.13libs"))
import hou
from loom import local, provider


class SettingsTests(unittest.TestCase):
    def setUp(self):
        self.index = 1
        self.keys = ["Alt+E"]
        self.symbol = "h.pane.gview.tool:sop_polyextrude::2.0"
        self.manager = SimpleNamespace(
            currentKeymap=lambda: "Local artist keymap",
            changeIndex=lambda: self.index,
            contextsInContext=lambda context: [{"symbol": "h.pane.gview"}] if context == "h" else [],
            commandBindingsInContext=lambda context: [{"symbol": self.symbol, "label": "", "help": ""}]
                if context == "h.pane.gview" else [],
            assignments=lambda context, symbol: tuple(self.keys))
        local._cache = None

    def tearDown(self):
        local._cache = None

    def test_searches_live_tool_bindings_not_just_registered_state_commands(self):
        with patch.object(hou, "hotkeys", self.manager, create=True):
            result = local.hotkeys("Poly Extrude")
            self.assertEqual(result["keymap"], "Local artist keymap")
            self.assertEqual(result["matches"][0]["keys"], ["Alt+E"])
            self.assertEqual(result["matches"][0]["context"], "h.pane.gview")
            self.assertEqual(local.hotkeys("Alt+E")["total"], 1)

    def test_cached_assignments_are_invalidated_when_live_keymap_changes(self):
        with patch.object(hou, "hotkeys", self.manager, create=True):
            self.assertEqual(local.hotkeys("PolyExtrude")["matches"][0]["keys"], ["Alt+E"])
            self.keys = ["Ctrl+E"]
            self.index += 1
            self.assertEqual(local.hotkeys("PolyExtrude")["matches"][0]["keys"], ["Ctrl+E"])

    def test_missing_match_does_not_claim_no_shortcut_exists(self):
        with patch.object(hou, "hotkeys", self.manager, create=True):
            result = local.hotkeys("Nonexistent action")
            self.assertEqual(result["total"], 0)
            self.assertIn("not proof", result["note"])

    def test_context_query_uses_existing_tool_shape_for_resumed_conversations(self):
        from loom.tools import SceneTools, tool_specs
        tools = object.__new__(SceneTools)
        with patch.object(hou, "hotkeys", self.manager, create=True):
            result = tools.inspect("context", "/", "PolyExtrude", 0, 10)
            self.assertEqual(result["hotkeys"]["matches"][0]["keys"], ["Alt+E"])
            self.assertIn("fps", result["local_settings"])
        spec = next(tool for tool in tool_specs() if tool["name"] == "loom_scene")
        self.assertIn("context", spec["inputSchema"]["properties"]["operation"]["enum"])
        self.assertIn("query", spec["inputSchema"]["properties"])

    def test_tool_host_enabled_without_enabling_shell_or_patch_tools(self):
        with tempfile.TemporaryDirectory(prefix="loom-launch-config-") as home:
            with patch.object(provider, "data_root", return_value=Path(home)), \
                 patch.object(provider, "find_codex", return_value="codex.exe"):
                command, env, cwd = provider.launch_config()
            self.assertIn("features.code_mode_host=true", command)
            self.assertNotIn("features.code_mode_host=false", command)
            self.assertIn("features.shell_tool=false", command)
            self.assertIn("features.unified_exec=false", command)
            self.assertIn("features.apply_patch_tool=false", command)
            self.assertEqual(env["CODEX_HOME"], str(Path(home) / "codex"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
