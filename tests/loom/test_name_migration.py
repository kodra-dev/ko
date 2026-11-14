"""OFFLINE migration guards: temporary bytes/paths only, never real credentials."""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python3.13libs"))
from loom.paths import data_root, migrate_data_home
from loom import provider


class MigrationTests(unittest.TestCase):
    def test_whole_home_moves_once_without_parsing_or_changing_contents(self):
        with tempfile.TemporaryDirectory() as folder:
            base = Path(folder)
            legacy = base / "ko-ai"
            files = {"codex/auth.json": b"opaque-test-bytes-not-a-real-token",
                     "codex/state.sqlite": b"opaque-database-test-bytes",
                     "workspaces/abc/chat.json": b'{"thread_id":"test-thread"}',
                     "preferences.json": b'{"model":"test-model"}', "LOOM.md": b"custom guidance"}
            for name, content in files.items():
                path = legacy / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(content)
            root = migrate_data_home(base)
            self.assertEqual(root, base / "Loom")
            self.assertFalse(legacy.exists())
            self.assertEqual(migrate_data_home(base), root)
            for name, content in files.items():
                self.assertEqual((root / name).read_bytes(), content)

    def test_conflicting_homes_are_not_silently_merged(self):
        with tempfile.TemporaryDirectory() as folder:
            base = Path(folder)
            for name in ("Loom", "ko-ai"):
                (base / name).mkdir()
                (base / name / "keep").write_text(name)
            with self.assertRaisesRegex(RuntimeError, "Nothing was merged"):
                migrate_data_home(base)
            self.assertEqual((base / "ko-ai/keep").read_text(), "ko-ai")
            self.assertEqual((base / "Loom/keep").read_text(), "Loom")

    def test_failed_rename_does_not_copy_delete_or_create_a_new_empty_history(self):
        with tempfile.TemporaryDirectory() as folder:
            legacy = Path(folder) / "ko-ai"
            legacy.mkdir()
            (legacy / "keep").write_text("untouched")
            with patch.object(Path, "rename", side_effect=PermissionError("in use")):
                with self.assertRaisesRegex(RuntimeError, "Close all old"):
                    migrate_data_home(folder)
            self.assertTrue((legacy / "keep").exists())
            self.assertFalse((Path(folder) / "Loom").exists())

    def test_active_history_owner_blocks_default_home_migration(self):
        from loom.checkpoints.storage import History
        with tempfile.TemporaryDirectory() as folder:
            legacy = Path(folder) / "ko-ai"
            history = History("abc123", root=legacy)
            try:
                with self.assertRaisesRegex(RuntimeError, "workspace is in use"):
                    migrate_data_home(folder)
                self.assertTrue(legacy.exists())
                self.assertFalse((Path(folder) / "Loom").exists())
            finally:
                history.close()
            self.assertEqual(migrate_data_home(folder), Path(folder) / "Loom")

    def test_canonical_override_wins_and_custom_legacy_override_remains_readable(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch.dict(os.environ, {"LOOM_HOME": folder, "KO_AI_HOME": folder + "-old"}):
                self.assertEqual(data_root(), Path(folder).resolve())
            with patch.dict(os.environ, {"LOOM_HOME": "", "KO_AI_HOME": folder}):
                self.assertEqual(data_root(), Path(folder).resolve())

    def test_legacy_override_pointing_at_default_path_is_migrated_too(self):
        with tempfile.TemporaryDirectory() as folder:
            legacy = Path(folder) / "ko-ai"
            legacy.mkdir()
            (legacy / "keep").write_text("test")
            with patch.dict(os.environ, {"LOOM_HOME": "", "KO_AI_HOME": str(legacy), "LOCALAPPDATA": folder}):
                self.assertEqual(data_root(), Path(folder) / "Loom")
            self.assertFalse(legacy.exists())
            self.assertEqual((Path(folder) / "Loom/keep").read_text(), "test")

    def test_later_loading_a_legacy_hip_translates_metadata_before_native_save(self):
        import hou
        from loom.checkpoints.scene import Workspace
        hou.hipFile.clear(suppress_save_prompt=True)
        with tempfile.TemporaryDirectory() as folder:
            legacy = Path(folder) / "legacy.hip"
            root = hou.node("/")
            root.setUserData("ko_ai.workspace", "abc123")
            hou.hipFile.save(str(legacy))
            hou.hipFile.clear(suppress_save_prompt=True)
            workspace = Workspace(root=folder)
            try:
                original = workspace.history.id
                hou.hipFile.load(str(legacy), suppress_save_prompt=True)
                self.assertEqual(hou.node("/").userData("loom.workspace"), "abc123")
                self.assertIsNone(hou.node("/").userData("ko_ai.workspace"))
                self.assertEqual(workspace.history.id, original)  # no mid-turn rebind
            finally:
                workspace.close()
                hou.hipFile.clear(suppress_save_prompt=True)

    def test_relocated_native_rollout_is_supplied_to_resume_without_reading_it(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            rollout = root / "codex/sessions/2026/01/rollout-test-thread.jsonl"
            rollout.parent.mkdir(parents=True)
            rollout.write_bytes(b"not parsed by Loom")
            with patch.object(provider, "data_root", return_value=root):
                params = provider.resume_params("test-thread")
            self.assertEqual(params["path"], str(rollout))
            self.assertEqual(params["threadId"], "test-thread")

    def test_offline_guard_blocks_process_creation(self):
        with patch.dict(os.environ, {"LOOM_BLOCK_PROVIDER": "1"}), patch.object(provider.subprocess, "Popen") as spawn:
            with self.assertRaisesRegex(RuntimeError, "offline test runner"):
                provider.CodexTransport().start()
            spawn.assert_not_called()

    def test_legacy_hip_identity_translates_without_a_new_uuid(self):
        import hou
        from loom.checkpoints.scene import migrate_identity
        hou.hipFile.clear(suppress_save_prompt=True)
        root = hou.node("/")
        try:
            root.setUserData("ko_ai.workspace", "abc123")
            root.setUserData("ko_ai.checkpoint", "def456")
            migrate_identity()
            self.assertEqual(root.userData("loom.workspace"), "abc123")
            self.assertEqual(root.userData("loom.checkpoint"), "def456")
            self.assertIsNone(root.userData("ko_ai.workspace"))
            migrate_identity()
            self.assertEqual(root.userData("loom.workspace"), "abc123")
        finally:
            hou.hipFile.clear(suppress_save_prompt=True)

    def test_new_identity_takes_precedence_over_stale_legacy_metadata(self):
        import hou
        from loom.checkpoints.scene import migrate_identity
        hou.hipFile.clear(suppress_save_prompt=True)
        root = hou.node("/")
        try:
            root.setUserData("loom.workspace", "abc123")
            root.setUserData("ko_ai.workspace", "outdated")
            migrate_identity()
            self.assertEqual(root.userData("loom.workspace"), "abc123")
            self.assertIsNone(root.userData("ko_ai.workspace"))
        finally:
            hou.hipFile.clear(suppress_save_prompt=True)

    def test_legacy_scene_tool_alias_is_still_readable_without_advertising_it(self):
        from loom.tools import SceneTools, tool_specs
        tools = object.__new__(SceneTools)
        result = tools.dispatch("ko_scene", {"operation": "context"})
        self.assertIn("hip", result)
        self.assertEqual([spec["name"] for spec in tool_specs()], ["loom_scene", "loom_propose"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
