"""Run only in a separate disposable hython process; never in an artist's session."""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import hou

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python3.13libs"))
from ko_ai.scene import Workspace, CHECKPOINT_KEY, WORKSPACE_KEY, graph_snapshot, ui_context
from ko_ai.tools import SceneTools
from ko_ai.storage import graph_diff


class HoudiniTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="ko-ai-hom-test-")
        hou.hipFile.clear(suppress_save_prompt=True)
        hou.hipFile.setName(str(Path(self.temp.name) / "working.hip"))
        self.workspace = Workspace(root=self.temp.name)
        self.tools = SceneTools(self.workspace, allow_proposals=True, approve_cook=lambda _: True)

    def tearDown(self):
        self.workspace.close()
        hou.hipFile.clear(suppress_save_prompt=True)
        self.temp.cleanup()

    def make_geometry(self):
        geo = hou.node("/obj").createNode("geo", "test_geo", run_init_scripts=False)
        box = geo.createNode("box", "box1")
        box.setDisplayFlag(True)
        return geo, box

    def test_checkpoint_switch_branch_and_return(self):
        _, box = self.make_geometry()
        working_path, hip = hou.hipFile.path(), hou.getenv("HIP")
        a = self.workspace.checkpoint("Original")
        self.assertEqual(hou.hipFile.path(), working_path)
        self.assertEqual(hou.getenv("HIP"), hip)
        box.parm("sizex").set(3)
        b = self.workspace.checkpoint("Wide")
        box.parm("sizex").set(7)  # Unsaved manual changes must survive switching!
        result = self.workspace.visit(a["id"])
        preserved = result["preserved"]
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 1)
        self.assertEqual(hou.getenv("HIP"), hip)
        hou.node("/obj/test_geo/box1").parm("sizey").set(4)
        branch = self.workspace.checkpoint("Tall alternative")
        self.assertEqual(branch["parent"], a["id"])
        self.workspace.visit(preserved)
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 7)
        self.workspace.visit(b["id"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 3)
        self.assertTrue(all(self.workspace.history.scene_path(r["id"]).exists()
                            for r in self.workspace.history.entries))

    def test_unchanged_visits_and_return_do_not_create_checkpoints(self):
        _, box = self.make_geometry()
        original = self.workspace.checkpoint("Original")
        box.parm("sizex").set(3)
        wide = self.workspace.checkpoint("Wide")
        for checkpoint_id in [original["id"], wide["id"], original["id"], wide["id"]] * 3:
            result = self.workspace.visit(checkpoint_id)
            self.assertFalse(result["created_checkpoint"])
            self.assertEqual(len(self.workspace.history.entries), 2)
        self.workspace.visit(self.workspace.return_target)
        self.assertEqual(len(self.workspace.history.entries), 2)
        result = self.workspace.visit(self.workspace.active)
        self.assertTrue(result["unchanged"])
        self.assertEqual(len(self.workspace.history.entries), 2)

    def test_unchanged_navigation_after_workspace_reload(self):
        _, box = self.make_geometry()
        original = self.workspace.checkpoint("Original")
        box.parm("sizex").set(3)
        wide = self.workspace.checkpoint("Wide")
        self.workspace.close()
        self.workspace = Workspace(root=self.temp.name)
        self.assertFalse(self.workspace.has_live_changes())
        self.workspace.visit(original["id"])
        self.workspace.visit(wide["id"])
        self.assertEqual(len(self.workspace.history.entries), 2)

    def test_non_graph_edits_are_still_preserved(self):
        geo, box = self.make_geometry()
        original = self.workspace.checkpoint("Original")
        # Neither sticky-note text nor user data is in graph_snapshot().
        note = geo.createStickyNote()
        note.setText("Do not lose this manual annotation")
        box.setUserData("artist_note", "Keep this too")
        result = self.workspace.visit(original["id"])
        self.assertTrue(result["created_checkpoint"])
        self.workspace.visit(result["preserved"])
        self.assertEqual(hou.node("/obj/test_geo/box1").userData("artist_note"), "Keep this too")
        self.assertEqual(hou.node("/obj/test_geo").stickyNotes()[0].text(),
                         "Do not lose this manual annotation")

    def test_replace_version_retains_prior_revision_and_other_versions(self):
        _, box = self.make_geometry()
        original = self.workspace.checkpoint("Original")
        original_digest = original["sha256"]
        box.parm("sizex").set(3)
        wide = self.workspace.checkpoint("Wide")
        box.parm("sizex").set(7)
        updated = self.workspace.replace(original["id"])
        self.assertEqual(updated["label"], "Original")
        self.assertEqual(updated["replaces"], original["id"])
        self.assertEqual(updated["based_on"], wide["id"])
        self.assertEqual(original["replaced_by"], updated["id"])
        self.assertEqual(self.workspace.history.get(original["id"])["sha256"], original_digest)
        self.assertEqual(wide["parent"], original["id"])
        self.assertEqual(len(self.workspace.history.entries), 3)
        self.workspace.visit(wide["id"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 3)
        self.workspace.visit(updated["id"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 7)
        self.workspace.visit(original["id"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 1)
        self.assertEqual(len(self.workspace.history.entries), 3)

    def test_replace_rejects_pinned_versions_and_save_failure_keeps_original(self):
        _, box = self.make_geometry()
        original = self.workspace.checkpoint("Original")
        self.workspace.history.pin(original["id"], True)
        box.parm("sizex").set(5)
        with self.assertRaisesRegex(ValueError, "Unpin"):
            self.workspace.replace(original["id"])
        self.workspace.history.pin(original["id"], False)
        with patch.object(Workspace, "checkpoint", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.workspace.replace(original["id"])
        self.assertNotIn("replaced_by", original)
        self.assertEqual(len(self.workspace.history.entries), 1)
        self.assertEqual(box.parm("sizex").eval(), 5)

    def test_replacing_unchanged_active_version_is_noop(self):
        self.make_geometry()
        original = self.workspace.checkpoint("Original")
        result = self.workspace.replace(original["id"])
        self.assertEqual(result["id"], original["id"])
        self.assertEqual(len(self.workspace.history.entries), 1)

    def test_visiting_active_base_preserves_live_edits(self):
        _, box = self.make_geometry()
        original = self.workspace.checkpoint("Original")
        box.parm("sizex").set(11)
        result = self.workspace.visit(original["id"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 1)
        self.workspace.visit(result["preserved"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 11)

    def test_relative_hip_dependency_after_checkout(self):
        geo, box = self.make_geometry()
        geometry_path = Path(self.temp.name) / "source.bgeo.sc"
        box.geometry().saveToFile(str(geometry_path))
        file_node = geo.createNode("file", "dependency")
        file_node.parm("file").set("$HIP/source.bgeo.sc")
        original = self.workspace.checkpoint("Relative dependency")
        box.parm("sizex").set(5)
        self.workspace.checkpoint("Changed")
        self.workspace.visit(original["id"])
        node = hou.node("/obj/test_geo/dependency")
        node.cook(force=True)
        self.assertEqual(node.geometry().intrinsicValue("pointcount"), 8)
        self.assertEqual(node.errors(), ())

    def test_type_and_schema_discovery(self):
        geo, box = self.make_geometry()
        result = self.tools.inspect("types", geo.path(), "box")
        self.assertTrue(any(t["name"] == "box" for t in result["types"]))
        schema = self.tools.inspect("schema", box.path(), limit=100)
        self.assertTrue(any(p["name"] == "sizex" for p in schema["parameters"]))

    def test_pre_switch_save_failure_aborts(self):
        _, box = self.make_geometry()
        a = self.workspace.checkpoint("Original")
        box.parm("sizex").set(5)
        self.workspace.checkpoint("Changed")
        box.parm("sizex").set(6)
        with patch.object(Workspace, "checkpoint", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.workspace.visit(a["id"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 6)

    def test_corrupt_target_does_not_touch_scene(self):
        _, box = self.make_geometry()
        a = self.workspace.checkpoint("Original")
        box.parm("sizex").set(5)
        self.workspace.checkpoint("Changed")
        self.workspace.history.scene_path(a["id"]).write_bytes(b"broken")
        with self.assertRaises(RuntimeError):
            self.workspace.visit(a["id"])
        self.assertEqual(box.parm("sizex").eval(), 5)

    def test_failed_load_recovers_preserved_scene(self):
        _, box = self.make_geometry()
        a = self.workspace.checkpoint("Original")
        box.parm("sizex").set(9)
        self.workspace.checkpoint("Changed")
        original_load = self.workspace._load
        calls = []
        def fail_once(path, record):
            calls.append(path)
            if len(calls) == 1:
                hou.node("/obj/test_geo").destroy()
                raise RuntimeError("simulated partially failed load")
            original_load(path, record)
        with patch.object(self.workspace, "_load", side_effect=fail_once):
            with self.assertRaisesRegex(RuntimeError, "recovered"):
                self.workspace.visit(a["id"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 9)

    def test_persistent_binding(self):
        self.make_geometry()
        a = self.workspace.checkpoint("Original")
        root = self.workspace.history.root
        self.workspace.close()
        self.workspace = Workspace(root=self.temp.name)
        self.assertEqual(self.workspace.history.root, root)
        self.assertEqual(self.workspace.active, a["id"])

    def test_inspection_and_geometry(self):
        self.make_geometry()
        search = self.tools.inspect("search", "/", "box1")
        self.assertTrue(any(r["path"] == "/obj/test_geo/box1" for r in search["matches"]))
        info = self.tools.inspect("geometry", "/obj/test_geo/box1")
        self.assertEqual(info["points"], 8)
        self.assertEqual(info["primitives"], 6)
        self.assertTrue(ui_context()["top_level"])

    def test_proposal_apply_checkpoints_and_diff(self):
        self.make_geometry()
        result = self.tools.propose("Widen box", [
            {"op": "set", "path": "/obj/test_geo/box1", "parm": "sizex", "value": 3}])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 1)
        applied = self.tools.apply(result["proposal"])
        self.assertEqual(applied["status"], "applied")
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 3)
        self.assertIn("/obj/test_geo/box1", applied["diff"]["modified"])
        self.assertEqual(len(self.workspace.history.entries), 2)
        self.workspace.visit(applied["baseline"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 1)

    def test_stale_proposal_rejected(self):
        _, box = self.make_geometry()
        result = self.tools.propose("Widen", [
            {"op": "set", "path": box.path(), "parm": "sizex", "value": 3}])
        box.parm("sizey").set(2)
        with self.assertRaisesRegex(ValueError, "Scene changed"):
            self.tools.apply(result["proposal"])
        self.assertEqual(box.parm("sizex").eval(), 1)

    def test_failed_batch_is_recorded_and_recoverable(self):
        self.make_geometry()
        result = self.tools.propose("Partial", [
            {"op": "set", "path": "/obj/test_geo/box1", "parm": "sizex", "value": 3},
            {"op": "set", "path": "/obj/test_geo/box1", "parm": "nonexistent", "value": 1}])
        applied = self.tools.apply(result["proposal"])
        self.assertEqual(applied["status"], "failed")
        self.assertIn("Parameter not found", applied["error"])
        self.workspace.visit(applied["baseline"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 1)

    def test_read_only_and_cook_permissions(self):
        self.make_geometry()
        tools = SceneTools(self.workspace)
        with self.assertRaises(PermissionError):
            tools.dispatch("ko_propose", {"label": "test", "steps_json": "[]"})
        with self.assertRaises(PermissionError):
            tools.inspect("geometry", "/obj/test_geo/box1")
        with self.assertRaises(PermissionError):
            tools.dispatch("execute", {})

    def test_create_connect_flags(self):
        result = self.tools.propose("Build box", [
            {"op": "create", "parent": "/obj", "type": "geo", "name": "candidate"},
            {"op": "create", "parent": "/obj/candidate", "type": "box", "name": "box1"},
            {"op": "create", "parent": "/obj/candidate", "type": "null", "name": "OUT"},
            {"op": "connect", "path": "/obj/candidate/OUT", "input": 0,
             "source": "/obj/candidate/box1", "output": 0},
            {"op": "flags", "path": "/obj/candidate/OUT", "display": True, "render": True}])
        applied = self.tools.apply(result["proposal"])
        self.assertEqual(applied["status"], "applied", applied.get("error"))
        self.assertEqual(hou.node("/obj/candidate/OUT").geometry().intrinsicValue("pointcount"), 8)
        record = graph_snapshot()["nodes"]["/obj/candidate/OUT"]
        self.assertEqual(record["inputs"][0]["source"], "/obj/candidate/box1")


if __name__ == "__main__":
    unittest.main(verbosity=2)
