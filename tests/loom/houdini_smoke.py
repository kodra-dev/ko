"""Run only in a separate disposable hython process; never in an artist's session."""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import hou

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python3.13libs"))
from loom.checkpoints.scene import Workspace, CHECKPOINT_KEY, WORKSPACE_KEY, graph_snapshot, ui_context
from loom.tools import SceneTools
from loom.checkpoints.storage import graph_diff


class HoudiniTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="loom-hom-test-")
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

    def test_python_and_structured_edits_share_one_ai_checkpoint(self):
        _, box = self.make_geometry()
        self.tools.begin_turn()
        self.tools.permission_mode = "scene_edit"
        result = self.tools.dispatch("loom_python", {"label": "Wider box", "code":
            "hou.node('/obj/test_geo/box1').parm('sizex').set(3)\nprint('python worked')"})
        self.assertEqual(result["status"], "applied")
        self.assertIn("python worked", result["execution"]["output"])
        self.assertNotIn("code", result)
        self.assertEqual(box.parm("sizex").eval(), 3)
        prepared = self.tools.propose("Taller box", [{"op": "set", "path": box.path(), "parm": "sizey", "value": 4}])
        structured = self.tools.apply(prepared["proposal"])
        second = self.tools.execute_python("Deeper box", "hou.node('/obj/test_geo/box1').parm('sizez').set(5)")
        self.assertEqual(result["result"], structured["result"])
        self.assertEqual(result["result"], second["result"])
        self.assertEqual(len(self.workspace.history.entries), 2)
        self.workspace.visit(result["baseline"])
        restored = hou.node("/obj/test_geo/box1")
        self.assertEqual(restored.parmTuple("size").eval(), (1, 1, 1))
        self.workspace.visit(result["result"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parmTuple("size").eval(), (3, 4, 5))

    def test_python_syntax_error_has_no_checkpoint_or_scene_effects(self):
        self.make_geometry()
        self.tools.permission_mode = "scene_edit"
        result = self.tools.execute_python("Invalid", "if:")
        self.assertEqual(result["status"], "failed")
        self.assertTrue(result["retryable"])
        self.assertEqual(len(self.workspace.history.entries), 0)

    def test_python_partial_failure_is_saved_and_reported_not_rolled_back(self):
        _, box = self.make_geometry()
        self.tools.permission_mode = "scene_edit"
        result = self.tools.execute_python("Partial edit",
            "hou.node('/obj/test_geo/box1').parm('sizex').set(9)\nprint('before failure')\nraise ValueError('expected failure')")
        self.assertEqual(result["status"], "failed")
        self.assertIn("expected failure", result["error"])
        self.assertTrue(result["retryable"])
        self.assertIn("before failure", result["execution"]["output"])
        self.assertEqual(box.parm("sizex").eval(), 9)
        self.assertTrue(self.workspace.history.get(result["result"])["label"].startswith("Partial:"))

    def test_python_system_exit_is_captured_and_trace_restored(self):
        self.tools.permission_mode = "scene_edit"
        previous = sys.gettrace()
        result = self.tools.execute_python("Exit test", "print('alive')\nraise SystemExit(2)")
        self.assertEqual(result["status"], "failed")
        self.assertIn("SystemExit", result["error"])
        self.assertFalse(result["retryable"])
        self.assertIs(sys.gettrace(), previous)

    def test_python_loop_cancellation_is_cooperative_and_restores_trace(self):
        self.tools.permission_mode = "scene_edit"
        polls = []
        def cancel():
            polls.append(True)
            if len(polls) > 1:
                raise RuntimeError("cancelled test loop")
        self.tools.progress = cancel
        previous = sys.gettrace()
        result = self.tools.execute_python("Loop test", "while True:\n    x = 1")
        self.assertEqual(result["status"], "failed")
        self.assertIn("cancelled test loop", result["error"])
        self.assertFalse(result["retryable"])
        self.assertIs(sys.gettrace(), previous)

    def test_python_hip_switch_does_not_save_result_into_new_workspace(self):
        self.tools.permission_mode = "scene_edit"
        with patch.object(self.workspace, "save_current", wraps=self.workspace.save_current) as save:
            result = self.tools.execute_python("Bad scene switch", "hou.hipFile.clear(suppress_save_prompt=True)")
        self.assertEqual(result["status"], "failed")
        self.assertIn("workspace", result["error"])
        self.assertFalse(result["retryable"])
        # Initial checkpoint creation may save; no final result save after the switch.
        self.assertFalse(any(call.kwargs.get("force") for call in save.call_args_list))

    def make_versions(self):
        _, box = self.make_geometry()
        original = self.workspace.checkpoint("Original")
        wide = self.workspace.checkpoint("Wide")
        box.parm("sizex").set(3)
        self.workspace.save_current()
        return original, wide

    def test_switch_updates_old_checkpoint_without_extra_copy(self):
        a, b = self.make_versions()
        path, hip = hou.hipFile.path(), hou.getenv("HIP")
        hou.node("/obj/test_geo/box1").parm("sizex").set(7)
        result = self.workspace.visit(a["id"])
        self.assertTrue(result["updated_previous"])
        self.assertEqual(len(self.workspace.history.entries), 2)
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 1)
        self.assertEqual((hou.hipFile.path(), hou.getenv("HIP")), (path, hip))
        self.workspace.visit(b["id"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 7)

    def test_visit_continues_after_successful_save_with_warning(self):
        a, b = self.make_versions()
        hou.node("/obj/test_geo/box1").parm("sizex").set(7)
        original_hscript = hou.hscript
        warning = 'Warning:     Bad parameter reference: "../advdt" in parameter /obj/test_geo/adt.\n'

        def warn_on_save(command):
            output, error = original_hscript(command)
            return (output, warning + error) if command.startswith("mwrite ") else (output, error)

        with patch.object(hou, "hscript", side_effect=warn_on_save):
            result = self.workspace.visit(a["id"])
        self.assertEqual(result["active"], a["id"])
        self.assertEqual(self.workspace.active, a["id"])
        self.assertIn("Bad parameter reference", self.workspace.last_warning)
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 1)
        self.workspace.visit(b["id"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 7)
        self.assertEqual(self.workspace.last_warning, "")

    def test_visit_still_aborts_on_save_error_with_warning(self):
        a, b = self.make_versions()
        hou.node("/obj/test_geo/box1").parm("sizex").set(7)
        original_hscript = hou.hscript

        def fail_on_save(command):
            if command.startswith("mwrite "):
                return "", "Warning: parameter reference\nError: unable to write file\n"
            return original_hscript(command)

        with patch.object(hou, "hscript", side_effect=fail_on_save):
            with self.assertRaisesRegex(RuntimeError, "unable to write file"):
                self.workspace.visit(a["id"])
        self.assertEqual(self.workspace.active, b["id"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 7)

    def test_new_checkpoint_flushes_parent_then_activates_child(self):
        _, box = self.make_geometry()
        a = self.workspace.checkpoint("Original")
        box.parm("sizex").set(3)
        b = self.workspace.checkpoint("Alternative")
        self.assertEqual(b["parent"], a["id"])
        self.assertEqual(self.workspace.graph(a["id"])["nodes"][box.path()]["parms"]["sizex"], "3")
        box.parm("sizex").set(7)
        self.workspace.visit(a["id"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 3)
        self.workspace.visit(b["id"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 7)

    def test_unchanged_visits_and_reload_do_not_create_versions(self):
        a, b = self.make_versions()
        self.workspace.close()
        self.workspace = Workspace(root=self.temp.name)
        for key in [a["id"], b["id"]] * 3:
            self.workspace.visit(key)
            self.assertEqual(len(self.workspace.history.entries), 2)

    def test_visiting_current_is_noop_even_with_edits(self):
        _, box = self.make_geometry()
        a = self.workspace.checkpoint("Original")
        box.parm("sizex").set(11)
        with patch.object(self.workspace, "_load") as load:
            result = self.workspace.visit(a["id"])
        load.assert_not_called()
        self.assertTrue(result["unchanged"])
        self.assertEqual(box.parm("sizex").eval(), 11)

    def test_non_graph_edits_save_into_current(self):
        a, b = self.make_versions()
        geo = hou.node("/obj/test_geo")
        geo.createStickyNote().setText("Keep annotation")
        hou.node("/obj/test_geo/box1").setUserData("artist_note", "Keep too")
        self.workspace.visit(a["id"])
        self.workspace.visit(b["id"])
        self.assertEqual(hou.node("/obj/test_geo/box1").userData("artist_note"), "Keep too")
        self.assertEqual(hou.node("/obj/test_geo").stickyNotes()[0].text(), "Keep annotation")
        self.assertEqual(len(self.workspace.history.entries), 2)

    def test_replace_same_id_no_revision(self):
        a, b = self.make_versions()
        hou.node("/obj/test_geo/box1").parm("sizex").set(7)
        updated = self.workspace.replace(a["id"])
        self.assertEqual((updated["id"], updated["label"]), (a["id"], "Original"))
        self.assertEqual(self.workspace.active, a["id"])
        self.assertEqual(len(self.workspace.history.entries), 2)
        self.workspace.visit(b["id"])
        self.assertEqual(hou.node("/obj/test_geo/box1").parm("sizex").eval(), 7)

    def test_atomic_update_failure_preserves_old_saved_files(self):
        _, box = self.make_geometry()
        a = self.workspace.checkpoint("Original")
        old_path = self.workspace.history.verify(a["id"])
        box.parm("sizex").set(5)
        with patch("loom.checkpoints.storage.atomic_json", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.workspace.save_current()
        self.assertEqual(self.workspace.history.get(a["id"])["sha256"], a["sha256"])
        self.assertTrue(old_path.exists())
        self.workspace.history.verify(a["id"])
        self.assertEqual(box.parm("sizex").eval(), 5)

    def test_builtin_save_updates_current_without_recursion(self):
        _, box = self.make_geometry()
        a = self.workspace.checkpoint("Original")
        box.parm("sizex").set(5)
        hou.hipFile.save()
        self.assertIsNotNone(self.workspace._pending_native_save)
        with patch("hou.hscript", wraps=hou.hscript) as hscript:
            self.workspace.process_pending_save()
        self.assertEqual(hscript.call_count, 0)  # Copy the saved HIP; do not issue another Houdini save.
        self.assertIsNone(self.workspace._pending_native_save)
        self.assertEqual(self.workspace.active, a["id"])
        self.assertEqual(self.workspace.graph(a["id"])["nodes"][box.path()]["parms"]["sizex"], "5")
        self.assertEqual(len(self.workspace.history.entries), 1)

    def test_save_as_then_visit_keeps_new_filename_and_hip(self):
        a, b = self.make_versions()
        hou.hipFile.save(str(Path(self.temp.name) / "renamed.hip"))
        self.workspace.process_pending_save()
        path, hip = hou.hipFile.path(), hou.getenv("HIP")
        self.workspace.visit(a["id"])
        self.assertEqual((hou.hipFile.path(), hou.getenv("HIP")), (path, hip))
        self.workspace.visit(b["id"])
        hou.hipFile.save()
        self.workspace.process_pending_save()
        self.assertTrue(Path(path).exists())
        self.assertEqual(hou.hipFile.path(), path)

    def test_native_save_checkpoint_failure_is_reported(self):
        self.make_geometry()
        a = self.workspace.checkpoint("Original")
        hou.hipFile.save()
        with patch.object(self.workspace, "_copy_native_save", side_effect=OSError("checkpoint disk full")):
            self.workspace.process_pending_save()
        self.assertIn("Working HIP saved", self.workspace.save_warning)
        self.assertTrue(Path(hou.hipFile.path()).exists())
        self.workspace.history.verify(a["id"])

    def test_capture_saves_current_without_requiring_selection_or_clean_graph(self):
        a, b = self.make_versions()
        box = hou.node("/obj/test_geo/box1")
        box.parm("sizex").set(8)
        hou.setFrame(hou.frame() + 1)
        def capture(directory):
            self.workspace.history.verify(b["id"])
            self.assertEqual(self.workspace.graph(b["id"])["nodes"][box.path()]["parms"]["sizex"], "8")
            (directory / "preview-test.png").write_bytes(b"fake preview")
            return "preview-test.png"
        with patch("loom.checkpoints.scene.capture_preview", side_effect=capture):
            record = self.workspace.capture_current_preview()
        self.assertEqual(record["preview"], "preview-test.png")
        self.assertEqual(self.workspace.active, b["id"])
        self.assertEqual(len(self.workspace.history.entries), 2)

    def test_capture_save_failure_does_not_start_capture(self):
        self.make_versions()
        with patch.object(self.workspace, "save_current", side_effect=OSError("disk full")), \
             patch("loom.checkpoints.scene.capture_preview") as capture:
            with self.assertRaises(OSError):
                self.workspace.capture_current_preview()
            capture.assert_not_called()

    def test_capture_cook_changes_are_saved_before_attaching_preview(self):
        a, b = self.make_versions()
        def capture(directory):
            hou.node("/obj/test_geo/box1").parm("sizex").set(10)
            (directory / "cooked.png").write_bytes(b"preview")
            return "cooked.png"
        with patch("loom.checkpoints.scene.capture_preview", side_effect=capture):
            self.workspace.capture_current_preview()
        self.assertEqual(self.workspace.graph(b["id"])["nodes"]["/obj/test_geo/box1"]["parms"]["sizex"], "10")
        self.assertEqual(self.workspace.history.get(b["id"])["preview"], "cooked.png")

    def test_native_save_does_not_mistake_later_live_edits_for_saved_contents(self):
        a, b = self.make_versions()
        box = hou.node("/obj/test_geo/box1")
        box.parm("sizex").set(5)
        hou.hipFile.save()
        box.parm("sizex").set(7)
        self.workspace.process_pending_save()
        self.assertEqual(self.workspace.graph(b["id"])["nodes"][box.path()]["parms"]["sizex"], "5")
        self.assertTrue(self.workspace.has_live_changes())
        self.assertEqual(box.parm("sizex").eval(), 7)

    def test_failed_capture_keeps_successfully_saved_scene(self):
        a, b = self.make_versions()
        hou.node("/obj/test_geo/box1").parm("sizex").set(9)
        with patch("loom.checkpoints.scene.capture_preview", side_effect=RuntimeError("no viewport")):
            with self.assertRaisesRegex(RuntimeError, "no viewport"):
                self.workspace.capture_current_preview()
        self.workspace.history.verify(b["id"])
        self.assertEqual(self.workspace.graph(b["id"])["nodes"]["/obj/test_geo/box1"]["parms"]["sizex"], "9")

    def test_delete_current_refused_and_children_reparented(self):
        a, b = self.make_versions()
        child = self.workspace.checkpoint("Child")
        with self.assertRaises(ValueError):
            self.workspace.delete(child["id"])
        self.workspace.delete(b["id"])
        self.assertEqual(self.workspace.history.get(child["id"])["parent"], a["id"])
        self.workspace.history.verify(child["id"])
        self.assertEqual(self.workspace.active, child["id"])

    def test_cleanup_saves_same_current_id(self):
        a, b = self.make_versions()
        hou.node("/obj/test_geo/box1").parm("sizex").set(12)
        self.workspace.cleanup()
        self.assertEqual([r["id"] for r in self.workspace.history.entries], [b["id"]])
        self.assertEqual(self.workspace.graph(b["id"])["nodes"]["/obj/test_geo/box1"]["parms"]["sizex"], "12")
        self.assertFalse(self.workspace.history.directory(a["id"]).exists())

    def test_cleanup_save_failure_deletes_nothing(self):
        a, b = self.make_versions()
        with patch.object(self.workspace, "save_current", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.workspace.cleanup()
        self.assertEqual(len(self.workspace.history.entries), 2)
        self.workspace.history.verify(a["id"])
        self.workspace.history.verify(b["id"])

    def test_cleanup_without_current_creates_one(self):
        self.make_geometry()
        result = self.workspace.cleanup()
        self.assertEqual(len(self.workspace.history.entries), 1)
        self.assertEqual(self.workspace.active, result["kept"])

    def test_visit_without_current_requires_first_checkpoint(self):
        a, b = self.make_versions()
        hou.node("/").destroyUserData(CHECKPOINT_KEY)
        with self.assertRaisesRegex(ValueError, "Create a checkpoint"):
            self.workspace.visit(a["id"])
        self.assertEqual(len(self.workspace.history.entries), 2)

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

    def test_type_schema_queries_do_not_return_object_transforms(self):
        geo, _ = self.make_geometry()
        for node_type, expected in (("subdivide", "iterations"), ("attribwrangle", "snippet")):
            result = self.tools.inspect("schema", geo.path(), node_type, limit=100)
            self.assertEqual(result["node_type"], node_type)
            names = [p["name"] for p in result["parameters"]]
            self.assertIn(expected, names)
            self.assertNotIn("xOrd", names)
        with self.assertRaises(ValueError):
            self.tools.inspect("schema", geo.path(), "nonexistent")

    def test_wrangle_internal_unconnected_inputs_do_not_break_followup_edits(self):
        geo, box = self.make_geometry()
        self.tools.begin_turn()
        first = self.tools.propose("Start subdivision", [
            {"op": "create", "parent": geo.path(), "type": "subdivide", "name": "subdiv"},
            {"op": "connect", "path": geo.path() + "/subdiv", "input": 0, "source": box.path(), "output": 0},
            {"op": "create", "parent": geo.path(), "type": "attribwrangle", "name": "shape"},
            {"op": "connect", "path": geo.path() + "/shape", "input": 0, "source": geo.path() + "/subdiv", "output": 0}])
        self.assertEqual(self.tools.apply(first["proposal"])["status"], "applied")
        graph_snapshot()  # Includes the wrangle's unconnected internal VOP inputs.
        second = self.tools.propose("Shape geometry", [
            {"op": "set", "path": geo.path() + "/subdiv", "parm": "iterations", "value": 2},
            {"op": "set", "path": geo.path() + "/shape", "parm": "class", "value": "detail"},
            {"op": "set", "path": geo.path() + "/shape", "parm": "snippet", "value": "i@verified = 1;"}])
        result = self.tools.apply(second["proposal"])
        self.assertEqual(result["status"], "applied", result.get("error"))
        self.assertEqual(geo.node("shape").geometry().attribValue("verified"), 1)
        self.tools.end_turn()

    def test_pre_switch_save_failure_aborts(self):
        _, box = self.make_geometry()
        a = self.workspace.checkpoint("Original")
        box.parm("sizex").set(5)
        self.workspace.checkpoint("Changed")
        box.parm("sizex").set(6)
        with patch.object(Workspace, "save_current", side_effect=OSError("disk full")):
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
        def fail_once(path, record, working_path=None):
            calls.append(path)
            if len(calls) == 1:
                hou.node("/obj/test_geo").destroy()
                raise RuntimeError("simulated partially failed load")
            original_load(path, record, working_path)
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

    def test_tuple_writes_validate_all_components_before_mutation(self):
        geo, box = self.make_geometry()
        def set_value(value):
            self.tools._apply_step({"op": "set", "path": box.path(), "parm": "size", "value": value})
        set_value([2, 3, 4])
        self.assertEqual(box.parmTuple("size").eval(), (2, 3, 4))
        with self.assertRaisesRegex(ValueError, "3 components"):
            set_value([7, 8])
        box.parm("sizez").setExpression("4")
        with self.assertRaisesRegex(ValueError, "animated/expression"):
            set_value([7, 8, 9])
        self.assertEqual(box.parmTuple("size").eval(), (2, 3, 4))
        box.parm("sizez").deleteAllKeyframes()
        box.parm("sizez").setKeyframe(hou.Keyframe(4))
        with self.assertRaisesRegex(ValueError, "animated/expression"):
            set_value([7, 8, 9])
        self.assertEqual(box.parm("sizex").eval(), 2)

    def test_container_selection_tuple_recovery_and_geometry_queries(self):
        geo, box = self.make_geometry()
        # Reproduce the original no-op: primitive group patterns do not promote point P.
        rim = geo.createNode("polyextrude::2.0", "rim")
        rim.setInput(0, box)
        rim.parm("group").set("@P.y>0.49")
        rim.parm("inset").set(0.1)
        self.assertEqual(rim.geometry().intrinsicValue("primitivecount"), 6)
        top = geo.createNode("groupcreate", "top")
        top.setInput(0, box)
        for parm, value in (("groupname", "top"), ("grouptype", "primitive"),
                            ("groupbase", False), ("groupnormal", True),
                            ("dir", [0, 1, 0]), ("angle", 5)):
            self.tools._apply_step({"op": "set", "path": top.path(), "parm": parm, "value": value})
        groups = self.tools.inspect("geometry", top.path(), "groups")
        self.assertIn({"name": "top", "type": "primitive", "count": 1}, groups["groups"])
        selected = self.tools.inspect("geometry", top.path(), "group:primitive:top")
        self.assertEqual(selected["total"], 1)
        prims = self.tools.inspect("geometry", box.path(), "primitives", limit=2)
        self.assertEqual((prims["total"], prims["next_offset"]), (6, 2))
        all_prims = self.tools.inspect("geometry", box.path(), "primitives")["primitives"]
        face = next(p for p in all_prims if p["number"] == selected["members"][0])
        self.assertAlmostEqual(face["center"][1], 0.5)
        self.assertEqual(face["normal"], [0, 1, 0])
        rim.setInput(0, top)
        rim.parm("group").set("top")
        rim.parm("outputfrontgrp").set(True)
        rim.parm("frontgrp").set("cavity")
        self.assertEqual(self.tools.inspect("geometry", rim.path(), "group:primitive:cavity")["total"], 1)
        depth = geo.createNode("polyextrude::2.0", "depth")
        depth.setInput(0, rim)
        depth.parm("group").set("cavity")
        depth.parm("dist").set(-0.9)
        result = self.tools.inspect("geometry", depth.path())
        self.assertFalse(result["errors"])
        self.assertGreater(result["primitives"], 6)
        cavity = depth.geometry().findPrimGroup("cavity").prims()[0]
        self.assertTrue(all(abs(p.position()[1] + 0.4) < 1e-6 for p in cavity.points()))
        self.assertEqual(result["bounds"], [[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]])
        with self.assertRaisesRegex(ValueError, "Unsupported geometry query"):
            self.tools.inspect("geometry", box.path(), "nonsense")
        with self.assertRaisesRegex(ValueError, "group not found"):
            self.tools.inspect("geometry", box.path(), "group:point:missing")

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
        self.assertEqual(self.workspace.graph(applied["baseline"])["nodes"]["/obj/test_geo/box1"]["parms"]["sizex"], "1")
        self.assertEqual(self.workspace.graph(applied["result"])["nodes"]["/obj/test_geo/box1"]["parms"]["sizex"], "3")
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

    def test_read_only_edits_and_automatic_geometry_inspection(self):
        self.make_geometry()
        tools = SceneTools(self.workspace)
        with self.assertRaises(PermissionError):
            tools.dispatch("loom_propose", {"label": "test", "steps_json": "[]"})
        self.assertEqual(tools.inspect("geometry", "/obj/test_geo/box1")["points"], 8)
        with self.assertRaises(PermissionError):
            tools.dispatch("execute", {})

    def test_ai_turn_saves_current_and_creates_only_one_short_marked_child(self):
        geo, box = self.make_geometry()
        box.setPosition(hou.Vector2(3, 7))
        baseline = self.workspace.checkpoint("Working scene")
        box.parm("sizex").set(2)
        self.tools.begin_turn()
        first = self.tools.propose("Extrude the selected cube faces with a very long description that should not become a wide checkpoint name", [
            {"op": "create", "parent": geo.path(), "type": "null", "name": "stage1"},
            {"op": "connect", "path": geo.path() + "/stage1", "input": 0, "source": box.path(), "output": 0}])
        a = self.tools.apply(first["proposal"])
        second = self.tools.propose("Finish result", [
            {"op": "create", "parent": geo.path(), "type": "null", "name": "OUT"},
            {"op": "connect", "path": geo.path() + "/OUT", "input": 0, "source": geo.path() + "/stage1", "output": 0}])
        b = self.tools.apply(second["proposal"])
        self.tools.end_turn()
        self.assertEqual(len(self.workspace.history.entries), 2)
        self.assertEqual(a["baseline"], baseline["id"])
        self.assertEqual(a["result"], b["result"])
        record = self.workspace.history.get(b["result"])
        self.assertEqual(record["created_by"], "ai")
        self.assertLessEqual(len(record["label"]), 40)
        self.assertFalse(record["label"].startswith(("AI:", "Before:")))
        self.assertEqual(tuple(box.position()), (3, 7))
        stage, out = geo.node("stage1"), geo.node("OUT")
        self.assertLess(stage.position()[1], box.position()[1])
        self.assertLess(out.position()[1], stage.position()[1])
        self.assertTrue(out.isDisplayFlagSet())
        box_path, out_path = box.path(), out.path()
        self.workspace.visit(baseline["id"])
        self.assertEqual(hou.node(box_path).parm("sizex").eval(), 2)  # live edits saved into current
        self.assertIsNone(hou.node(out_path))

    def test_explicit_display_choice_is_not_overridden(self):
        geo, box = self.make_geometry()
        proposal = self.tools.propose("Alternative", [
            {"op": "create", "parent": geo.path(), "type": "null", "name": "alternative"},
            {"op": "connect", "path": geo.path() + "/alternative", "input": 0, "source": box.path(), "output": 0},
            {"op": "flags", "path": box.path(), "display": True}])
        self.tools.apply(proposal["proposal"])
        self.assertTrue(box.isDisplayFlagSet())
        self.assertFalse(geo.node("alternative").isDisplayFlagSet())

    def test_displayed_downstream_output_is_preserved_when_editing_upstream(self):
        geo, box = self.make_geometry()
        out = geo.createNode("null", "OUT")
        out.setInput(0, box)
        out.setDisplayFlag(True)
        proposal = self.tools.propose("Widen cube", [
            {"op": "set", "path": box.path(), "parm": "sizex", "value": 3}])
        applied = self.tools.apply(proposal["proposal"])
        self.assertTrue(out.isDisplayFlagSet())
        self.assertEqual(applied["post_edit"]["display"], out.path())

    def test_ai_creation_api_strips_prefixes_and_caps_generated_titles(self):
        record = self.workspace.checkpoint("AI: After: Extrude the cube and make this generated checkpoint title far too long", created_by="ai")
        self.assertFalse(record["label"].startswith(("AI:", "After:")))
        self.assertLessEqual(len(record["label"]), 40)
        self.assertEqual(record["created_by"], "ai")
        self.workspace.history.rename(record["id"], "Custom name")
        self.workspace.save_current(force=True)
        self.assertEqual(self.workspace.history.get(record["id"])["created_by"], "ai")

    def test_default_layout_never_repositions_existing_nodes(self):
        geo, box = self.make_geometry()
        old = geo.createNode("null", "keep_here")
        box.setPosition(hou.Vector2(12, 4))
        old.setPosition(hou.Vector2(-8, -3))
        positions = {node.path(): tuple(node.position()) for node in (box, old)}
        proposal = self.tools.propose("Add output", [
            {"op": "create", "parent": geo.path(), "type": "null", "name": "OUT"},
            {"op": "connect", "path": geo.path() + "/OUT", "input": 0, "source": box.path(), "output": 0},
            {"op": "layout", "path": geo.path()}])
        self.tools.apply(proposal["proposal"])
        self.assertEqual({node.path(): tuple(node.position()) for node in (box, old)}, positions)

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
