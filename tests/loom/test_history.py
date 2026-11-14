import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from loom.checkpoints.storage import History, atomic_json, digest, graph_diff, read_json
from loom.tools import validate_steps


class HistoryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.history = History("a" * 32, self.temp.name)

    def tearDown(self):
        self.history.close()
        self.temp.cleanup()

    def checkpoint(self, key, parent=None, replaces=None):
        directory = self.history.directory(key)
        directory.mkdir(parents=True)
        scene = directory / "scene.hip"
        scene.write_bytes(b"fake hip " + key.encode())
        record = {"id": key, "parent": parent, "label": key, "pinned": False,
                  "file": "scene.hip", "sha256": digest(scene)}
        if replaces:
            record["replaces"] = replaces
        self.history.commit(record)
        return record

    def test_branch_survives_deleting_ancestor(self):
        self.checkpoint("1" * 32)
        self.checkpoint("2" * 32, "1" * 32)
        self.checkpoint("3" * 32, "1" * 32)
        self.history.delete("1" * 32)
        self.assertTrue(self.history.verify("2" * 32).exists())
        self.assertTrue(self.history.verify("3" * 32).exists())

    def test_replacement_keeps_old_snapshot_and_recovers_index_relationship(self):
        original = self.checkpoint("1" * 32)
        replacement = self.checkpoint("2" * 32, replaces=original["id"])
        self.assertEqual(original["replaced_by"], replacement["id"])
        self.assertTrue(self.history.verify(original["id"]).exists())
        del original["replaced_by"]
        self.history.flush()
        self.history.close()
        self.history = History("a" * 32, self.temp.name)
        self.assertEqual(self.history.get(original["id"])["replaced_by"], replacement["id"])
        self.assertTrue(self.history.verify(replacement["id"]).exists())

    def test_bulk_cleanup_keeps_current_and_explicitly_includes_pins(self):
        original = self.checkpoint("1" * 32)
        current = self.checkpoint("2" * 32, replaces=original["id"])
        self.history.pin(original["id"], True)
        with self.assertRaises(ValueError):
            self.history.prune_except(current["id"])
        result = self.history.prune_except(current["id"], include_pinned=True)
        self.assertEqual(result["removed"], [original["id"]])
        self.assertEqual([r["id"] for r in self.history.entries], [current["id"]])
        self.assertNotIn("replaces", self.history.get(current["id"]))
        self.assertIsNone(self.history.get(current["id"])["parent"])
        self.assertTrue(self.history.verify(current["id"]).exists())
        self.assertFalse(self.history.directory(original["id"]).exists())

    def test_bulk_cleanup_index_failure_deletes_nothing(self):
        self.checkpoint("1" * 32)
        self.checkpoint("2" * 32)
        with patch("loom.checkpoints.storage.atomic_json", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.history.prune_except("2" * 32)
        self.assertEqual(len(self.history.entries), 2)
        self.assertTrue(self.history.verify("1" * 32).exists())
        self.assertTrue(self.history.verify("2" * 32).exists())

    def test_locked_cleanup_files_are_retried_only_on_explicit_cleanup(self):
        self.checkpoint("1" * 32)
        self.checkpoint("2" * 32)
        with patch("shutil.rmtree", side_effect=OSError("locked")):
            result = self.history.prune_except("2" * 32)
        self.assertEqual(len(result["file_errors"]), 1)
        self.history.close()
        self.history = History("a" * 32, self.temp.name)
        self.assertEqual(len(self.history.entries), 1)
        self.assertTrue(self.history.directory("1" * 32).exists())
        self.assertEqual(self.history.index["pending_cleanup"], ["1" * 32])
        result = self.history.prune_except("2" * 32)
        self.assertFalse(result["file_errors"])
        self.assertFalse(self.history.directory("1" * 32).exists())
        self.assertEqual(self.history.index["pending_cleanup"], [])

    def test_cleanup_protection(self):
        self.checkpoint("1" * 32)
        with self.assertRaises(ValueError):
            self.history.delete("1" * 32, protected=["1" * 32])
        self.history.pin("1" * 32, True)
        self.history.delete("1" * 32)  # Legacy pins no longer block explicit deletion.
        self.assertEqual(self.history.entries, [])

    def test_delete_reparents_children_and_survives_reopen(self):
        root = self.checkpoint("1" * 32)
        middle = self.checkpoint("2" * 32, root["id"])
        child = self.checkpoint("3" * 32, middle["id"])
        self.history.delete(middle["id"])
        self.assertEqual(self.history.get(child["id"])["parent"], root["id"])
        self.history.close()
        self.history = History("a" * 32, self.temp.name)
        self.assertEqual(self.history.get(child["id"])["parent"], root["id"])
        self.history.delete(root["id"])
        self.assertIsNone(self.history.get(child["id"])["parent"])

    def test_delete_index_failure_does_not_change_memory_or_files(self):
        a = self.checkpoint("1" * 32)
        b = self.checkpoint("2" * 32, a["id"])
        with patch("loom.checkpoints.storage.atomic_json", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.history.delete(a["id"])
        self.assertEqual(self.history.get(b["id"])["parent"], a["id"])
        self.history.verify(a["id"])
        self.history.verify(b["id"])

    def test_update_and_rename_are_atomic_and_keep_identity(self):
        record = self.checkpoint("1" * 32)
        updated = {**record, "label": "Updated", "updated": "2026-11-14T12:34:56+00:00"}
        self.history.update(updated)
        self.assertEqual(len(self.history.entries), 1)
        with patch("loom.checkpoints.storage.atomic_json", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.history.rename(record["id"], "Not committed")
        self.assertEqual(self.history.get(record["id"])["label"], "Updated")
        self.history.close()
        self.history = History("a" * 32, self.temp.name)
        self.assertEqual(self.history.get(record["id"])["updated"], updated["updated"])

    def test_corruption_detected(self):
        self.checkpoint("1" * 32)
        self.history.scene_path("1" * 32).write_bytes(b"broken")
        with self.assertRaises(RuntimeError):
            self.history.verify("1" * 32)

    def test_workspace_exclusive_lock(self):
        with self.assertRaises(RuntimeError):
            History("a" * 32, self.temp.name)

    def test_index_crash_recovery(self):
        record = self.checkpoint("1" * 32)
        self.history.index["checkpoints"] = []
        self.history.flush()
        self.history.close()
        self.history = History("a" * 32, self.temp.name)
        self.assertEqual(self.history.get(record["id"])["id"], record["id"])

    def test_tombstone_prevents_resurrection(self):
        self.checkpoint("1" * 32)
        with patch("shutil.rmtree", side_effect=OSError("locked")):
            result = self.history.delete("1" * 32)
        self.assertTrue(result["file_errors"])
        self.history.close()
        self.history = History("a" * 32, self.temp.name)
        self.assertEqual(self.history.entries, [])

    def test_atomic_write_preserves_previous_on_failure(self):
        path = Path(self.temp.name) / "test.json"
        atomic_json(path, {"old": True})
        with patch("os.replace", side_effect=OSError("disk error")):
            with self.assertRaises(OSError):
                atomic_json(path, {"new": True})
        self.assertEqual(read_json(path), {"old": True})

    def test_path_traversal_rejected(self):
        with self.assertRaises(ValueError):
            self.history.directory("../../elsewhere")

    def test_delete_descendants_is_atomic_and_keeps_parent_and_other_branches(self):
        from unittest.mock import patch
        root = self.checkpoint("1" * 32)
        # Reuse the storage fixture's parent fields explicitly.
        child = self.checkpoint("2" * 32, parent=root["id"])
        grandchild = self.checkpoint("3" * 32, parent=child["id"])
        other = self.checkpoint("4" * 32)
        with patch("loom.checkpoints.storage.atomic_json", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.history.delete_descendants(root["id"])
        self.assertEqual(len(self.history.entries), 4)
        self.history.delete_descendants(root["id"])
        self.assertEqual({r["id"] for r in self.history.entries}, {root["id"], other["id"]})

    def test_delete_descendants_refuses_current_anywhere_below_parent(self):
        root = self.checkpoint("1" * 32)
        child = self.checkpoint("2" * 32, parent=root["id"])
        grandchild = self.checkpoint("3" * 32, parent=child["id"])
        with self.assertRaisesRegex(ValueError, "current"):
            self.history.delete_descendants(root["id"], protected=[grandchild["id"]])
        self.assertEqual(len(self.history.entries), 3)
        self.history.delete_descendants(root["id"], protected=[root["id"]])
        self.assertEqual(len(self.history.entries), 1)

    def test_graph_diff(self):
        a = {"nodes": {"/obj/a": {"parms": {"x": 1}}, "/obj/deleted": {}}}
        b = {"nodes": {"/obj/a": {"parms": {"x": 2}}, "/obj/new": {}}}
        result = graph_diff(a, b)
        self.assertEqual(result["created"], ["/obj/new"])
        self.assertEqual(result["deleted"], ["/obj/deleted"])
        self.assertEqual(result["modified"]["/obj/a"]["parms"]["after"], {"x": 2})

    def test_bad_operations_rejected(self):
        for steps in ([{"op": "execute", "code": "pass"}],
                      [{"op": "delete", "path": "/obj"}],
                      [{"op": "create", "parent": "/obj", "type": "geo", "name": "../bad"}],
                      [{"op": "set", "path": "relative", "parm": "x", "value": 1}],
                      [{"op": "flags", "path": "/obj/a", "display": "yes"}]):
            with self.assertRaises(ValueError):
                validate_steps(steps)


if __name__ == "__main__":
    unittest.main()
