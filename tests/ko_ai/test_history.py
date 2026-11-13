import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from ko_ai.storage import History, atomic_json, digest, graph_diff, read_json
from ko_ai.tools import validate_steps


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
        with patch("ko_ai.storage.atomic_json", side_effect=OSError("disk full")):
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
        with self.assertRaises(ValueError):
            self.history.delete("1" * 32)

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
            with self.assertRaises(OSError):
                self.history.delete("1" * 32)
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
