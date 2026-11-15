"""Offline file-tool coverage; no Houdini, provider or artist files."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from loom.files import file_operations, requests_from_json, MAX_RESULT
from loom.execution import BoundedOutput, MAX_OUTPUT
from loom.tools import SceneTools


class FileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="loom-file-tools-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root / "feature.py"
        self.path.write_bytes(b"# feature\r\nvalue = 1\r\n")

    def run_ops(self, *requests):
        return file_operations(requests_from_json(json.dumps(requests)), lambda: None)

    def test_read_list_search_and_hash(self):
        result = self.run_ops({"op": "read", "path": str(self.path), "offset": 1, "limit": 1})
        read = result["results"][0]
        self.assertEqual(read["text"], "value = 1\r\n")
        self.assertEqual(read["sha256"], hashlib.sha256(self.path.read_bytes()).hexdigest())
        self.assertEqual(self.run_ops({"op": "search", "path": str(self.path), "query": "VALUE"})[
            "results"][0]["matches"][0]["line"], 2)
        self.assertEqual(self.run_ops({"op": "list", "path": str(self.root)})[
            "results"][0]["entries"][0]["name"], "feature.py")

    def test_exact_edit_preserves_crlf_and_rejects_stale_hash(self):
        sha = hashlib.sha256(self.path.read_bytes()).hexdigest()
        request = {"op": "edit", "path": str(self.path), "expected_sha256": sha,
                   "edits": [{"old": "value = 1", "new": "value = 2"}]}
        self.assertEqual(self.run_ops(request)["status"], "completed")
        self.assertEqual(self.path.read_bytes(), b"# feature\r\nvalue = 2\r\n")
        self.assertEqual(self.run_ops(request)["status"], "failed")

    def test_new_file_creation_never_overwrites_existing_without_hash(self):
        target = self.root / "new" / "tool.py"
        request = {"op": "write", "path": str(target), "content": "pass\n", "expected_sha256": ""}
        self.assertEqual(self.run_ops(request)["status"], "completed")
        self.assertEqual(self.run_ops(request)["status"], "failed")
        self.assertEqual(target.read_text(), "pass\n")

    def test_nonunique_and_overlapping_replacements_do_not_publish(self):
        self.path.write_text("abcdef abcdef")
        before = self.path.read_bytes()
        sha = hashlib.sha256(before).hexdigest()
        for edits in ([{"old": "abcdef", "new": "x"}],
                      [{"old": "abcdef abc", "new": "x"}, {"old": "def abcdef", "new": "y"}]):
            result = self.run_ops({"op": "edit", "path": str(self.path), "expected_sha256": sha, "edits": edits})
            self.assertEqual(result["status"], "failed")
            self.assertEqual(self.path.read_bytes(), before)

    def test_harness_and_credentials_are_blocked(self):
        import loom.files
        harness = Path(loom.files.__file__).resolve()
        result = self.run_ops({"op": "write", "path": str(harness), "content": "bad", "expected_sha256": ""})
        self.assertIn("running harness", result["results"][0]["error"])
        for target in (self.root / "auth.json", self.root / ".ssh" / "key"):
            result = self.run_ops({"op": "read", "path": str(target)})
            self.assertIn("Credential", result["results"][0]["error"])

    def test_failed_batch_is_explicitly_partial_and_stops(self):
        new = self.root / "new.py"
        result = self.run_ops({"op": "write", "path": str(new), "content": "first", "expected_sha256": ""},
                              {"op": "read", "path": str(self.root / "missing")},
                              {"op": "write", "path": str(self.root / "skipped"), "content": "x", "expected_sha256": ""})
        self.assertEqual(result["status"], "failed")
        self.assertTrue(result["partial"])
        self.assertEqual(new.read_text(), "first")
        self.assertFalse((self.root / "skipped").exists())

    def test_cancellation_keeps_completed_file_results_and_skips_remaining(self):
        calls = []
        def progress():
            calls.append(True)
            if len(calls) > 1:
                raise RuntimeError("cancelled file batch")
        requests = [{"op": "write", "path": str(self.root / name), "content": "ok", "expected_sha256": ""}
                    for name in ("first.py", "second.py")]
        result = file_operations(requests, progress)
        self.assertEqual(result["status"], "failed")
        self.assertTrue(result["partial"])
        self.assertEqual(result["results"][0]["status"], "written")
        self.assertIn("cancelled", result["results"][1]["error"])
        self.assertFalse((self.root / "second.py").exists())

    def test_atomic_failure_preserves_original_and_cleans_staging(self):
        before = self.path.read_bytes()
        with patch("loom.files.os.replace", side_effect=OSError("disk error")):
            result = self.run_ops({"op": "write", "path": str(self.path), "content": "new",
                                   "expected_sha256": hashlib.sha256(before).hexdigest()})
        self.assertEqual(result["status"], "failed")
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(list(self.root.glob("*.tmp")), [])

    def test_payloads_and_output_are_bounded(self):
        for requests in ([], [{"op": "read", "path": "relative.py"}],
                         [{"op": "delete", "path": str(self.path)}],
                         [{"op": "read", "path": str(self.path), "limit": -1}]):
            with self.assertRaises(ValueError):
                requests_from_json(json.dumps(requests))
        self.path.write_text("x" * 100000)
        result = self.run_ops({"op": "read", "path": str(self.path)})
        self.assertLess(len(json.dumps(result)), MAX_RESULT)
        self.assertTrue(result["results"][0]["truncated"])
        output = BoundedOutput()
        output.write("a" * MAX_OUTPUT)
        self.assertFalse(output.truncated)
        output.write("b" * 100000)
        self.assertTrue(output.truncated)
        self.assertLess(len(output.value()), MAX_OUTPUT + 50)

    def test_scene_mode_cannot_write_and_read_only_cannot_execute_python(self):
        tools = object.__new__(SceneTools)
        tools.permission_mode = "read_only"
        tools.tidy_scope = None
        tools.progress = lambda: None
        request = {"requests_json": json.dumps([{"op": "read", "path": str(self.path)}])}
        self.assertEqual(tools.dispatch("loom_files", request)["status"], "completed")
        with self.assertRaises(PermissionError):
            tools.dispatch("loom_python", {"label": "Blocked", "code": "raise SystemExit()"})
        tools.permission_mode = "scene_edit"
        request = {"requests_json": json.dumps([{"op": "write", "path": str(self.root / "blocked"),
                                                  "content": "x", "expected_sha256": ""}])}
        with self.assertRaises(PermissionError):
            tools.dispatch("loom_files", request)
        self.assertFalse((self.root / "blocked").exists())
        tools.permission_mode = "tool_dev"
        self.assertEqual(tools.dispatch("loom_files", request)["status"], "completed")
        tools.tidy_scope = object()
        with self.assertRaises(PermissionError):
            tools.dispatch("loom_python", {"label": "Blocked", "code": "pass"})

    def test_legacy_context_route_preserves_permissions(self):
        tools = object.__new__(SceneTools)
        tools.permission_mode = "read_only"
        tools.tidy_scope = None
        args = {"operation": "context", "path": "/", "query": "@python:" + json.dumps(
            {"label": "Blocked", "code": "print('never')"})}
        with self.assertRaises(PermissionError):
            tools.dispatch("ko_scene", args)


if __name__ == "__main__":
    unittest.main()
