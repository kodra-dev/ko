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

    def test_recursive_find_search_and_paging(self):
        nested = self.root / "nested"
        nested.mkdir()
        (nested / "Other.PY").write_text("target\nTARGET again\n", encoding="utf-8")
        request = {"op": "find", "path": str(self.root), "pattern": "*.py", "limit": 1}
        first = self.run_ops(request)["results"][0]
        self.assertEqual(first["matches"], [{"path": str(self.path)}])
        self.assertEqual(first["next_offset"], 1)
        second = self.run_ops(dict(request, offset=1))["results"][0]
        self.assertEqual(second["matches"][0]["path"], str(nested / "Other.PY"))
        self.assertIsNone(second["next_offset"])
        result = self.run_ops({"op": "search_tree", "path": str(self.root),
                               "query": "TARGET", "pattern": "*.py"})["results"][0]
        self.assertEqual([m["line"] for m in result["matches"]], [1, 2])
        self.assertFalse(result["partial"])
        self.assertEqual(result["matches"][1]["text"], "TARGET again")

    def test_recursive_protected_and_unsupported_files(self):
        for name in (".git", ".ssh", ".codex"):
            directory = self.root / name
            directory.mkdir()
            (directory / "secret.py").write_text("secret")
        (self.root / "auth.json").write_text("secret")
        (self.root / "binary").write_bytes(b"secret\0")
        (self.root / "nonutf8").write_bytes(b"\xff")
        (self.root / "huge").write_bytes(b"x" * 1_000_001)
        result = self.run_ops({"op": "search_tree", "path": str(self.root), "query": "secret"})["results"][0]
        self.assertEqual(result["matches"], [])
        self.assertEqual(result["skipped"]["protected"], 4)
        self.assertEqual(result["skipped"]["unsupported_text"], 3)
        self.assertTrue(result["partial"])

    def test_recursive_limits_and_validation(self):
        request = {"op": "find", "path": str(self.root)}
        with patch("loom.files.MAX_TREE_ENTRIES", 0):
            result = self.run_ops(request)["results"][0]
            self.assertIn("entry_limit", result["stop_reasons"])
            self.assertTrue(result["rescan_required"])
        nested = self.root / "nested"
        nested.mkdir()
        (nested / "hidden.py").write_text("target")
        with patch("loom.files.MAX_TREE_DEPTH", 0):
            self.assertIn("depth_limit", self.run_ops(request)["results"][0]["stop_reasons"])
        with patch("loom.files.MAX_TREE_BYTES", 1):
            result = self.run_ops(dict(request, op="search_tree", query="target"))["results"][0]
            self.assertIn("byte_limit", result["stop_reasons"])
        with patch("loom.files.MAX_TREE_OUTPUT", 1):
            result = self.run_ops(request)["results"][0]
            self.assertIn("output_limit", result["stop_reasons"])
        for bad in (dict(request, pattern="../*"), dict(request, pattern=""),
                    dict(request, op="search_tree", query=""), dict(request, path=str(self.path))):
            self.assertEqual(self.run_ops(bad)["status"], "failed")

    def test_recursive_output_paging_and_batch_trim_do_not_lose_matches(self):
        self.path.write_text(("target " + "x" * 1500 + "\n") * 50)
        request = {"op": "search_tree", "path": str(self.root), "query": "target"}
        result = self.run_ops(request)["results"][0]
        self.assertIn("output_limit", result["stop_reasons"])
        self.assertEqual(len(result["matches"][0]["text"]), 1000)
        self.assertEqual(result["next_offset"], len(result["matches"]))
        next_page = self.run_ops(dict(request, offset=result["next_offset"]))["results"][0]
        self.assertEqual(next_page["matches"][0]["line"], len(result["matches"]) + 1)
        batch = self.run_ops(request, request, request)
        trimmed = batch["results"][-1]
        self.assertTrue(trimmed["truncated"])
        self.assertEqual(trimmed["next_offset"], 0)
        self.assertEqual(trimmed["matches"], [])
        self.assertEqual(batch["remaining_requests_skipped"], 1)

    def test_recursive_inaccessible_directory_and_cancellation(self):
        request = {"op": "find", "path": str(self.root)}
        with patch("loom.files.os.scandir", side_effect=PermissionError("denied")):
            result = self.run_ops(request)["results"][0]
            self.assertTrue(result["partial"])
            self.assertEqual(result["skipped"], {"inaccessible": 1})
        calls = []
        def progress():
            calls.append(True)
            if len(calls) == 3:
                raise RuntimeError("cancelled traversal")
        result = file_operations([request, {"op": "read", "path": str(self.path)}], progress)
        self.assertEqual(result["status"], "failed")
        self.assertEqual(len(result["results"]), 1)
        self.assertIn("cancelled", result["results"][0]["error"])

    def test_recursive_junction_is_not_followed(self):
        import os
        if os.name != "nt":
            self.skipTest("Windows junction test")
        import subprocess
        with tempfile.TemporaryDirectory(prefix="loom-outside-") as outside:
            Path(outside, "secret.py").write_text("secret")
            link = self.root / "linked"
            subprocess.run(["cmd", "/c", "mklink", "/J", str(link), outside],
                           check=True, capture_output=True)
            try:
                result = self.run_ops({"op": "find", "path": str(self.root)})["results"][0]
                self.assertEqual(result["matches"], [{"path": str(self.path)}])
                self.assertEqual(result["skipped"]["links"], 1)
                self.assertEqual(self.run_ops({"op": "find", "path": str(link)})["status"], "failed")
            finally:
                link.rmdir()

    def test_recursive_tools_are_available_read_only_and_legacy(self):
        tools = object.__new__(SceneTools)
        tools.permission_mode = "read_only"
        tools.tidy_scope = None
        tools.progress = lambda: None
        payload = json.dumps([{"op": "find", "path": str(self.root)}])
        self.assertEqual(tools.dispatch("loom_files", {"requests_json": payload})["status"], "completed")
        result = tools.dispatch("ko_scene", {"operation": "context", "path": "/",
                                            "query": "@files:" + json.dumps({"requests_json": payload})})
        self.assertEqual(result["status"], "completed")

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
