"""Offline local-help tool regression tests."""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python3.13libs"))
from loom.docs import inspect_docs
from loom.tools import SceneTools


class DocsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="loom-docs-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "nodes").mkdir()
        (self.root / "nodes/subdivide.txt").write_text("Subdivide\nDepth uses iterations\nThird line\n", encoding="utf-8")
        self.env = patch.dict(os.environ, {"LOOM_HOUDINI_DOCS": str(self.root)})
        self.env.start()
        self.addCleanup(self.env.stop)

    def test_search_then_read_through_scene_tool(self):
        tools = object.__new__(SceneTools)
        tools.progress = lambda: None
        result = tools.inspect("docs_search", "/", "subdivide", limit=10)
        self.assertEqual(result["matches"][0]["path"], "nodes/subdivide.txt")
        page = tools.inspect("docs_read", "nodes/subdivide.txt", offset=1, limit=1)
        self.assertEqual(page["text"], "Depth uses iterations")
        self.assertEqual(page["next_offset"], 2)
        self.assertIsNone(tools.inspect("docs_read", "nodes/subdivide.txt", offset=2)["next_offset"])

    def test_search_content_with_directory_scope(self):
        result = inspect_docs("docs_search", "nodes", "iterations")
        self.assertEqual(result["matches"][0]["line"], 2)

    def test_rejects_escape_and_non_doc_files(self):
        for operation in ("docs_read", "docs_search"):
            with self.assertRaises(ValueError):
                inspect_docs(operation, "../outside", "subdivide")
        (self.root / "script.py").write_text("secret", encoding="utf-8")
        with self.assertRaises(ValueError):
            inspect_docs("docs_read", "script.py")

    def test_missing_configured_root_is_actionable(self):
        with patch.dict(os.environ, {"LOOM_HOUDINI_DOCS": str(self.root / "missing")}):
            with self.assertRaisesRegex(ValueError, "LOOM_HOUDINI_DOCS"):
                inspect_docs("docs_search", query="subdivide")
