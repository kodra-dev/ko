"""hou.pyi generation and Python workspace setup. Offline; no Houdini, Zed or provider.

Set KO_HOU_TEST_HFS (or KO_VEX_TEST_HFS) to a Houdini install to also check the
real hou.py.
"""
from pathlib import Path
import ast
import json
import os
import queue
import sys
import tempfile
import textwrap
import threading
import time
import unittest
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python3.13libs"))
import ko_hou_stubs as stubs
import ko_zed as zed

SOURCE = textwrap.dedent('''
    """hou docs"""
    import _hou

    class Error(Exception):
        pass

    class ObjectWasDeleted(Error):
        pass

    class EnumValue(object):
        def name(self) -> "std::string":
            return _hou.EnumValue_name(self)

    class Vector3(object):
        def length(self) -> "double":
            return 0

    class NetworkMovableItem(object):
        def __init__(self, *args, **kwargs):
            raise AttributeError("No constructor defined")

    class Node(NetworkMovableItem):
        """A node."""
        def children(self) -> "std::vector< HOM_ElemPtr< HOM_Node >,std::allocator< HOM_ElemPtr< HOM_Node > > >":
            """Child nodes."""
            return _hou.Node_children(self)
        def setInput(self, index: "int", item: "NetworkMovableItem", output: "int"=0) -> "void":
            return None
        def parm(self, name: "char const *") -> "HOM_Parm *":
            return None
        def type(self) -> "HOM_NodeType *":
            return None
        def str(self) -> "std::string":
            return ""

    class SopNode(Node):
        def geometry(self) -> "HOM_Geometry *":
            return None

    class ObjNode(Node):
        def worldTransform(self) -> "HOM_Matrix4":
            return None

    class Geometry(object):
        def points(self) -> "_PointTuple":
            return ()
        def setPointFloatAttribValues(self, name: "char const *", values: "std::vector< double,std::allocator< double > > const &") -> "void":
            return None
        def attribs(self) -> "std::map< std::string,hboost::any,std::less< std::string > >":
            return {}

    class Point(object):
        def position(self) -> "HOM_Vector3":
            return None
        def setPosition(self, value: "_DoubleTuple") -> "void":
            return None

    class Parm(object):
        def eval(self) -> "PyObject *":
            return None

    class parmTemplateType(object):
        pass
    parmTemplateType.String = _hou.cvar.parmTemplateType_String

    class hipFile(object):
        def path(self) -> "std::string":
            return ""

    def pwd() -> "HOM_Node *":
        """Current node."""
        return _hou.pwd()

    def node(path: "char const *") -> "HOM_Node *":
        return _hou.node(path)

    def __hipFile() -> "HOM_hipFile &":
        return _hou.__hipFile()

    def __createTopLevelFunc(name):
        return None

    expandString = __createTopLevelFunc("expandString")

    def __finishImport():
        global nodeType, data, webServer
        import houpythonportion, hwebserver
        nodeType = _hou.nodeType
        data = houpythonportion.data
        webServer = hwebserver
''')


def load(text):
    """Parse a stub and return its top-level classes and functions."""
    tree = ast.parse(text)
    return ({n.name: n for n in tree.body if isinstance(n, ast.ClassDef)},
            {n.name: n for n in tree.body if isinstance(n, ast.FunctionDef)}, tree)


class TranslatorTests(unittest.TestCase):
    def setUp(self):
        self.t = stubs.TypeTranslator({"Node", "Vector3", "Parm", "Matrix4", "hipFile"},
                                      {"hipFile": "_hipFile"}, {"Node": "_AnyNode"})

    def test_scalars_and_strings(self):
        self.assertEqual(self.t.translate("void"), "None")
        self.assertEqual(self.t.translate("char const *", True), "str")
        self.assertEqual(self.t.translate("std::string const &"), "str")
        self.assertEqual(self.t.translate("int64"), "int")
        self.assertEqual(self.t.translate("HOM_BinaryString"), "bytes")
        self.assertEqual(self.t.translate("hboost::any"), "Any")

    def test_containers_and_positions(self):
        vector = "std::vector< std::string,std::allocator< std::string > >"
        self.assertEqual(self.t.translate(vector), "tuple[str, ...]")
        self.assertEqual(self.t.translate(vector, True), "Sequence[str]")
        floats = "std::vector< double,std::allocator< double > > const &"
        self.assertEqual(self.t.translate(floats, True), "Sequence[float] | Vector3")
        self.assertEqual(self.t.translate("_DoubleTuple", True), "Sequence[float] | Vector3")
        self.assertEqual(self.t.translate("_StringTuple"), "tuple[str, ...]")
        mapping = "std::map< std::string,hboost::any,std::less< std::string > >"
        self.assertEqual(self.t.translate(mapping), "dict[str, Any]")
        self.assertEqual(self.t.translate("std::vector< int >::size_type"), "int")
        self.assertEqual(self.t.translate("std::vector< double >::value_type const &"), "float")
        self.assertEqual(self.t.translate("HOM_IterableList< std::vector< HOM_ElemPtr< HOM_Parm > > > *"),
                         "tuple[tuple[Parm, ...], ...]")

    def test_hou_classes(self):
        self.assertEqual(self.t.translate("HOM_Node *"), "_AnyNode")       # return: any node subclass
        self.assertEqual(self.t.translate("HOM_Node *", True), "Node | None")
        self.assertEqual(self.t.translate("Node", True), "Node | None")    # SWIG's Python-name spelling
        self.assertEqual(self.t.translate("HOM_Vector3 const &", True), "Vector3")  # value type
        self.assertEqual(self.t.translate("HOM_hipFile &"), "_hipFile")
        self.assertEqual(self.t.translate("HOM_Unknown *"), "Any")
        self.assertEqual(self.t.translate("_NodeTuple"), "tuple[_AnyNode, ...]")
        self.assertEqual(self.t.translate("std::vector< broken"), "Any")


class GenerateTests(unittest.TestCase):
    def setUp(self):
        self.text = stubs.generate(SOURCE, "22.0.test")
        self.classes, self.functions, self.tree = load(self.text)

    def method(self, cls, name):
        return next(n for n in self.classes[cls].body if isinstance(n, ast.FunctionDef) and n.name == name)

    def test_output_is_valid_python_and_keeps_docstrings(self):
        compile(self.text, "hou.pyi", "exec")
        self.assertEqual(ast.get_docstring(self.functions["pwd"]), "Current node.")
        self.assertEqual(ast.get_docstring(self.method("Node", "children")), "Child nodes.")
        self.assertIn("22.0.test", self.text.splitlines()[0])

    def test_node_returns_have_subclass_members(self):
        self.assertEqual(ast.unparse(self.functions["pwd"].returns), "_AnyNode")
        any_node = [ast.unparse(base) for base in self.classes["_AnyNode"].bases]
        self.assertEqual(any_node[:2], ["SopNode", "ObjNode"])  # SopNode first: geometry() wins
        self.assertEqual(ast.unparse(self.method("Node", "children").returns), "tuple[_AnyNode, ...]")
        # Classes with subclasses tolerate subclass-only members instead of erroring.
        self.assertTrue(any(isinstance(n, ast.FunctionDef) and n.name == "__getattr__"
                            for n in self.classes["Node"].body))
        self.assertFalse(any(isinstance(n, ast.FunctionDef) and n.name == "__getattr__"
                             for n in self.classes["SopNode"].body))

    def test_signatures(self):
        set_input = ast.unparse(self.method("Node", "setInput").args)
        self.assertEqual(set_input, "self, index: int, item: NetworkMovableItem | None, output: int=...")
        self.assertEqual(ast.unparse(self.method("Point", "setPosition").args.args[1].annotation),
                         "Sequence[float] | Vector3")
        self.assertEqual(ast.unparse(self.method("Geometry", "points").returns), "tuple[Point, ...]")
        self.assertEqual(ast.unparse(self.method("Parm", "eval").returns), "Any")
        # A member named "str" shadows the builtin inside Node's body.
        self.assertEqual(ast.unparse(self.method("Node", "str").returns), "builtins.str")
        # Constructor-less SWIG proxies do not keep a raising __init__.
        self.assertFalse(any(isinstance(n, ast.FunctionDef) and n.name == "__init__"
                             for n in self.classes["NetworkMovableItem"].body))

    def test_enums_singletons_exceptions_and_aliases(self):
        self.assertIn("    String: EnumValue", self.text)
        self.assertIn("_hipFile", self.classes)
        self.assertNotIn("hipFile", self.classes)
        self.assertIn("hipFile: _hipFile", self.text)
        self.assertEqual([ast.unparse(b) for b in self.classes["Error"].bases], ["Exception"])
        self.assertEqual([ast.unparse(b) for b in self.classes["ObjectWasDeleted"].bases], ["Error"])
        self.assertIn("def expandString(*args: Any, **kwargs: Any) -> Any: ...", self.text)
        self.assertNotIn("__createTopLevelFunc", self.text)
        self.assertNotIn("__hipFile", self.text)
        # Runtime-assigned names: functions from _hou, submodules/values otherwise.
        self.assertIn("def nodeType(*args: Any, **kwargs: Any) -> Any: ...", self.text)
        self.assertIn("\ndata: Any", self.text)
        self.assertIn("\nwebServer: Any", self.text)
        self.assertIn("\nsession: Any", self.text)

    def test_python_portion_methods_and_functions(self):
        with tempfile.TemporaryDirectory() as temp:
            portion = Path(temp) / "houpythonportion"
            (portion / "qt").mkdir(parents=True)
            (portion / "__init__.py").write_text(
                'def expandString(str_value):\n    """Expand."""\n    return str_value\n'
                'def _private():\n    pass\n', encoding="utf-8")
            (portion / "hipFile.py").write_text(
                "from houpythonportion.__utils__ import _addMethod\nimport hou\n"
                "@_addMethod(hou.hipFile)\ndef addEventCallback(self, callback):\n    pass\n"
                "@_addMethod(hou.Node)\ndef children(self):\n    pass\n", encoding="utf-8")
            (portion / "qt" / "__init__.py").write_text(
                "import hou\n@_addStaticMethod(hou.Node)\ndef fromData(data, *, strict=False):\n    pass\n",
                encoding="utf-8")
            (portion / "broken.py").write_text("def (:\n", encoding="utf-8")
            text = stubs.generate(SOURCE, "", stubs.python_portion(portion))
        classes, functions, _ = load(text)
        hip = {n.name: n for n in classes["_hipFile"].body if isinstance(n, ast.FunctionDef)}
        self.assertEqual(ast.unparse(hip["addEventCallback"].args), "self, callback: Any")
        node = [n for n in classes["Node"].body if isinstance(n, ast.FunctionDef)]
        # SWIG's own typed method wins over a same-named Python-portion method.
        self.assertEqual(sum(n.name == "children" for n in node), 1)
        from_data = next(n for n in node if n.name == "fromData")
        self.assertEqual([ast.unparse(d) for d in from_data.decorator_list], ["staticmethod"])
        self.assertEqual(ast.get_docstring(functions["expandString"]), "Expand.")
        self.assertNotIn("_private", text)


class WorkspaceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        base = Path(self.temp.name)
        self.root = base / "Houdini-Zed-01"
        self.root.mkdir()
        self.support = base / "repo/editors/houdini-python"
        self.hou = base / "hfs/python3.13libs/hou.py"
        self.hou.parent.mkdir(parents=True)
        self.hou.write_text(SOURCE, encoding="utf-8")
        self.settings = self.root / ".zed/settings.json"

    def prepare(self, paths=("C:\\HFS\\houdini\\python3.13libs",), version="22.0.466"):
        return stubs.prepare_python_workspace(self.root, self.hou, list(paths), version, self.support)

    def test_stub_goes_in_a_folder_named_for_the_houdini_version(self):
        self.assertEqual(self.prepare(), {"stub": "written", "settings": "written"})
        stub = self.support / "stubs/22.0.466/hou.pyi"
        self.assertTrue(stub.is_file())
        self.assertEqual(stub.read_text(encoding="utf-8").splitlines()[0], stubs.header("22.0.466"))
        self.assertIn("Houdini 22.0.466", stubs.header("22.0.466"))
        self.assertEqual(self.prepare(), {"stub": "unchanged", "settings": "unchanged"})
        # Another Houdini version gets its own folder; the first is untouched.
        self.prepare(version="22.5.100")
        self.assertTrue((self.support / "stubs/22.5.100/hou.pyi").is_file())
        self.assertTrue(stub.is_file())
        for bad in ("", "../x", "22.0", "22.0.466/../.."):
            with self.assertRaises(ValueError):
                stubs.stub_dir(bad, self.support)

    def test_committed_stub_is_reused_unless_generator_changed(self):
        stub = self.support / "stubs/22.0.466/hou.pyi"
        stub.parent.mkdir(parents=True)
        stub.write_text(stubs.header("22.0.466") + "\r\n# committed\r\n", encoding="utf-8", newline="")
        self.assertEqual(self.prepare()["stub"], "unchanged")  # CRLF checkouts still match.
        self.assertIn("# committed", stub.read_text(encoding="utf-8"))
        stub.write_text("# Generated by ko_hou_stubs v0 from Houdini 22.0.466. Do not edit.\n", encoding="utf-8")
        self.assertEqual(self.prepare()["stub"], "written")
        self.assertIn("class SopNode", stub.read_text(encoding="utf-8"))

    def test_settings_point_servers_at_support_files(self):
        self.prepare()
        settings = json.loads(self.settings.read_text(encoding="utf-8"))
        self.assertEqual(settings["preview_tabs"], {"enabled": False})
        analysis = settings["lsp"]["basedpyright"]["settings"]["basedpyright.analysis"]
        support = str(self.support).replace("\\", "/")
        self.assertEqual(analysis["stubPath"], support + "/stubs/22.0.466")
        self.assertEqual(analysis["extraPaths"], [support + "/builtins", "C:/HFS/houdini/python3.13libs"])
        self.assertEqual(analysis["typeCheckingMode"], "standard")
        ruff = settings["lsp"]["ruff"]["initialization_options"]["settings"]
        self.assertEqual(ruff["configuration"], support + "/ruff.toml")
        # Nothing is written into the workspace besides Zed's own settings folder.
        self.assertEqual([p.name for p in self.root.iterdir()], [".zed"])

    def test_existing_settings_are_merged_not_replaced(self):
        self.settings.parent.mkdir()
        self.settings.write_text('{"preview_tabs": {"enabled": false}}\n', encoding="utf-8")  # Old slot.
        self.assertEqual(self.prepare()["settings"], "written")
        self.assertIn("basedpyright", self.settings.read_text(encoding="utf-8"))
        # Your own settings and choices survive; Ko's paths follow a Houdini upgrade.
        settings = json.loads(self.settings.read_text(encoding="utf-8"))
        analysis = settings["lsp"]["basedpyright"]["settings"]["basedpyright.analysis"]
        analysis["typeCheckingMode"] = "strict"
        analysis["extraPaths"].append("D:/my/own/library")
        settings["theme"] = "One Dark"
        settings["preview_tabs"]["enabled"] = True
        settings["lsp"]["rust-analyzer"] = {"binary": {"path": "x"}}
        self.settings.write_text(json.dumps(settings), encoding="utf-8")
        self.assertEqual(self.prepare(["E:/HFS/python3.13libs"], version="22.5.100")["settings"], "written")
        merged = json.loads(self.settings.read_text(encoding="utf-8"))
        analysis = merged["lsp"]["basedpyright"]["settings"]["basedpyright.analysis"]
        self.assertTrue(analysis["stubPath"].endswith("/stubs/22.5.100"))
        self.assertEqual(analysis["typeCheckingMode"], "strict")
        support = str(self.support).replace("\\", "/")
        # The old Houdini path is replaced; your own entry stays after Ko's.
        self.assertEqual(analysis["extraPaths"], [support + "/builtins", "E:/HFS/python3.13libs",
                                                  "D:/my/own/library"])
        self.assertEqual(merged["theme"], "One Dark")
        self.assertEqual(merged["preview_tabs"], {"enabled": True})
        self.assertEqual(merged["lsp"]["rust-analyzer"], {"binary": {"path": "x"}})
        self.assertEqual(self.prepare(["E:/HFS/python3.13libs"], version="22.5.100")["settings"], "unchanged")

    def test_comments_are_backed_up_and_invalid_json_is_left_alone(self):
        self.settings.parent.mkdir()
        commented = ('// my notes\n{\n  /* block */ "theme": "One Dark", // trailing\n'
                     '  "url": "http://example.com//not-a-comment",\n}\n')
        self.settings.write_text(commented, encoding="utf-8")
        self.assertEqual(self.prepare()["settings"], "written")
        merged = json.loads(self.settings.read_text(encoding="utf-8"))
        self.assertEqual(merged["theme"], "One Dark")
        self.assertEqual(merged["url"], "http://example.com//not-a-comment")
        backups = list(self.settings.parent.glob("settings.json.backup-*"))
        self.assertEqual([b.read_text(encoding="utf-8") for b in backups], [commented])
        self.assertEqual(self.prepare()["settings"], "unchanged")  # No comments now, no new backup.
        self.assertEqual(len(list(self.settings.parent.glob("settings.json.backup-*"))), 1)

        broken = '{"theme": '
        self.settings.write_text(broken, encoding="utf-8")
        self.assertTrue(self.prepare()["settings"].startswith("not updated"))
        self.assertEqual(self.settings.read_text(encoding="utf-8"), broken)
        errors = queue.Queue()
        inputs = {"hou_source": str(self.hou), "library_paths": [], "houdini_version": "22.0.466",
                  "support": self.support}
        zed.start_python_setup(self.root, errors, inputs).join(30)
        self.assertIn("not valid JSON", errors.get_nowait())  # Reported in Houdini's status bar.

    def test_bridge_setup_runs_in_background_and_reports_errors(self):
        errors = queue.Queue()
        inputs = {"hou_source": str(self.hou), "library_paths": [], "houdini_version": "22.0.466",
                  "support": self.support}
        thread = zed.start_python_setup(self.root, errors, inputs)
        thread.join(30)
        self.assertFalse(thread.is_alive())
        self.assertTrue(errors.empty())
        self.assertTrue(self.settings.is_file())

        thread = zed.start_python_setup(self.root, errors, dict(inputs, houdini_version="unknown"))
        thread.join(30)
        self.assertIn("Python type stubs unavailable", errors.get_nowait())

    def test_first_open_waits_for_workspace_setup(self):
        release, calls = threading.Event(), []
        setup = threading.Thread(target=release.wait, args=(5,))
        setup.start()
        launcher = zed.Launcher(self.root, "fake-zed", exists=lambda marker: True,
                                run=lambda command, **kw: calls.append(command) or SimpleNamespace(returncode=0))
        self.addCleanup(launcher.close)
        launcher.ready = setup
        launcher.open(self.root / "a.py")
        setup.join(0.3)
        self.assertEqual(calls, [])  # Still waiting for the stubs/settings.
        release.set()
        setup.join(5)
        deadline = time.monotonic() + 5
        while not calls and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertEqual(len(calls), 1)
        self.assertIsNone(launcher.ready)

    def test_bridge_setup_input_failure_does_not_raise(self):
        errors = queue.Queue()
        sys.modules.pop("hou", None)
        try:
            self.assertIsNone(zed.start_python_setup(self.root, errors))  # No hou outside Houdini.
        finally:
            sys.modules.pop("hou", None)
        self.assertIn("Python type stubs unavailable", errors.get_nowait())


class ProjectTests(unittest.TestCase):
    """Folders opened in Zed directly, such as this repository."""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        base = Path(self.temp.name)
        self.hfs = base / "Houdini 22.0.466"
        for folder in ("houdini/python3.13libs/houpythonportion", "packages/apex/python3.13libs",
                       "packages/ocio", "python313/lib/site-packages", "python313/lib/site-packages-forced"):
            (self.hfs / folder).mkdir(parents=True)
        (self.hfs / "houdini/python3.13libs/hou.py").write_text(SOURCE, encoding="utf-8")
        self.project = base / "my-tools"
        (self.project / "python3.13libs").mkdir(parents=True)
        self.support = base / "repo/editors/houdini-python"

    def analysis(self):
        settings = json.loads((self.project / ".zed/settings.json").read_text(encoding="utf-8"))
        return settings, settings["lsp"]["basedpyright"]["settings"]["basedpyright.analysis"]

    def test_library_paths_found_from_an_install(self):
        names = [Path(p).relative_to(self.temp.name).as_posix()
                 for p in stubs.hfs_library_paths(self.hfs, self.project)]
        self.assertEqual(names, ["my-tools/python3.13libs",
                                 "Houdini 22.0.466/houdini/python3.13libs",
                                 "Houdini 22.0.466/packages/apex/python3.13libs",
                                 "Houdini 22.0.466/python313/lib/site-packages-forced",
                                 "Houdini 22.0.466/python313/lib/site-packages"])
        with self.assertRaises(ValueError):
            stubs.hfs_library_paths(self.project)

    def test_module_project_keeps_hou_imports_required(self):
        result = stubs.prepare_project(self.project, self.hfs, support=self.support)
        self.assertEqual(result, {"stub": "written", "settings": "written"})
        settings, analysis = self.analysis()
        self.assertTrue(analysis["stubPath"].endswith("/stubs/22.0.466"))  # Version from the folder name.
        self.assertFalse(any(p.endswith("/builtins") for p in analysis["extraPaths"]))
        self.assertNotIn("ruff", settings["lsp"])  # The project's own Ruff config stays in charge.
        self.assertNotIn("preview_tabs", settings)

    def test_snippet_folder_can_use_implicit_names(self):
        stubs.prepare_project(self.project, self.hfs, implicit_names=True, support=self.support)
        settings, analysis = self.analysis()
        self.assertTrue(analysis["extraPaths"][0].endswith("/builtins"))
        self.assertIn("ruff", settings["lsp"])

    def test_command_line(self):
        from contextlib import redirect_stdout
        import io
        from unittest.mock import patch
        output = io.StringIO()
        with redirect_stdout(output):
            self.assertEqual(stubs.main([str(self.hfs), "--project", str(self.project)], self.support), 0)
        self.assertIn("settings.json: written", output.getvalue())
        with self.assertRaises(SystemExit), patch("sys.stderr", io.StringIO()):
            stubs.main([str(self.project)], self.support)  # Not a Houdini install.


class CommittedSupportTests(unittest.TestCase):
    """The files in editors/houdini-python/ that the Zed settings point at."""

    def test_builtins_and_ruff_config(self):
        import tomllib
        builtins = (stubs.SUPPORT_DIR / "builtins/__builtins__.pyi").read_text(encoding="utf-8")
        compile(builtins, "__builtins__.pyi", "exec")
        self.assertIn("import hou as hou", builtins)
        self.assertIn("kwargs: dict[str, Any]", builtins)
        ruff = tomllib.loads((stubs.SUPPORT_DIR / "ruff.toml").read_text(encoding="utf-8"))
        self.assertEqual(ruff["builtins"], ["hou", "kwargs"])

    def test_committed_stubs_match_their_folder_and_generator(self):
        folders = sorted((stubs.SUPPORT_DIR / "stubs").iterdir())
        self.assertTrue(folders, "no committed hou stubs")
        for folder in folders:
            with self.subTest(version=folder.name):
                stubs.stub_dir(folder.name)  # Folder name is a Houdini version.
                with (folder / "hou.pyi").open(encoding="utf-8") as stream:
                    first = stream.readline().rstrip("\r\n")
                self.assertEqual(first, stubs.header(folder.name),
                                 "regenerate with: python python3.13libs/ko_hou_stubs.py <HFS>")


@unittest.skipUnless(os.environ.get("KO_HOU_TEST_HFS") or os.environ.get("KO_VEX_TEST_HFS"),
                     "set KO_HOU_TEST_HFS to a Houdini install")
class RealHouTests(unittest.TestCase):
    def test_real_hou_stub(self):
        hfs = Path(os.environ.get("KO_HOU_TEST_HFS") or os.environ["KO_VEX_TEST_HFS"])
        source = next((hfs / "houdini").glob("python3.*libs/hou.py"))
        text = stubs.generate(source.read_text(encoding="utf-8"), "real",
                              stubs.python_portion(source.parent / "houpythonportion"))
        classes, functions, _ = load(text)
        self.assertEqual(ast.unparse(functions["pwd"].returns), "_AnyNode")
        self.assertIn("SopNode", [ast.unparse(b) for b in classes["_AnyNode"].bases])
        geometry = next(n for n in classes["SopNode"].body if isinstance(n, ast.FunctionDef) and n.name == "geometry")
        self.assertEqual(ast.unparse(geometry.returns), "Geometry")
        hip = {n.name for n in classes["_hipFile"].body if isinstance(n, ast.FunctionDef)}
        self.assertIn("addEventCallback", hip)  # From houpythonportion.
        self.assertIn("ui: _ui", text)
        self.assertEqual([ast.unparse(b) for b in classes["Error"].bases], ["Exception"])


if __name__ == "__main__":
    unittest.main()
