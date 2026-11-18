"""Generate a typed ``hou.pyi`` stub from Houdini's SWIG-generated ``hou.py``.

``hou.py`` annotates everything with C++ type strings (``"HOM_Node *"``,
``"std::vector< std::string,... >"``), which Python type checkers cannot
resolve, so ``hou.pwd().geometry()`` is ``Unknown``. This module reads
``hou.py`` with ``ast`` (it never imports or executes it) and writes a stub
with real Python types and the original docstrings.

Stubs are committed per Houdini version in editors/houdini-python/stubs/<version>/.
The Zed bridge generates a missing one and points the workspace's basedpyright
and Ruff settings at that folder. To generate one by hand:

    python python3.13libs/ko_hou_stubs.py "C:/Program Files/Side Effects Software/Houdini 22.0.466"

Pure Python, no ``hou`` import: usable from Houdini, hython or plain Python.
"""
from pathlib import Path
import ast
import json
import os
import re
import shutil
import tempfile
import uuid

# Bump when the generated output changes, so cached stubs are regenerated.
GENERATOR_VERSION = 1
ANY_NODE = "_AnyNode"
# hou.session is the scene's Python module (Window > Python Source Editor).
RUNTIME_MODULES = ("session",)

_SCALARS = {
    "void": "None", "bool": "bool", "double": "float", "float": "float", "fpreal": "float",
    "fpreal32": "float", "fpreal64": "float", "int": "int", "int64": "int", "int32": "int",
    "exint": "int", "long": "int", "unsigned": "int", "unsigned int": "int", "size_t": "int",
    "ptrdiff_t": "int", "char": "str", "char const": "str", "string": "str", "UT_String": "str",
    "UT_StringHolder": "str", "HOM_BinaryString": "bytes", "PyObject": "Any",
    "InterpreterObject": "Any", "any": "Any", "void const": "Any", "SwigPyIterator": "Any",
}
_TUPLES = {
    "_StringTuple": "str", "_DoubleTuple": "float", "_FloatTuple": "float", "_IntTuple": "int",
    "_Int64Tuple": "int", "_BoolTuple": "bool", "_EnumTuple": "EnumValue",
}
_SEQUENCES = {"vector", "set", "list", "deque", "unordered_set"}
_MAPPINGS = {"map", "unordered_map"}
_SEQUENCES |= {"HOM_IterableList", "HOM_IterableIndexedList"}
_WRAPPERS = {"HOM_ElemPtr", "UT_SharedPtr", "shared_ptr"}
_VALUE_TYPES = {"Vector2", "Vector3", "Vector4", "Matrix2", "Matrix3", "Matrix4", "Quaternion",
                "Color", "BoundingBox", "BoundingRect", "EnumValue", "Ramp", "OrientedBoundingBox",
                "OrientedBoundingRect", "Keyframe", "StringKeyframe", "BaseKeyframe"}
_BUILTIN_BASES = {"Exception", "BaseException", "RuntimeError", "ValueError", "TypeError", "KeyError"}
_PREFIXES = ("std::", "hboost::", "boost::", "swig::")
_BUILTIN_TYPES = {"str", "int", "float", "bool", "bytes", "tuple", "dict", "list", "set", "type", "object"}


def _split_template(text):
    """'a<b,c<d>>::x' -> ('a', ['b', 'c<d>'], '::x'); no template -> (text, None, '')."""
    start = text.find("<")
    if start < 0:
        return text, None, ""
    depth, args, current = 0, [], start + 1
    for index in range(start, len(text)):
        char = text[index]
        if char == "<":
            depth += 1
        elif char == ">":
            depth -= 1
            if depth == 0:
                args.append(text[current:index])
                return text[:start], args, text[index + 1:]
        elif char == "," and depth == 1:
            args.append(text[current:index])
            current = index + 1
    return text, None, ""  # Unbalanced: treat as an opaque name.


class TypeTranslator:
    """Translate SWIG/C++ annotation strings to stub type expressions."""

    def __init__(self, classes, renamed=None, returned=None):
        self.classes = set(classes)
        self.renamed = dict(renamed or {})
        # Classes replaced by a more useful type in return position only.
        self.returned = dict(returned or {})

    def hou_class(self, name, parameter=True):
        name = name[4:] if name.startswith("HOM_") else name
        if name in self.renamed:
            return self.renamed[name]
        if not parameter and name in self.returned:
            return self.returned[name]
        # Private SWIG classes are not emitted, so they cannot be referenced.
        return name if name in self.classes and not name.startswith("_") else None

    def _sequence(self, item):
        """Parameter sequences: HOM also accepts its vector types where floats are expected."""
        if item == "float":
            vectors = [name for name in ("Vector2", "Vector3", "Vector4") if name in self.classes]
            return " | ".join(["Sequence[float]"] + vectors)
        return f"Sequence[{item}]"

    def translate(self, text, parameter=False):
        try:
            result = self._node(text, parameter)
        except RecursionError:
            return "Any"
        # SWIG maps None to a null object pointer, e.g. node.setInput(0, None).
        # Value types (vectors, matrices, enum values...) are passed by value.
        if parameter and result in self.classes and result not in _VALUE_TYPES:
            result += " | None"
        return result

    def _node(self, text, parameter):
        text = re.sub(r"\b(const|struct|class|typename)\b", " ", text)
        text = re.sub(r"\s+", " ", text.replace("*", " ").replace("&", " ")).strip()
        name, args, rest = _split_template(text)
        name = name.strip()
        for prefix in _PREFIXES:
            if name.startswith(prefix):
                name = name[len(prefix):]
        rest = rest.strip()
        if rest.startswith("::"):
            member = rest[2:].strip()
            if member in ("size_type", "difference_type"):
                return "int"
            if args and member == "value_type":
                return self._node(args[0], parameter) if name in _SEQUENCES else "Any"
            if args and member == "key_type":
                return self._node(args[0], parameter) if name in _MAPPINGS else "Any"
            if args and member == "mapped_type" and len(args) > 1:
                return self._node(args[1], parameter)
            return "Any"
        if args is not None:
            if name in _WRAPPERS:
                return self._node(args[0], parameter)
            if name in _SEQUENCES:
                item = self._node(args[0], parameter)
                return self._sequence(item) if parameter else f"tuple[{item}, ...]"
            if name in _MAPPINGS and len(args) >= 2:
                key, value = self._node(args[0], parameter), self._node(args[1], parameter)
                return f"Mapping[{key}, {value}]" if parameter else f"dict[{key}, {value}]"
            if name == "pair" and len(args) >= 2:
                return f"tuple[{self._node(args[0], parameter)}, {self._node(args[1], parameter)}]"
            if name == "optional":
                return f"{self._node(args[0], parameter)} | None"
            return "Any"
        if name in _SCALARS:
            return _SCALARS[name]
        if name in _TUPLES:
            item = _TUPLES[name]
            return self._sequence(item) if parameter else f"tuple[{item}, ...]"
        if name.startswith("_") and name.endswith("Tuple"):
            item = self.hou_class(name[1:-len("Tuple")], parameter) or "Any"
            return self._sequence(item) if parameter else f"tuple[{item}, ...]"
        if name == "HOM_EnumValue":
            return "EnumValue"
        return self.hou_class(name, parameter) or "Any"


def _annotation(node, translator, parameter):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return translator.translate(node.value, parameter)
    return "Any"


def _docstring(body, indent):
    doc = ast.get_docstring(body, clean=False) if body is not None else None
    if not doc:
        return []
    doc = doc.replace("\\", "\\\\").replace('"""', '\\"\\"\\"')
    return [f'{indent}"""{doc}"""']


def _signature(function, translator, shadowed, method):
    args = function.args
    positional = args.posonlyargs + args.args
    defaults = [None] * (len(positional) - len(args.defaults)) + list(args.defaults)
    parts = []
    for index, (arg, default) in enumerate(zip(positional, defaults)):
        if index == 0 and method and arg.arg in ("self", "cls"):
            parts.append(arg.arg)
            continue
        kind = _qualify(_annotation(arg.annotation, translator, True), shadowed)
        if isinstance(default, ast.Constant) and default.value is None and "None" not in kind.split(" | "):
            kind += " | None"
        parts.append(f"{arg.arg}: {kind}" + (" = ..." if default is not None else ""))
        if args.posonlyargs and arg is args.posonlyargs[-1]:
            parts.append("/")
    if args.vararg:
        parts.append(f"*{args.vararg.arg}: Any")
    elif args.kwonlyargs:
        parts.append("*")
    for arg, default in zip(args.kwonlyargs, args.kw_defaults):
        kind = _qualify(_annotation(arg.annotation, translator, True), shadowed)
        parts.append(f"{arg.arg}: {kind}" + (" = ..." if default is not None else ""))
    if args.kwarg:
        parts.append(f"**{args.kwarg.arg}: Any")
    returns = _qualify(_annotation(function.returns, translator, False), shadowed)
    if function.name == "__init__":
        returns = "None"
    return f"({', '.join(parts)}) -> {returns}"


def _qualify(expression, shadowed):
    """Use builtins.X where a class defines a member that shadows builtin X."""
    if not shadowed:
        return expression
    return re.sub(r"\b(" + "|".join(sorted(shadowed)) + r")\b(?!\.)",
                  lambda match: "builtins." + match.group(1), expression)


def _decorators(function):
    names = []
    for decorator in function.decorator_list:
        if isinstance(decorator, ast.Name) and decorator.id in ("staticmethod", "classmethod", "property"):
            names.append(decorator.id)
    return names


def _function_lines(function, translator, indent, shadowed=(), method=False):
    lines = [f"{indent}@{name}" for name in _decorators(function)]
    lines.append(f"{indent}def {function.name}{_signature(function, translator, shadowed, method)}:")
    doc = _docstring(function, indent + "    ")
    lines.extend(doc or [f"{indent}    ..."])
    if doc:
        lines.append(f"{indent}    ...")
    return lines


def _plain_function_lines(function, indent, method):
    """Unannotated Python-portion function: keep names/defaults, type as Any."""
    args = function.args
    positional = args.posonlyargs + args.args
    defaults = [None] * (len(positional) - len(args.defaults)) + list(args.defaults)
    parts = []
    for index, (arg, default) in enumerate(zip(positional, defaults)):
        if index == 0 and method:
            parts.append(arg.arg)
        else:
            parts.append(f"{arg.arg}: Any" + (" = ..." if default is not None else ""))
    if args.vararg:
        parts.append(f"*{args.vararg.arg}: Any")
    elif args.kwonlyargs:
        parts.append("*")
    for arg, default in zip(args.kwonlyargs, args.kw_defaults):
        parts.append(f"{arg.arg}: Any" + (" = ..." if default is not None else ""))
    if args.kwarg:
        parts.append(f"**{args.kwarg.arg}: Any")
    lines = [f"{indent}def {function.name}({', '.join(parts)}) -> Any:"]
    doc = _docstring(function, indent + "    ")
    return lines + doc + [f"{indent}    ..."]


def python_portion(directory):
    """Collect functions that houpythonportion attaches to hou at import time.

    Returns ({class name: [(kind, FunctionDef)]}, {module function name: FunctionDef}),
    found from ``@_addMethod(hou.X)``, ``@_addStaticMethod(hou.X)`` and
    ``@_addFunction(hou)`` decorators, plus plain top-level functions that hou.py
    re-exports through ``__createTopLevelFunc``.
    """
    methods, functions = {}, {}
    directory = Path(directory) if directory else None
    if not directory or not directory.is_dir():
        return methods, functions
    for path in sorted(directory.rglob("*.py")):
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"))
        except (OSError, SyntaxError, UnicodeDecodeError, ValueError):
            continue
        for node in tree.body:
            if not isinstance(node, ast.FunctionDef):
                continue
            if path.name == "__init__.py" and path.parent == directory and not node.name.startswith("_"):
                functions.setdefault(node.name, node)
            for decorator in node.decorator_list:
                if not (isinstance(decorator, ast.Call) and isinstance(decorator.func, ast.Name)
                        and decorator.args):
                    continue
                kind = decorator.func.id.lstrip("_")
                target = decorator.args[0]
                if kind in ("addMethod", "addStaticMethod") and isinstance(target, ast.Attribute) \
                        and isinstance(target.value, ast.Name) and target.value.id == "hou":
                    methods.setdefault(target.attr, []).append((kind, node))
                elif kind == "addFunction" and isinstance(target, ast.Name) and target.id == "hou":
                    functions[node.name] = node
    return methods, functions


def generate(source_text, version="", portion=None):
    """Return the text of ``hou.pyi`` for the given ``hou.py`` source.

    ``portion`` is the result of :func:`python_portion`, for the parts of hou
    written in Python.
    """
    portion_methods, portion_functions = portion or ({}, {})
    tree = ast.parse(source_text)
    classes = {node.name: node for node in tree.body if isinstance(node, ast.ClassDef)}
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)]

    # Singleton submodules (hou.hipFile, hou.ui ...) are created at import time by
    # private factories such as ``def __hipFile() -> "HOM_hipFile &"``. Their
    # classes have instance methods, so expose an instance, not the class.
    singletons = {}
    for function in functions:
        name = function.name
        if name.startswith("__") and not name.endswith("__") and name[2:] in classes:
            singletons[name[2:]] = "_" + name[2:]

    children = {}
    for node in classes.values():
        for base in node.bases:
            if isinstance(base, ast.Name) and base.id in classes:
                children.setdefault(base.id, []).append(node.name)
    subclassed = set(children)

    # hou.pwd()/hou.node() are declared to return Node, but HOM always returns
    # the most-derived type (a SopNode inside a Python SOP). A synthetic class
    # deriving from every concrete node class lets ``hou.pwd().geometry()``
    # complete and type-check. Used in return position only.
    def leaves(name):
        below = children.get(name, [])
        return [name] if not below else [leaf for child in below for leaf in leaves(child)]
    node_leaves = []
    if "Node" in classes:
        preferred = ("SopNode", "ObjNode", "LopNode", "CopNode", "Cop2Node", "DopNode", "RopNode")
        found = [leaf for leaf in leaves("Node") if leaf != "Node"]
        node_leaves = [leaf for leaf in preferred if leaf in found] + [
            leaf for leaf in found if leaf not in preferred]
    returned = {"Node": ANY_NODE, "OpNode": ANY_NODE} if node_leaves else {}
    translator = TypeTranslator(classes, singletons, returned)

    # Enumeration values are attached after their classes, e.g.
    # ``parmTemplateType.String = _hou.cvar.parmTemplateType_String``.
    enum_values = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target = node.targets[0]
            if (isinstance(target, ast.Attribute) and isinstance(target.value, ast.Name)
                    and target.value.id in classes and isinstance(node.value, ast.Attribute)
                    and isinstance(node.value.value, ast.Attribute) and node.value.value.attr == "cvar"):
                enum_values.setdefault(target.value.id, []).append(target.attr)

    out = [
        header(version),
        "# Types are translated from SWIG C++ annotations; unknown types are Any.",
        "import builtins",
        "from typing import Any, Mapping, Sequence",
        "",
    ]
    out.extend(_docstring(tree, ""))
    out.append("")

    for name, node in classes.items():
        if name.startswith("_") or name == "SwigPyIterator":
            continue
        members = [item for item in node.body if isinstance(item, ast.FunctionDef)]
        member_names = {item.name for item in members} | set(enum_values.get(name, ()))
        shadowed = member_names & _BUILTIN_TYPES
        bases = [base.id for base in node.bases if isinstance(base, ast.Name) and (
            base.id in _BUILTIN_BASES or (base.id in classes and not base.id.startswith("_")))]
        bases = [singletons.get(base, base) for base in bases]
        stub_name = singletons.get(name, name)
        out.append(f"class {stub_name}" + (f"({', '.join(bases)})" if bases else "") + ":")
        body = _docstring(node, "    ")
        for value in enum_values.get(name, ()):
            body.append(f"    {value}: EnumValue")
        for item in members:
            if item.name.startswith("_") and not (item.name.startswith("__") and item.name.endswith("__")):
                continue
            if item.name in ("__repr__", "__swig_destroy__"):
                continue
            # Abstract SWIG proxies just raise; keep them constructible in the stub.
            if item.name == "__init__" and len(item.body) <= 2 and any(
                    isinstance(stmt, ast.Raise) for stmt in item.body):
                continue
            body.extend(_function_lines(item, translator, "    ", shadowed, method=True))
        defined = {item.name for item in members}
        for kind, function in portion_methods.get(name, ()):
            if function.name in defined or function.name.startswith("_"):
                continue
            defined.add(function.name)
            if kind == "addStaticMethod":
                body.append("    @staticmethod")
            body.extend(_plain_function_lines(function, "    ", method=kind == "addMethod"))
        if name in subclassed and name != "EnumValue":
            # HOM always returns the most-derived type (hou.pwd() is a SopNode
            # inside a Python SOP), so subclass members must not be errors.
            body.append("    def __getattr__(self, name: builtins.str) -> Any: ...")
        out.extend(body or ["    ..."])
        out.append("")

    if node_leaves:
        # A few same-named methods differ between node classes (verb, selection);
        # checkers report that inside this stub only, never in user code.
        out.append(f"class {ANY_NODE}({', '.join(node_leaves)}):  # pyright: ignore")
        out.append('    """A node of any concrete type (stub-only helper; HOM returns the actual subclass)."""')
        out.append("")

    for stub_class in sorted(singletons):
        out.append(f"{stub_class}: {singletons[stub_class]}")
    out.append("")

    seen = set(classes) | set(singletons)
    for function in functions:
        if function.name.startswith("_") or function.name in seen:
            continue
        seen.add(function.name)
        out.extend(_function_lines(function, translator, ""))
        out.append("")

    # Top-level functions written in Python, re-exported by hou.py, e.g.
    # ``expandString = __createTopLevelFunc("expandString")``.
    exported = set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name) and isinstance(node.value, ast.Call)
                and isinstance(node.value.func, ast.Name) and node.value.func.id.endswith("createTopLevelFunc")):
            exported.add(node.targets[0].id)
    for name, function in portion_functions.items():
        if name in seen or (name not in exported and not any(
                isinstance(d, ast.Call) for d in function.decorator_list)):
            continue
        seen.add(name)
        out.extend(_plain_function_lines(function, "", method=False))
        out.append("")

    # Remaining top-level names, including those assigned while the module
    # finishes importing. ``nodeType = _hou.nodeType`` and createTopLevelFunc
    # results are functions; anything else (``data = houpythonportion.data``,
    # ``webServer = hwebserver``) is a submodule or value.
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            name = node.targets[0].id
            if name not in seen and not name.startswith("_") and not name[:1].isupper():
                seen.add(name)
                value = node.value
                function = (isinstance(value, ast.Attribute) and isinstance(value.value, ast.Name)
                            and value.value.id == "_hou") or (
                    isinstance(value, ast.Call) and isinstance(value.func, ast.Name)
                    and value.func.id.endswith("createTopLevelFunc"))
                out.append(f"def {name}(*args: Any, **kwargs: Any) -> Any: ..." if function else f"{name}: Any")
    # Added by Houdini at runtime rather than by hou.py.
    for name in RUNTIME_MODULES:
        if name not in seen:
            out.append(f"{name}: Any")
    out.append("")
    return "\n".join(out)


def header(version):
    return f"# Generated by ko_hou_stubs v{GENERATOR_VERSION} from Houdini {version or 'hou.py'}. Do not edit."


# Committed Python editor support: editors/houdini-python/ in this repository.
SUPPORT_DIR = Path(__file__).resolve().parents[1] / "editors" / "houdini-python"


def stub_dir(version, support=SUPPORT_DIR):
    """Folder holding hou.pyi for one Houdini version, e.g. stubs/22.0.466."""
    if not re.fullmatch(r"\d+\.\d+\.\d+(\.\d+)?", version or ""):
        raise ValueError(f"Not a Houdini version: {version!r}")
    return Path(support) / "stubs" / version


def _atomic_write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(dir=path.parent, prefix=path.name + ".", suffix=".tmp")
    try:
        with os.fdopen(handle, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(text)
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def ensure_stub(source, version, support=SUPPORT_DIR):
    """Generate stubs/<version>/hou.pyi from that Houdini's hou.py if it is missing.

    A committed stub is reused as long as it came from the current generator
    version (its first line says so). Returns (path, written).
    """
    destination = stub_dir(version, support) / "hou.pyi"
    try:
        with destination.open(encoding="utf-8") as stream:
            if stream.readline().rstrip("\r\n") == header(version):
                return destination, False
    except OSError:
        pass
    source = Path(source)
    portion = python_portion(source.parent / "houpythonportion")
    _atomic_write(destination, generate(source.read_text(encoding="utf-8"), version, portion))
    return destination, True


def _slash(path):
    return str(Path(path)).replace("\\", "/")


ANALYSIS = ("lsp", "basedpyright", "settings", "basedpyright.analysis")
RUFF = ("lsp", "ruff", "initialization_options", "settings")


def zed_settings(stubs, library_paths, support=SUPPORT_DIR, implicit_names=True):
    """What Ko merges into a Zed settings file: ({key path: value}, {key path: default}).

    Ko always sets the first group; the second only where the key is missing,
    so your own choices win. ``implicit_names`` adds ``hou``/``kwargs`` as
    builtins, which suits code stored in Houdini (snippets, callbacks, shelf
    tools) but would hide a missing ``import hou`` in a normal module.
    """
    extra_paths = [_slash(Path(support) / "builtins")] if implicit_names else []
    managed = {
        ANALYSIS + ("stubPath",): _slash(stubs),
        ANALYSIS + ("extraPaths",): extra_paths + [_slash(p) for p in library_paths],
    }
    if implicit_names:
        # Ruff does not read __builtins__.pyi; its config declares the same names.
        managed[RUFF + ("configuration",)] = _slash(Path(support) / "ruff.toml")
    defaults = {
        # "standard" avoids strict-mode noise on loosely typed Houdini code.
        ANALYSIS + ("typeCheckingMode",): "standard",
        ANALYSIS + ("diagnosticSeverityOverrides", "reportMissingModuleSource"): "none",
    }
    return managed, defaults


def hfs_library_paths(hfs, project=None):
    """Houdini's Python library folders, found from an install (outside Houdini).

    Inside Houdini the bridge uses sys.path instead, which also includes
    $HOUDINI_PATH packages; this covers HFS, its bundled packages, its
    site-packages (numpy, PySide6) and the project's own pythonX.Ylibs.
    """
    hfs = Path(hfs)
    libs = sorted((hfs / "houdini").glob("python3.*libs"))
    if not libs:
        raise ValueError(f"No houdini/python3.*libs under {hfs}")
    name = libs[-1].name                                   # e.g. python3.13libs
    interpreter = "python" + name[len("python"):-len("libs")].replace(".", "")  # python313
    paths = []
    if project and (Path(project) / name).is_dir():
        paths.append(Path(project) / name)
    paths.append(hfs / "houdini" / name)
    paths += sorted(p for p in (hfs / "packages").glob(f"*/{name}") if p.is_dir())
    paths += [p for p in (hfs / interpreter / "lib" / "site-packages-forced",
                          hfs / interpreter / "lib" / "site-packages") if p.is_dir()]
    return [str(p.resolve()) for p in paths]


def load_jsonc(text):
    """Parse Zed's JSON-with-comments. Returns (value, had_comments)."""
    out, index, comments, length = [], 0, False, len(text)
    while index < length:
        char = text[index]
        if char == '"':
            end = index + 1
            while end < length and text[end] != '"':
                end += 2 if text[end] == "\\" else 1
            out.append(text[index:end + 1])
            index = end + 1
        elif text.startswith("//", index):
            comments = True
            end = text.find("\n", index)
            index = length if end < 0 else end
        elif text.startswith("/*", index):
            comments = True
            end = text.find("*/", index + 2)
            index = length if end < 0 else end + 2
        else:
            out.append(char)
            index += 1
    # Zed also accepts trailing commas.
    cleaned = re.sub(r",(\s*[}\]])", r"\1", "".join(out))
    return (json.loads(cleaned) if cleaned.strip() else {}), comments


# Folder names of Houdini-provided library paths, i.e. the extraPaths Ko writes.
_LIBRARY_NAME = re.compile(r"python\d+\.\d+libs|site-packages(-[\w-]+)?|viewer_states|viewer_handles|builtins")


def _ko_path(entry):
    return bool(_LIBRARY_NAME.fullmatch(Path(str(entry)).name))


def merge_zed_settings(root, managed, defaults=None):
    """Merge Ko's keys into <root>/.zed/settings.json, keeping everything else.

    Library paths you add to extraPaths are kept after Ko's. Comments are lost
    when the file has to change, so a file with comments is first copied to
    settings.json.backup-<id>. Returns "written", "unchanged" or an error message.
    """
    path = Path(root) / ".zed" / "settings.json"
    try:
        text = path.read_text(encoding="utf-8-sig")
    except FileNotFoundError:
        text = ""
    try:
        current, comments = load_jsonc(text)
    except ValueError as error:
        return f"not updated: {path} is not valid JSON ({error})"
    if not isinstance(current, dict):
        return f"not updated: {path} is not a JSON object"
    merged = json.loads(json.dumps(current))  # Deep copy.

    def parent(keys):
        node = merged
        for key in keys[:-1]:
            if not isinstance(node.get(key), dict):
                node[key] = {}
            node = node[key]
        return node

    for keys, value in managed.items():
        node = parent(keys)
        if keys[-1] == "extraPaths" and isinstance(node.get("extraPaths"), list):
            value = value + [p for p in node["extraPaths"] if p not in value and not _ko_path(p)]
        node[keys[-1]] = value
    for keys, value in (defaults or {}).items():
        parent(keys).setdefault(keys[-1], value)
    if merged == current and text:
        return "unchanged"
    if comments:
        shutil.copy2(path, path.with_name(f"{path.name}.backup-{uuid.uuid4().hex[:8]}"))
    _atomic_write(path, json.dumps(merged, indent=2) + "\n")
    return "written"


def prepare_python_workspace(root, hou_source, library_paths, houdini_version, support=SUPPORT_DIR):
    """Make the bridge slot's Python servers understand hou. Returns {item: outcome}."""
    stubs, written = ensure_stub(hou_source, houdini_version, support)
    managed, defaults = zed_settings(stubs.parent, library_paths, support)
    defaults[("preview_tabs", "enabled")] = False
    return {"stub": "written" if written else "unchanged",
            "settings": merge_zed_settings(root, managed, defaults)}


def prepare_project(project, hfs, version=None, implicit_names=False, support=SUPPORT_DIR):
    """Set up any folder opened in Zed (e.g. this repo) for Houdini Python code."""
    source = next(Path(hfs, "houdini").glob("python3.*libs/hou.py"), None)
    if source is None:
        raise ValueError(f"No houdini/python3.*libs/hou.py under {hfs}")
    stubs, written = ensure_stub(source, version or _version_from_hfs(hfs), support)
    managed, defaults = zed_settings(stubs.parent, hfs_library_paths(hfs, project), support, implicit_names)
    return {"stub": "written" if written else "unchanged",
            "settings": merge_zed_settings(project, managed, defaults)}


def _version_from_hfs(hfs):
    match = re.search(r"(\d+\.\d+\.\d+)", Path(hfs).name)
    if not match:
        raise ValueError(f"Cannot read the Houdini version from {str(hfs)!r}; pass --version.")
    return match.group(1)


def main(argv=None, support=SUPPORT_DIR):
    import argparse
    parser = argparse.ArgumentParser(
        description="Generate editors/houdini-python/stubs/<version>/hou.pyi, and optionally "
                    "point a Zed project's Python language servers at it.")
    parser.add_argument("hfs", help="Houdini install folder, e.g. 'C:/Program Files/Side Effects Software/Houdini 22.0.466'")
    parser.add_argument("--version", help="Houdini version (default: from the folder name)")
    parser.add_argument("--project", help="also merge Ko's Python settings into <PROJECT>/.zed/settings.json")
    parser.add_argument("--implicit-hou", action="store_true",
                        help="treat hou/kwargs as builtins, for folders of Houdini snippets or callbacks")
    options = parser.parse_args(argv)
    try:
        if options.project:
            result = prepare_project(options.project, options.hfs, options.version, options.implicit_hou, support)
            print(f"hou.pyi: {result['stub']}; {Path(options.project) / '.zed/settings.json'}: {result['settings']}")
            return 0 if result["settings"] in ("written", "unchanged") else 1
        source = next(Path(options.hfs, "houdini").glob("python3.*libs/hou.py"), None)
        if source is None:
            raise ValueError(f"No houdini/python3.*libs/hou.py under {options.hfs}")
        path, written = ensure_stub(source, options.version or _version_from_hfs(options.hfs), support)
        print(("Wrote " if written else "Up to date: ") + str(path))
        return 0
    except ValueError as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())


