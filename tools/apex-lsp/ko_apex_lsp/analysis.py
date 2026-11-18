"""APEX Script documents: diagnostics, symbols, light type inference, cursor context.

APEX Script uses Python syntax and Houdini transpiles it from Python's ``ast``, so the
document is parsed with this interpreter's ``ast`` (the server runs on Houdini's own
Python). Positions in the LSP are UTF-16 code units; ``ast`` columns are UTF-8 bytes.
"""
import ast
import bisect
import re
from dataclasses import dataclass, field

from .catalog import single_type, split_top

# Constructs Houdini's APEX transpiler rejects (apex/graphscript.py ApexNodeVisitor),
# keyed by real ast class names, with Houdini's own messages. Severity 1 = error.
UNSUPPORTED = {
    ast.While: ('while', 'While is not supported in Apex Script.'),
    ast.ClassDef: ('class', 'Construction of classes is not supported in Apex Script.'),
    ast.Try: ('try', 'Exception handling is not supported in Apex Script.'),
    ast.With: ('with', 'With is not supported in Apex Script.'),
    ast.Match: ('match', 'Matching is not supported in Apex Script.'),
    ast.Global: ('global', 'Global is not supported in Apex Script.'),
    ast.Yield: ('yield', 'Yield is not supported in Apex Script.'),
    ast.Break: ('break', 'Break is not supported in Apex Script.'),
    ast.Continue: ('continue', 'Continue is not supported in Apex Script.'),
    ast.Assert: ('assert', 'Assert is not supported in Apex Script.'),
    ast.Delete: ('del', 'Delete is not supported in Apex Script.'),
}
# Houdini adds this line to saved snippet files for editor autocomplete and strips it on load.
AUTOCOMPLETE_IMPORT = 'from apexscript import *'

TOKEN = re.compile(r'''
    (?P<comment>\#[^\n]*)
  | (?P<string>(?:[rRbBuUfF]{1,2})?(?:\'\'\'[\s\S]*?(?:\'\'\'|\Z)|"""[\s\S]*?(?:"""|\Z)|'(?:\\.|[^'\\\n])*'?|"(?:\\.|[^"\\\n])*"?))
  | (?P<name>[A-Za-z_]\w*)
  | (?P<number>(?:\d[\d_]*(?:\.[\d_]*)?|\.\d[\d_]*)(?:[eE][+-]?\d+)?[jJ]?)
  | (?P<op>\*\*=?|//=?|->|[-+*/%@&|^<>=!]=|[^\s\w])
''', re.X)


@dataclass(frozen=True)
class Token:
    text: str
    start: int
    end: int
    kind: str


@dataclass(eq=False)
class Symbol:
    name: str
    kind: str              # 'function' | 'variable' | 'parameter'
    start: int             # offset of the defining name
    end: int
    type: str = ''
    detail: str = ''       # function signature
    doc: str = ''
    returns: str = ''
    params: list = field(default_factory=list)
    scope: object = None   # enclosing function Symbol, None for module level
    body: tuple = (0, 0)   # function body offsets
    children: list = field(default_factory=list)


class Text:
    """Offset/position conversions for one document revision."""

    def __init__(self, text):
        self.text = text
        self.lines = [0] + [m.end() for m in re.finditer('\n', text)]

    def line_text(self, line):
        start = self.lines[line]
        end = self.lines[line+1] if line+1 < len(self.lines) else len(self.text)
        return self.text[start:end].rstrip('\r\n')

    def offset(self, position):
        line = position['line']
        if line >= len(self.lines):
            return len(self.text)
        start = self.lines[max(0, line)]
        end = self.lines[line+1] if line+1 < len(self.lines) else len(self.text)
        units = position['character']
        for i in range(start, end):
            if units <= 0:
                return i
            units -= 2 if ord(self.text[i]) > 0xffff else 1
        return end

    def position(self, offset):
        line = bisect.bisect_right(self.lines, offset) - 1
        prefix = self.text[self.lines[line]:offset]
        return {'line': line, 'character': sum(2 if ord(c) > 0xffff else 1 for c in prefix)}

    def range(self, start, end):
        return {'start': self.position(start), 'end': self.position(end)}

    def ast_offset(self, lineno, col):
        """Offset of an ast (1-based line, UTF-8 byte column) location."""
        line = min(max(lineno-1, 0), len(self.lines)-1)
        raw = self.line_text(line).encode('utf-8')[:col]
        return self.lines[line] + len(raw.decode('utf-8', errors='ignore'))

    def node_range(self, node):
        start = self.ast_offset(node.lineno, node.col_offset)
        end_line = getattr(node, 'end_lineno', None) or node.lineno
        end_col = getattr(node, 'end_col_offset', None)
        end = self.ast_offset(end_line, end_col) if end_col is not None else start
        return start, max(end, start)


def tokenize(text, end=None):
    text = text if end is None else text[:end]
    return [Token(m.group(), m.start(), m.end(), m.lastgroup) for m in TOKEN.finditer(text)]


def annotation_type(node):
    """``Float``, ``Float[outputy]`` (renamed output port) and ``geo.IntersectCache``."""
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        base = annotation_type(node.value)
        return base + '::' + node.attr if base else ''
    if isinstance(node, ast.Subscript) and isinstance(node.value, ast.Name) and node.value.id not in ('tuple', 'list'):
        return node.value.id
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return ''


def unparse(node):
    try:
        return ast.unparse(node)
    except Exception:
        return ''


class Analysis:
    """One parse of a document (possibly a repaired copy, see ``analyze``)."""

    def __init__(self, text, tree, catalog):
        self.text = Text(text)
        self.catalog = catalog
        self.symbols = []      # every Symbol, definition order
        self.functions = []    # top-level function Symbols (children nested)
        self.tree = tree
        if tree is not None:
            self._collect(tree.body, None)

    # ----- symbol collection and inference -------------------------------------------
    def _name_range(self, node, name):
        """Range of the identifier ``name`` at the start of a def/arg node."""
        start = self.text.ast_offset(node.lineno, node.col_offset)
        found = self.text.text.find(name, start)
        return (found, found+len(name)) if found >= 0 else (start, start)

    def _define(self, name, kind, start, end, scope, type_name='', **extra):
        existing = next((s for s in self.symbols if s.name == name and s.scope is scope
                         and s.kind in ('variable', 'parameter')), None)
        if existing and kind == 'variable':
            # APEX Script is statically typed: the first assignment fixes the type.
            if not existing.type and type_name:
                existing.type = type_name
            return existing
        symbol = Symbol(name, kind, start, end, type_name, scope=scope, **extra)
        self.symbols.append(symbol)
        return symbol

    def _collect(self, body, scope):
        for node in body:
            self._statement(node, scope)

    def _statement(self, node, scope):
        if isinstance(node, ast.FunctionDef):
            start, end = self._name_range(node, node.name)
            # Skip "def" so the name, not the keyword, is the target.
            found = self.text.text.find(node.name, self.text.ast_offset(node.lineno, node.col_offset) + 3)
            if found >= 0:
                start, end = found, found+len(node.name)
            params = []
            args = node.args.posonlyargs + node.args.args + node.args.kwonlyargs
            defaults = [None]*(len(node.args.posonlyargs + node.args.args) - len(node.args.defaults)) + node.args.defaults \
                + node.args.kw_defaults
            for arg, default in zip(args, defaults):
                params.append({'name': arg.arg, 'type': annotation_type(arg.annotation) if arg.annotation else '',
                               'default': unparse(default) if default is not None else '', 'inplace': False})
            returns = unparse(node.returns) if node.returns else ''
            body_start, body_end = self.text.node_range(node)
            symbol = Symbol(node.name, 'function', start, end, '', scope=scope, params=params, returns=returns,
                            doc=ast.get_docstring(node) or '', body=(body_start, body_end),
                            detail='def ' + node.name + '(' + ', '.join(
                                p['name'] + (': ' + p['type'] if p['type'] else '') + ('=' + p['default'] if p['default'] else '')
                                for p in params) + ')' + (' -> ' + returns if returns else ''))
            self.symbols.append(symbol)
            (scope.children if scope else self.functions).append(symbol)
            for arg, param in zip(args, params):
                a_start, a_end = self._name_range(arg, arg.arg)
                inferred = param['type']
                if not inferred and param['default']:
                    default = defaults[params.index(param)]
                    inferred = self.infer(default, symbol) if default is not None else ''
                self._define(arg.arg, 'parameter', a_start, a_end, symbol, inferred)
            self._collect(node.body, symbol)
            return
        if isinstance(node, ast.Assign):
            value_type = self.infer(node.value, scope)
            for target in node.targets:
                self._targets(target, node.value, value_type, scope)
        elif isinstance(node, ast.AnnAssign):
            type_name = annotation_type(node.annotation)
            if isinstance(node.target, ast.Name):
                start, end = self.text.node_range(node.target)
                self._define(node.target.id, 'variable', start, end, scope, type_name)
        elif isinstance(node, ast.AugAssign):
            pass
        elif isinstance(node, ast.For):
            self._for_targets(node, scope)
            self._collect(node.body, scope)
            self._collect(node.orelse, scope)
        elif isinstance(node, ast.If):
            self._collect(node.body, scope)
            self._collect(node.orelse, scope)

    def _targets(self, target, value, value_type, scope):
        if isinstance(target, ast.Name):
            start, end = self.text.node_range(target)
            self._define(target.id, 'variable', start, end, scope, value_type)
        elif isinstance(target, (ast.Tuple, ast.List)):
            # x, y = BindInput(2.6, 'test') / x, y = test(...) -> tuple[...]
            types = self._tuple_types(value, scope)
            for i, element in enumerate(target.elts):
                self._targets(element, None, types[i] if i < len(types) else '', scope)

    def _tuple_types(self, value, scope):
        if isinstance(value, (ast.Tuple, ast.List)):
            return [self.infer(e, scope) for e in value.elts]
        if isinstance(value, ast.Call):
            callee = value.func.id if isinstance(value.func, ast.Name) else ''
            if callee == 'BindInput':
                return [self.infer(a, scope) for a in value.args] + [self.infer(k.value, scope) for k in value.keywords]
            entry = self.callable(value.func, scope)
            returns = (entry or {}).get('returns', '').strip()
            for opener, closer in (('tuple[', ']'), ('(', ')'), ('[', ']')):
                if returns.startswith(opener) and returns.endswith(closer):
                    # Float[outputy] names an output port; the type is Float.
                    return [single_type(re.sub(r'\[.*\]$', '', t)) for t in split_top(returns[len(opener):-1])]
        return []

    def _for_targets(self, node, scope):
        element = ''
        index_target = None
        iterator = node.iter
        if isinstance(iterator, ast.Call) and isinstance(iterator.func, ast.Name) and iterator.func.id == 'range':
            element = 'Int'
        elif isinstance(iterator, ast.Call) and isinstance(iterator.func, ast.Name) and iterator.func.id == 'enumerate':
            if isinstance(node.target, ast.Tuple) and len(node.target.elts) == 2:
                index_target = node.target.elts[0]
                container = self.infer(iterator.args[0], scope) if iterator.args else ''
                element = container[:-5] if container.endswith('Array') else ''
                self._targets(index_target, None, 'Int', scope)
                self._targets(node.target.elts[1], None, element, scope)
                return
        elif (isinstance(iterator, ast.Call) and isinstance(iterator.func, ast.Attribute)
              and iterator.func.attr in ('points', 'prims')):
            element = 'Int'
        else:
            container = self.infer(iterator, scope)
            element = container[:-5] if container.endswith('Array') else ''
        self._targets(node.target, None, element, scope)

    def lookup(self, name, scope, before=None):
        """Innermost definition of ``name`` visible from ``scope``."""
        while True:
            for symbol in self.symbols:
                if symbol.name == name and symbol.scope is scope and (before is None or symbol.start <= before
                                                                       or symbol.kind == 'function'):
                    return symbol
            if scope is None:
                return None
            scope = scope.scope

    def callable(self, func, scope):
        """Catalogue entry or user function (as a dict) for a call's ``func`` node."""
        if isinstance(func, ast.Name):
            user = self.lookup(func.id, scope)
            if user and user.kind == 'function':
                return {'name': user.name, 'returns': user.returns, 'params': user.params}
            return self.catalog.function(func.id)
        if isinstance(func, ast.Attribute):
            chain = []
            node = func
            while isinstance(node, ast.Attribute):
                chain.append(node.attr)
                node = node.value
            if isinstance(node, ast.Name) and node.id == 'apex':
                path = 'apex'
                for part in reversed(chain[1:]):
                    path = self.catalog.child(path, part) or ''
                return self.catalog.namespace_function(path, chain[0]) if path else None
            receiver = self.infer(func.value, scope)
            return self.catalog.method(receiver, func.attr) if receiver else None
        return None

    def infer(self, node, scope):
        """Best-effort APEX type name of an expression, or ''."""
        if node is None:
            return ''
        if isinstance(node, ast.Constant):
            value = node.value
            if isinstance(value, bool):
                return 'Bool'
            return {int: 'Int', float: 'Float', str: 'String'}.get(type(value), '')
        if isinstance(node, ast.JoinedStr):
            return 'String'
        if isinstance(node, ast.Dict):
            return 'Dict'
        if isinstance(node, ast.Tuple):
            count = len(node.elts)
            # Rows may be tuples or Vector variables: m = (v, (4, 5, 6), (7, 8, 9)).
            if count in (3, 4) and all((isinstance(e, ast.Tuple) and len(e.elts) == count)
                                       or self.infer(e, scope) == f'Vector{count}' for e in node.elts):
                return f'Matrix{count}'
            return {2: 'Vector2', 3: 'Vector3', 4: 'Vector4', 9: 'Matrix3', 16: 'Matrix4'}.get(count, '')
        if isinstance(node, ast.List):
            element = self.infer(node.elts[0], scope) if node.elts else ''
            return element + 'Array' if element and element + 'Array' in self.catalog.types else ''
        if isinstance(node, ast.Name):
            symbol = self.lookup(node.id, scope)
            return symbol.type if symbol and symbol.kind != 'function' else ''
        if isinstance(node, (ast.Compare, ast.BoolOp)) or (isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.Not)):
            return 'Bool'
        if isinstance(node, ast.BinOp):
            return self.infer(node.left, scope)
        if isinstance(node, ast.UnaryOp):
            return self.infer(node.operand, scope)
        if isinstance(node, ast.IfExp):
            return self.infer(node.body, scope) or self.infer(node.orelse, scope)
        if isinstance(node, ast.Subscript):
            base = self.infer(node.value, scope)
            if isinstance(node.slice, ast.Slice):
                return base
            if base.endswith('Array'):
                return base[:-5]
            if base.startswith('Vector'):
                return 'Float'
            if base.startswith('Matrix'):
                return 'Vector' + base[-1]
            return ''
        if isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Name):
                name = func.id
                if name in self.catalog.types:
                    return name
                if name in ('BindInput', '_portalias'):
                    values = node.args + [k.value for k in node.keywords]
                    return self.infer(values[0], scope) if len(values) == 1 else ''
                if name == 'len':
                    return 'Int'
            entry = self.callable(func, scope)
            if entry:
                returns = single_type(entry.get('returns', ''))
                if returns:
                    return returns
                # In-place methods with no other outputs chain on the receiver.
                if entry.get('inplace') and isinstance(func, ast.Attribute):
                    return self.infer(func.value, scope)
            return ''
        if isinstance(node, ast.Attribute):
            receiver = self.infer(node.value, scope)
            # graph.addNode(...).xform_out: node attributes are ports.
            return 'ApexPortID' if receiver == 'ApexNodeID' else ''
        return ''

    # ----- queries ----------------------------------------------------------------------
    def _within(self, function, offset):
        start, end = function.body
        if start <= offset <= end:
            return True
        if offset < start:
            return False
        # ast ends a function at its last statement; indented lines after it (where the
        # user is typing) still belong to it until a line at or left of the "def".
        column = start - self.text.lines[bisect.bisect_right(self.text.lines, start) - 1]
        text = self.text.text
        line_start = text.rfind('\n', 0, offset) + 1
        if offset - line_start <= column or text[line_start:offset].strip():
            prefix = text[line_start:offset]
            if len(prefix) - len(prefix.lstrip()) <= column:
                return False
        for line in text[end:line_start].split('\n')[1:]:
            if line.strip() and len(line) - len(line.lstrip()) <= column:
                return False
        return True

    def scope_at(self, offset):
        best = None
        for symbol in self.symbols:
            if symbol.kind == 'function' and self._within(symbol, offset):
                if best is None or symbol.body[0] >= best.body[0]:
                    best = symbol
        return best

    def visible(self, offset):
        """Symbols visible at ``offset`` (innermost first, unique names)."""
        seen, result = set(), []
        scope = self.scope_at(offset)
        while True:
            for symbol in self.symbols:
                if symbol.scope is scope and symbol.name not in seen:
                    seen.add(symbol.name)
                    result.append(symbol)
            if scope is None:
                return result
            scope = scope.scope

    def symbol_at(self, offset):
        return next((s for s in self.symbols if s.start <= offset <= s.end), None)


def diagnostics(text, tree, error):
    """Syntax errors plus the constructs Houdini's transpiler rejects."""
    view = Text(text)
    result = []
    if error is not None:
        line = max((error.lineno or 1) - 1, 0)
        line = min(line, len(view.lines)-1)
        # SyntaxError offsets are 1-based characters (not bytes).
        start = view.lines[line] + max((error.offset or 1) - 1, 0)
        end_line = (getattr(error, 'end_lineno', None) or error.lineno or 1) - 1
        end_offset = getattr(error, 'end_offset', None)
        end = view.lines[min(max(end_line, 0), len(view.lines)-1)] + max(end_offset - 1, 0) if end_offset else start
        start = min(start, len(text))
        end = min(max(end, start), len(text))
        if end == start:
            line_end = view.lines[line+1]-1 if line+1 < len(view.lines) else len(text)
            end = min(start+1, max(line_end, start))
        result.append({'range': view.range(start, end), 'severity': 1, 'source': 'apex-script',
                       'message': error.msg or 'Invalid syntax'})
        return result
    for node in ast.walk(tree):
        rule = UNSUPPORTED.get(type(node))
        if rule:
            keyword, message = rule
            start = view.ast_offset(node.lineno, node.col_offset)
            result.append({'range': view.range(start, start+len(keyword)), 'severity': 1,
                           'source': 'apex-script', 'message': message})
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            start, end = view.node_range(node)
            if text[start:end].strip() == AUTOCOMPLETE_IMPORT:
                continue
            result.append({'range': view.range(start, end), 'severity': 2, 'source': 'apex-script',
                           'message': 'Imports are not supported in Apex Script.'})
    return result


def parse(text):
    """(tree, error) using this interpreter's parser."""
    try:
        return ast.parse(text), None
    except SyntaxError as error:
        return None, error
    except (ValueError, RecursionError, MemoryError) as error:
        failure = SyntaxError(str(error))
        failure.lineno, failure.offset = 1, 1
        return None, failure


def repaired(text, error):
    """(text, tree) with the failing line replaced by ``pass`` (symbols while typing).

    Only that one line changes, so (line, character) positions elsewhere still match the
    original document; absolute offsets do not, so callers convert through positions.
    """
    if error is None or not error.lineno:
        return None
    lines = text.split('\n')
    index = error.lineno - 1
    for candidate in (index, index - 1):
        if 0 <= candidate < len(lines):
            copy = list(lines)
            indent = re.match(r'[ \t]*', copy[candidate]).group()
            # Keep a following indented block valid after a broken "def ...:" or "for ...:".
            following = next((l for l in copy[candidate+1:] if l.strip()), '')
            deeper = len(re.match(r'[ \t]*', following).group()) > len(indent)
            copy[candidate] = indent + ('if True:' if deeper else 'pass')
            source = '\n'.join(copy)
            try:
                return source, ast.parse(source)
            except (SyntaxError, ValueError, RecursionError, MemoryError):
                continue
    return None
