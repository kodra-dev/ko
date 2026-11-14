"""Compiler-gated local-variable refactoring. Never writes user's source files."""
from pathlib import Path
import json
import re
import tempfile

from .model import Document, KEYWORDS, uri_path


class RefactorError(ValueError):
    pass


class CompilerError(RefactorError):
    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = diagnostics


def compiler_diagnostics(message, doc, graph, paths, line_shift):
    diagnostics = []
    normalize = lambda path: str(path).replace('\\\\', '/').replace('\\', '/').casefold()
    lookup = {normalize(paths[d.uri]): d for d in graph}
    pattern = re.compile(r'^(.+?):(\d+):(\d+)(?:-(\d+))?:\s*(Error|Warning)\s*(\d+):\s*(.*)$')
    for line in message.splitlines():
        match = pattern.match(line.strip())
        if not match:
            continue
        target = lookup.get(normalize(match[1]))
        if target is None:
            continue
        row = int(match[2])-1-(line_shift if target is doc else 0)
        if not 0 <= row < len(target.lines):
            continue
        raw_line = target.text[target.lines[row]:].split('\n', 1)[0]
        offset = lambda column: target.lines[row] + len(raw_line.encode('utf-8')[:column].decode('utf-8', 'ignore'))
        span = target.range(offset(int(match[3])-1), offset(int(match[4] or match[3])))
        diagnostic = {'range': span if target is doc else doc.range(0, 0), 'severity': 1 if match[5] == 'Error' else 2,
                      'code': int(match[6]), 'source': 'vcc', 'message': match[7]}
        if target is not doc:
            diagnostic['message'] = uri_path(target.uri).name + ': ' + diagnostic['message']
            diagnostic['relatedInformation'] = [{'location': {'uri': target.uri, 'range': span}, 'message': match[7]}]
        diagnostics.append(diagnostic)
    return diagnostics


ATTR_TYPES = {'i': 'int', 'f': 'float', 'v': 'vector', 'p': 'vector4', 'u': 'vector2',
              '2': 'matrix2', '3': 'matrix3', '4': 'matrix', 's': 'string', 'd': 'dict'}
KNOWN_ATTRS = {**dict.fromkeys(('P', 'N', 'Cd', 'v', 'uv', 'force', 'accel'), 'vector'),
               **dict.fromkeys(('ptnum', 'primnum', 'vtxnum', 'numpt', 'numprim', 'numvtx', 'id'), 'int'),
               **dict.fromkeys(('Time', 'TimeInc', 'Frame', 'pscale', 'age', 'life'), 'float')}


def source_for(doc, mode):
    """Same-length attribute substitutions plus a fixed line prefix.

    This is an approximation of a wrangle environment, not its generated kernel.
    Unknown attributes fail closed. It is used only for validating local rename,
    never to claim authoritative wrangle compiler diagnostics.
    """
    if mode == 'source' or any(s.kind == 12 and s.type in ('cvex', 'sop', 'surface', 'displacement') for s in doc.symbols):
        return doc.text, 0
    chars = list(doc.text)
    attrs = {}
    for token in doc.tokens:
        if token.kind != 'attribute':
            continue
        prefix, name = token.text.split('@')
        typ = ATTR_TYPES.get(prefix.replace('[]', '')) if prefix else KNOWN_ATTRS.get(name)
        if not typ:
            raise RefactorError('Unknown attribute type for ' + token.text + '; use an explicit type prefix.')
        if '[]' in prefix:
            typ += '[]'
        replacement = '_' * (len(prefix)+1) + name
        if replacement in {t.text for t in doc.code if t.kind == 'id'}:
            raise RefactorError('Generated attribute name collides with a local identifier.')
        attrs[replacement] = typ
        chars[token.start:token.end] = replacement
    includes = []
    for t in doc.tokens:
        if t.kind == 'directive':
            if not re.match(r'^\s*#\s*include\s*[<"]', t.text):
                raise RefactorError('Snippet preprocessing other than literal includes is not supported for rename.')
            includes.append(t.text)
            chars[t.start:t.end] = ''.join('\n' if c == '\n' else ' ' for c in t.text)
    parameters = []
    for name, typ in attrs.items():
        default = '{}' if typ.endswith('[]') or typ == 'dict' else '""' if typ == 'string' else '0'
        parameters.append('export ' + typ.replace('[]', '') + ' ' + name + ('[]' if typ.endswith('[]') else '') + ' = ' + default)
    prefix = '\n'.join(includes + ['cvex __ko_vex_snippet(' + '; '.join(parameters) + ') {']) + '\n'
    return prefix + ''.join(chars) + '\n}\n', prefix.count('\n')


def compile_ast(workspace, doc, graph=None):
    if not workspace.vcc.is_file():
        raise RefactorError('Rename requires a configured Houdini vcc compiler.')
    graph = graph or workspace.graph(doc)
    with tempfile.TemporaryDirectory(prefix='ko-vex-') as directory:
        root = Path(directory)
        paths = {d.uri: root / ('unit' + str(i) + ('.vfl' if i == 0 else '.h')) for i, d in enumerate(graph)}
        line_shift = 0
        for d in graph:
            text = d.text
            # Absolute includes target isolated copies, including unsaved buffers.
            for token, match, uri in reversed(list(workspace.includes(d))):
                if uri in paths:
                    text = text[:token.start] + '#include "' + paths[uri].as_posix() + '"' + text[token.end:]
            copied = Document(d.uri, text, d.version)
            if d is doc:
                text, line_shift = source_for(copied, workspace.mode)
            paths[d.uri].write_text(text, encoding='utf-8')
        command = [str(workspace.vcc), '-z', '-m', 'dumpast', '-c', workspace.context, '-o', 'stdout']
        for path in [uri_path(doc.uri).parent] + workspace.include_paths:
            command += ['-I', str(path)]
        # dumpast alone accepts some unresolved calls. Also require a complete
        # compilation; output is confined to the temporary directory, never run.
        validation = [str(workspace.vcc), '-z', '-d', '-c', workspace.context, '-o', str(root / 'check.vex')]
        for path in [uri_path(doc.uri).parent] + workspace.include_paths:
            validation += ['-I', str(path)]
        checked = workspace.run(validation + [str(paths[doc.uri])])
        if checked.returncode:
            raise CompilerError('VEX compiler rejected this document: ' + checked.stderr.strip()[-2000:],
                                compiler_diagnostics(checked.stderr, doc, graph, paths, line_shift))
        result = workspace.run(command + [str(paths[doc.uri])])
        try:
            ast = json.loads(result.stdout)
        except ValueError:
            message = (result.stderr or result.stdout).strip()[-2000:]
            raise RefactorError('VEX compiler could not validate this document: ' + message) from None
        if result.returncode or ast.get('type') != 'Module':
            raise RefactorError('VEX compiler rejected this document: ' + result.stderr.strip()[-2000:])
        main = str(paths[doc.uri]).replace('\\', '/').casefold()
        occurrences = []
        scopes = [{}]
        def visit(node):
            pushed = node.get('type') in ('Function', 'Shader') or node.get('sframe') is True
            if pushed:
                scopes.append({})
            loc = node.get('loc', {})
            key = (str(loc.get('src', '')), loc.get('line'), loc.get('cstart'))
            name = node.get('name')
            binding = None
            if node.get('type') == 'VarDecl':
                binding = key
                scopes[-1][name] = key
            elif node.get('type') == 'VarRef':
                binding = next((scope[name] for scope in reversed(scopes) if name in scope), None)
            yield node, binding
            for child in node.get('kids', []):
                yield from visit(child)
            if pushed:
                scopes.pop()

        def source_offset(loc):
            if str(loc.get('src', '')).replace('\\', '/').casefold() != main:
                return None
            line = loc.get('line', 0)-1-line_shift
            if not 0 <= line < len(doc.lines):
                return None
            raw_line = doc.text[doc.lines[line]:].split('\n', 1)[0]
            column = len(raw_line.encode('utf-8')[:max(0, loc.get('cstart', 1)-1)].decode('utf-8', 'ignore'))
            return doc.lines[line] + column

        for node, binding in visit(ast):
            loc = node.get('loc', {})
            if str(loc.get('src', '')).replace('\\', '/').casefold() != main:
                continue
            if node.get('type') not in ('VarDecl', 'VarRef'):
                continue
            offset = source_offset(loc)
            if offset is None:
                continue
            declaration = source_offset(dict(zip(('src', 'line', 'cstart'), binding))) if binding else None
            occurrences.append((node['type'], node.get('name'), offset, declaration))
        return ast, occurrences


def fingerprint(ast, old, new):
    """Ignore source locations and spelling, retain compiler's typed AST shape."""
    if isinstance(ast, list):
        return [fingerprint(x, old, new) for x in ast]
    if isinstance(ast, dict):
        return {k: (old if k == 'name' and v == new else fingerprint(v, old, new))
                for k, v in ast.items() if k not in ('loc', 'tloc') and not (ast.get('type') == 'Module' and k == 'name')}
    return ast


def rename(workspace, doc, token, new_name, prepare=False):
    graph = workspace.graph(doc)
    if workspace.vcc.is_file() and uri_path(doc.uri).is_relative_to(workspace.hfs.resolve()):
        raise RefactorError('Houdini installation headers are read-only for refactoring.')
    symbol = doc.resolve(token) if token else None
    if not symbol or symbol.kind != 13:
        raise RefactorError('Only resolved local variables and function parameters can currently be renamed.')
    is_source = workspace.mode == 'source' or uri_path(doc.uri).suffix in ('.h', '.vexh') or any(
        s.kind == 12 and s.type in ('cvex', 'sop', 'surface', 'displacement') for s in doc.symbols)
    if symbol.scope is doc.root and is_source:
        raise RefactorError('Global/header variables are not safe document-local refactors.')
    if doc.errors or doc.unsupported:
        raise RefactorError('Fix delimiter errors or unsupported scope constructs before renaming.')
    if graph.issues:
        raise RefactorError('Rename requires a complete include graph: ' + '; '.join(graph.issues[:3]))
    owner = next((s for s in doc.symbols if s.body is symbol.scope), None)
    if owner and owner.type in ('cvex', 'sop', 'surface', 'displacement') and symbol.token.start < doc.text.find('{', owner.token.end):
        raise RefactorError('Externally bound shader parameters cannot be renamed as local variables.')
    # This conservative boundary is intentional: declarations in inactive branches,
    # macro expansion and token pasting must not be mistaken for local identifiers.
    macro_names = set()
    for d in graph:
        for t in d.tokens:
            if t.kind == 'directive':
                m = re.match(r'^\s*#\s*define\s+(\w+)', t.text)
                if m:
                    macro_names.add(m[1])
                if d is doc and not re.match(r'^\s*#\s*include\s*[<"]', t.text):
                    raise RefactorError('Rename is disabled for documents containing preprocessor logic.')
    if any(t.kind == 'id' and t.text in macro_names for t in doc.tokens):
        raise RefactorError('Rename is disabled when the document invokes header macros.')
    refs = [t for t in doc.code if t.kind == 'id' and doc.resolve(t) is symbol]
    before, occurrences = compile_ast(workspace, doc, graph)
    compiler_refs = {offset for kind, name, offset, declaration in occurrences if name == symbol.name}
    for kind, name, offset, declaration in occurrences:
        if name == symbol.name:
            source_token = next((t for t in doc.code if t.start == offset and t.text == name), None)
            resolved = doc.resolve(source_token) if source_token else None
            if resolved is None or resolved.token.start != declaration:
                raise RefactorError('Compiler scope bindings disagree with the source index; refusing rename.')
    all_refs = {t.start for t in doc.code if t.kind == 'id' and t.text == symbol.name and doc.resolve(t)}
    if compiler_refs != all_refs or not {t.start for t in refs} <= compiler_refs:
        raise RefactorError('Compiler source mapping does not exactly match the resolved identifiers; refusing rename.')
    if prepare:
        return {'range': doc.range(token.start, token.end), 'placeholder': symbol.name}
    if not re.fullmatch(r'[A-Za-z_][A-Za-z_0-9]*', new_name) or new_name in KEYWORDS:
        raise RefactorError('The new name must be a non-keyword VEX identifier.')
    if new_name != symbol.name:
        if new_name in workspace.builtins or any(t.text == new_name for d in graph for t in d.tokens if t.kind == 'id') or new_name in macro_names:
            raise RefactorError('The new name already occurs in this compilation unit; choose an unambiguous name.')
    text = doc.text
    for t in reversed(refs):
        text = text[:t.start] + new_name + text[t.end:]
    changed = Document(doc.uri, text, doc.version)
    after, _ = compile_ast(workspace, changed, [changed] + graph[1:])
    if fingerprint(before, symbol.name, new_name) != fingerprint(after, symbol.name, new_name):
        raise RefactorError('Rename changes the compiler AST; refusing the refactor.')
    return {'documentChanges': [{'textDocument': {'uri': doc.uri, 'version': doc.version},
                                'edits': [{'range': doc.range(t.start, t.end), 'newText': new_name} for t in refs]}]}
