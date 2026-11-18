"""Language Server Protocol endpoint for Houdini OpenCL (``.cl``).

Offline only: there is no OpenCL compile. Houdini's @-binding code generation
happens inside Houdini, so diagnostics cover #bind syntax, delimiters and missing
headers, never driver compile errors.
"""
import json
import os
import re
import sys
import threading
import traceback
from pathlib import Path
from urllib.parse import urlparse
from urllib.request import url2pathname

from . import bindings as B
from .builtins import BUILTINS
from .headers import DEFAULT_INCLUDES, HeaderIndex, find_houdini, include_root
from .source import (HOUDINI_TYPES, INCLUDE, KEYWORDS, TYPES, Text, definitions, directive_name, tokenize)

MAX_FILE = 4 * 1024 * 1024
# LSP CompletionItemKind / SymbolKind / DiagnosticSeverity values.
K_METHOD, K_FUNCTION, K_FIELD, K_VARIABLE, K_CLASS, K_MODULE, K_PROPERTY = 2, 3, 5, 6, 7, 9, 10
K_KEYWORD, K_FILE, K_CONSTANT, K_STRUCT, K_TYPE = 14, 17, 21, 22, 25
S_FUNCTION, S_VARIABLE, S_STRUCT, S_CONSTANT, S_EVENT = 12, 13, 23, 14, 24
SOURCE = 'houdini-opencl'
CAPABILITIES = {
    'textDocumentSync': {'openClose': True, 'change': 1, 'save': False},
    'completionProvider': {'triggerCharacters': ['@', '.', '#', '<', '"', '/'], 'resolveProvider': True},
    'hoverProvider': True,
    'signatureHelpProvider': {'triggerCharacters': ['(', ','], 'retriggerCharacters': [',']},
    'definitionProvider': True,
    'documentSymbolProvider': True,
}
INCLUDE_TRIGGERS = ('<', '"', '/')
NUMBER_PREFIX = re.compile(r'(?:^|[^A-Za-z_0-9.@])\.?\d[A-Za-z_0-9.]*$')
TYPE_FLAGS = (['float', 'float2', 'float3', 'float4', 'float9', 'float16', 'int', 'int2', 'int3', 'int4',
               'fpreal', 'fpreal3', 'exint', 'float[]', 'int[]', 'float?', 'int?'])


def path_uri(path):
    return Path(path).resolve().as_uri()


def uri_path(uri):
    parsed = urlparse(uri)
    value = url2pathname(parsed.path)
    return Path('//' + parsed.netloc + value if parsed.netloc else value)


def fence(code):
    return '```c\n' + code + '\n```'


class Document:
    def __init__(self, uri, text, version):
        self.uri, self.version = uri, version
        self.update(text)

    def update(self, text):
        self.text = text
        self.view = Text(text)
        self.too_large = len(text.encode('utf-8', errors='replace')) > MAX_FILE
        source = '' if self.too_large else text
        self.tokens = tokenize(source)
        self.definitions = definitions(source, self.tokens)
        self.binds = [B.parse_bind(t.text, t.start) for t in self.tokens
                      if t.kind == 'directive' and directive_name(t.text) == 'bind']
        self.includes = []          # (quote, name, token)
        for t in self.tokens:
            if t.kind == 'directive' and directive_name(t.text) in ('include', 'import'):
                m = INCLUDE.match(t.text)
                if m:
                    self.includes.append((m[2], m[3].strip(), t))

    @property
    def directory(self):
        try:
            return uri_path(self.uri).parent
        except Exception:
            return None


class Server:
    def __init__(self, notify=lambda *args: None):
        self.notify = notify
        self.documents = {}
        self.settings = {}
        self.headers = HeaderIndex()
        self.hfs = None
        self.configured = False
        self.shutdown = False

    # ----- configuration ----------------------------------------------------------------
    def configure(self, settings):
        self.settings.update(settings or {})
        previous, self.hfs = self.hfs, find_houdini(self.settings)
        if self.configured and previous == self.hfs:
            return   # only report when the chosen installation changes
        self.configured = True
        root = include_root(self.hfs)
        if root is None or not root.is_dir():
            self.notify('window/showMessage', {'type': 2, 'message':
                        'Houdini OpenCL headers were not found, so header completion and include checks are off. '
                        'Set lsp.ko-opencl.settings.houdiniPath (or lsp.ko-vex.settings.houdiniPath) in Zed.'})
        else:
            self.notify('window/logMessage', {'type': 3, 'message': f'Houdini OpenCL headers: {root}'})

    def search_paths(self):
        paths = [Path(os.path.expandvars(p)).expanduser() for p in self.settings.get('includePaths') or []]
        root = include_root(self.hfs)
        if root is not None:
            paths.append(root)
        return paths

    def header_closure(self, doc):
        defaults = self.settings.get('defaultIncludes', DEFAULT_INCLUDES)
        return self.headers.closure([(q, n) for q, n, _ in doc.includes], doc.directory, self.search_paths(), defaults)

    # ----- diagnostics ------------------------------------------------------------------
    def diagnostics(self, doc):
        if doc.too_large:
            return [{'range': doc.view.range(0, 0), 'severity': 3, 'source': SOURCE,
                     'message': 'File exceeds 4 MiB; Houdini OpenCL checking is disabled.'}]
        result = []

        def add(start, end, severity, message):
            result.append({'range': doc.view.range(start, end), 'severity': severity, 'source': SOURCE, 'message': message})
        for bind in doc.binds:
            for start, end, severity, message in bind.problems:
                add(start, end, severity, message)
        for t in doc.tokens:
            if t.kind == 'directive' and directive_name(t.text) == 'runover':
                words = t.text.split('//', 1)[0].split()
                words = words[1:] if words[0] != '#' else words[2:]
                if not words:
                    add(t.start, t.end, 1, '#runover needs a mode: ' + ', '.join(B.RUNOVER_MODES) + '.')
                elif words[0] not in B.RUNOVER_MODES:
                    at = t.start + t.text.index(words[0], t.text.index('runover') + 7)
                    add(at, at + len(words[0]), 2, f'Unknown #runover mode `{words[0]}`. Expected ' + ', '.join(B.RUNOVER_MODES) + '.')
        search = self.search_paths()
        if search:
            conditional = self.conditional_depths(doc)
            for quote, name, t in doc.includes:
                if name and self.headers.resolve(name, quote, doc.directory, search) is None:
                    message = (f'Header not found: {name}. Searched $HFS/houdini/ocl/include'
                               + (' and lsp.ko-opencl.settings.includePaths.' if self.settings.get('includePaths') else '.'))
                    if conditional.get(t.start):
                        # e.g. host-only <math.h> behind #ifndef __OPENCL_VERSION__
                        add(t.start, t.end, 3, message + ' It is inside an #if block, which may be inactive.')
                    else:
                        add(t.start, t.end, 2, message)
        result.extend(self.delimiter_problems(doc))
        return result

    @staticmethod
    def conditional_depths(doc):
        """{directive start: #if nesting depth}, not counting a whole-file include guard."""
        directives = [t for t in doc.tokens if t.kind == 'directive']
        guard = None
        if len(directives) >= 2 and directive_name(directives[0].text) == 'ifndef':
            name = directives[0].text.split()[-1]
            if directive_name(directives[1].text) == 'define' and name in directives[1].text.split():
                guard = directives[0].start
        result, depth = {}, 0
        for t in directives:
            kind = directive_name(t.text)
            if kind == 'endif':
                depth = max(0, depth - 1)
            result[t.start] = depth
            if kind in ('if', 'ifdef', 'ifndef') and t.start != guard:
                depth += 1
        return result

    def delimiter_problems(self, doc):
        conditional = any(t.kind == 'directive' and directive_name(t.text) in ('if', 'ifdef', 'ifndef', 'elif', 'else')
                          for t in doc.tokens)
        # Conditional branches may legitimately open a brace twice; only warn there.
        severity = 2 if conditional else 1
        pairs, stack, problems = {')': '(', ']': '[', '}': '{'}, [], []
        for t in doc.tokens:
            if t.kind != 'op':
                continue
            if t.text in '([{':
                stack.append(t)
            elif t.text in pairs:
                if stack and stack[-1].text == pairs[t.text]:
                    stack.pop()
                else:
                    problems.append((t, f'Unmatched `{t.text}`.'))
                    if stack and any(s.text == pairs[t.text] for s in stack):
                        while stack and stack[-1].text != pairs[t.text]:
                            problems.append((stack.pop(), None))
                        stack.pop()
        for t in stack:
            problems.append((t, None))
        closers = {'(': ')', '[': ']', '{': '}'}
        return [{'range': doc.view.range(t.start, t.end), 'severity': severity, 'source': SOURCE,
                 'message': message or f'`{t.text}` is never closed with `{closers[t.text]}`.'} for t, message in problems[:50]]

    def publish(self, doc):
        self.notify('textDocument/publishDiagnostics', {'uri': doc.uri, 'version': doc.version, 'diagnostics': self.diagnostics(doc)})

    # ----- protocol ---------------------------------------------------------------------
    def handle(self, method, params):
        p = params or {}
        if method == 'initialize':
            self.configure(p.get('initializationOptions') or {})
            return {'capabilities': CAPABILITIES, 'serverInfo': {'name': 'ko-opencl-lsp', 'version': '0.1.0'}}
        if method in ('initialized', '$/cancelRequest', '$/setTrace', 'textDocument/didSave'):
            return None
        if method == 'shutdown':
            self.shutdown = True
            return None
        if method == 'exit':
            raise SystemExit(0)
        if method == 'workspace/didChangeConfiguration':
            settings = (p.get('settings') or {}).get('ko-opencl')
            if isinstance(settings, dict):
                self.configure(settings)
                for doc in self.documents.values():
                    self.publish(doc)
            return None
        if method == 'textDocument/didOpen':
            d = p['textDocument']
            doc = self.documents[d['uri']] = Document(d['uri'], d['text'], d.get('version'))
            self.publish(doc)
            return None
        if method == 'textDocument/didChange':
            doc = self.documents.get(p['textDocument']['uri'])
            if doc is not None and p.get('contentChanges'):
                doc.version = p['textDocument'].get('version')
                doc.update(p['contentChanges'][-1]['text'])
                self.publish(doc)
            return None
        if method == 'textDocument/didClose':
            uri = p['textDocument']['uri']
            self.documents.pop(uri, None)
            self.notify('textDocument/publishDiagnostics', {'uri': uri, 'diagnostics': []})
            return None
        if method == 'completionItem/resolve':
            return self.resolve_completion(p)
        doc = self.documents.get((p.get('textDocument') or {}).get('uri'))
        if method.startswith('textDocument/') and doc is None:
            return None
        if method == 'textDocument/completion':
            trigger = (p.get('context') or {}).get('triggerCharacter')
            return self.completion(doc, doc.view.offset(p['position']), trigger)
        if method == 'textDocument/hover':
            return self.hover(doc, doc.view.offset(p['position']))
        if method == 'textDocument/signatureHelp':
            return self.signature_help(doc, doc.view.offset(p['position']))
        if method == 'textDocument/definition':
            return self.definition(doc, doc.view.offset(p['position']))
        if method == 'textDocument/documentSymbol':
            return self.document_symbols(doc)
        raise KeyError('Method not supported: ' + method)

    # ----- lookups ----------------------------------------------------------------------
    def all_binds(self, doc):
        """#bind declarations of the document, then of included headers (header, Bind)."""
        result = [(None, b) for b in doc.binds if b.name]
        names = {b.name for _, b in result}
        for header in self.header_closure(doc):
            for t in header.binds:
                bind = B.parse_bind(t.text, t.start)
                if bind.name and bind.name not in names:
                    names.add(bind.name)
                    result.append((header, bind))
        return result

    def find_bind(self, doc, name):
        return next(((h, b) for h, b in self.all_binds(doc) if b.name == name), (None, None))

    def visible_locals(self, doc, offset):
        return [d for d in doc.definitions if d.scope is None or d.scope[0] <= offset <= d.scope[1]]

    def lookup(self, doc, name, offset):
        """('local', Definition) | ('header', Header, Definition) | ('builtin', entry) | ('type', doc)."""
        local = [d for d in self.visible_locals(doc, offset) if d.name == name and d.kind != 'kernel']
        if local:
            # Innermost scope (latest start) wins; functions over declarations.
            local.sort(key=lambda d: (d.scope[0] if d.scope else -1, d.kind in ('function', 'macro')))
            return ('local', local[-1])
        for header in self.header_closure(doc):
            for definition in header.definitions:
                if definition.name == name:
                    return ('header', header, definition)
        if name in BUILTINS:
            return ('builtin', BUILTINS[name])
        if name in HOUDINI_TYPES:
            return ('type', HOUDINI_TYPES[name])
        if name in TYPES:
            return ('type', 'OpenCL built-in type.')
        return None

    # ----- cursor helpers ---------------------------------------------------------------
    @staticmethod
    def token_at(tokens, offset):
        for t in tokens:
            if t.start <= offset <= t.end and t.kind != 'op':
                return t
            if t.start > offset:
                break
        return None

    @staticmethod
    def code_before(tokens, offset):
        return [t for t in tokens if t.end <= offset and t.kind not in ('comment', 'directive')]

    def receiver_binding(self, tokens, index):
        """For tokens[index] after '.', the @binding name it is a member of, else None."""
        if index >= 2 and tokens[index - 1].text == '.' and tokens[index - 2].kind == 'binding':
            return tokens[index - 2].text[1:]
        return None

    def binding_methods(self, doc, name):
        """(class, {method: row}) for @name."""
        _, bind = self.find_bind(doc, name)
        if bind is not None:
            return bind.binding_class, B.METHODS.get(bind.binding_class, {})
        if name in ('P', 'dPdx', 'dPdy', 'dPdxy'):
            return 'space', {k: (k, v, '') for k, v in B.SPACE_MEMBERS.items()}
        return None, {}

    # ----- completion -------------------------------------------------------------------
    def completion(self, doc, offset, trigger=None):
        text = doc.text
        line_start = text.rfind('\n', 0, offset) + 1
        line = text[line_start:offset]
        token = self.token_at(doc.tokens, max(0, offset - 1)) if offset else None
        if token and token.kind == 'comment' and token.start < offset and (token.end > offset or not token.text.startswith('/*')):
            return None
        stripped = line.lstrip()
        if stripped.startswith('#'):
            return self.directive_completion(doc, stripped, offset)
        if trigger in INCLUDE_TRIGGERS:
            return None   # `a < b`, `x / y`, strings: these only trigger #include completion
        if trigger == '#':
            return None
        if token and token.kind == 'string' and token.start < offset < token.end:
            return None
        m = re.search(r'@(\w*)$', line)
        if m:
            return self.binding_completion(doc, offset - len(m[0]), offset, at_line_start=not line[:m.start()].strip())
        m = re.search(r'@(\w+)\s*\.\s*(\w*)$', line)
        if m:
            cls, methods = self.binding_methods(doc, m[1])
            if cls is None:
                cls, methods = 'attribute', B.METHODS['attribute']
            return {'isIncomplete': False, 'items': [self.method_item(name, row, cls) for name, row in methods.items()]}
        if re.search(r'(\.|->)\s*\w*$', line):
            return None   # struct members and swizzles: not modelled, let the editor use words
        if NUMBER_PREFIX.search(line):
            return {'isIncomplete': False, 'items': []}
        return {'isIncomplete': False, 'items': self.global_items(doc, offset)}

    def method_item(self, name, row, cls):
        signature, description, requirement = row
        documentation = description + (f'\n\n_Requires: {requirement}._' if requirement else '')
        return {'label': name, 'kind': K_METHOD if '(' in signature else K_PROPERTY,
                'detail': f'{cls}: @name.{signature}', 'documentation': {'kind': 'markdown', 'value': documentation}}

    def directive_completion(self, doc, stripped, offset):
        def edited(items, typed):
            # Replace exactly what was typed: '.', '[', '?' and '/' are not word characters,
            # so the editor's own word range would leave "matrix." or "float[" behind.
            replace = doc.view.range(offset - len(typed), offset)
            for item in items:
                item['textEdit'] = {'range': replace, 'newText': item.get('insertText', item['label'])}
                item.setdefault('filterText', item['label'])
            return {'isIncomplete': False, 'items': items}
        m = re.match(r'#\s*(\w*)$', stripped)
        if m:
            return edited([
                {'label': name, 'kind': K_KEYWORD, 'detail': '#' + name,
                 **({'documentation': {'kind': 'markdown', 'value': d}} if d else {})}
                for name, d in B.DIRECTIVES.items()], m[1])
        m = re.match(r'#\s*(?:include|import)\s*[<"]([^>"]*)$', stripped)
        if m:
            typed = m[1]
            folder = typed.rsplit('/', 1)[0] + '/' if '/' in typed else ''
            items, seen = [], set()
            for relative in self.headers.files(self.search_paths()):
                if not relative.startswith(folder):
                    continue
                rest = relative[len(folder):]
                label = rest.split('/', 1)[0] + ('/' if '/' in rest else '')
                if label not in seen:
                    seen.add(label)
                    items.append({'label': label, 'kind': K_MODULE if label.endswith('/') else K_FILE})
            return edited(items, typed[len(folder):])
        m = re.match(r'#\s*runover\s+(\w*)$', stripped)
        if m:
            return edited([{'label': n, 'kind': K_KEYWORD, 'documentation': d}
                           for n, d in B.RUNOVER_MODES.items()], m[1])
        m = re.match(r'#\s*bind\b(.*)$', stripped)
        if m:
            words = m[1].split()
            starting = m[1].endswith((' ', '\t')) or not words
            index = len(words) if starting else len(words) - 1
            typed = '' if starting else words[-1]
            if index == 0:
                return edited([{'label': n, 'kind': K_CLASS, 'detail': f'{cls} binding', 'documentation': d}
                               for n, (cls, d) in B.BINDING_TYPES.items()], typed)
            if index == 1 or '=' in typed:
                return {'isIncomplete': False, 'items': []}
            items = [{'label': t, 'kind': K_TYPE, 'detail': 'type flag', 'documentation': B.TYPE_FLAG_DOC} for t in TYPE_FLAGS]
            for name, d in B.FLAGS.items():
                items.append({'label': name, 'kind': K_PROPERTY, 'detail': 'flag', 'documentation': d})
                items.append({'label': 'no' + name, 'kind': K_PROPERTY, 'detail': 'flag', 'documentation': d, 'sortText': '~no' + name})
            for name, d in B.KEY_FLAGS.items():
                items.append({'label': name + '=', 'kind': K_FIELD, 'detail': 'option', 'documentation': d})
            return edited(items, typed)
        return None

    def binding_completion(self, doc, start, offset, at_line_start):
        replace = doc.view.range(start, offset)
        items = []

        def item(label, kind, detail, documentation, sort):
            items.append({'label': label, 'kind': kind, 'detail': detail, 'filterText': label,
                          'sortText': sort + label, 'textEdit': {'range': replace, 'newText': label},
                          'documentation': {'kind': 'markdown', 'value': documentation}})
        names = set()
        for header, bind in self.all_binds(doc):
            names.add(bind.name)
            source = f' (from {header.path.name})' if header else ''
            item('@' + bind.name, K_VARIABLE, bind.describe(), self.describe_bind(bind) + source, '0')
        for name, (type_, where, d) in B.GLOBAL_BINDINGS.items():
            if name not in names:
                item('@' + name, K_CONSTANT, f'{type_} (global binding)', f'{d}\n\n_Valid when running over {where}._', '1')
        for name, d in B.KERNEL_MARKERS.items():
            item('@' + name, K_KEYWORD, '@' + name + ' { ... }', d, '0' if at_line_start else '9')
        return {'isIncomplete': False, 'items': items}

    def global_items(self, doc, offset):
        items, seen = [], set()

        def add(item):
            if item['label'] not in seen:
                seen.add(item['label'])
                items.append(item)
        kinds = {'function': K_FUNCTION, 'kernel': K_FUNCTION, 'macro': K_CONSTANT, 'struct': K_STRUCT,
                 'typedef': K_TYPE, 'variable': K_VARIABLE, 'parameter': K_VARIABLE, 'enumerator': K_CONSTANT}
        for d in self.visible_locals(doc, offset):
            if d.kind != 'kernel' or not d.name.startswith('@'):
                add({'label': d.name, 'kind': kinds.get(d.kind, K_VARIABLE), 'detail': d.detail, 'sortText': '0' + d.name})
        for header in self.header_closure(doc):
            for d in header.definitions:
                add({'label': d.name, 'kind': kinds.get(d.kind, K_FUNCTION), 'detail': d.detail[:120],
                     'sortText': '1' + d.name, 'data': {'h': str(header.path), 'n': d.name}})
        for name, d in HOUDINI_TYPES.items():
            add({'label': name, 'kind': K_TYPE, 'detail': 'Houdini OpenCL type', 'documentation': d, 'sortText': '2' + name})
        for name in TYPES:
            add({'label': name, 'kind': K_TYPE, 'detail': 'OpenCL type', 'sortText': '3' + name})
        for name, entry in BUILTINS.items():
            add({'label': name, 'kind': K_FUNCTION, 'detail': entry['sigs'][0] if entry['sigs'] else '',
                 'sortText': '4' + name, 'data': {'b': name}})
        for name in KEYWORDS:
            add({'label': name, 'kind': K_KEYWORD, 'sortText': '5' + name})
        return items

    def resolve_completion(self, item):
        data = item.get('data') or {}
        if data.get('b') in BUILTINS:
            item['documentation'] = {'kind': 'markdown', 'value': self.builtin_doc(BUILTINS[data['b']])}
        elif data.get('h'):
            header = self.headers.header(Path(data['h']))
            definition = next((d for d in header.definitions if d.name == data.get('n')), None) if header else None
            if definition:
                item['documentation'] = {'kind': 'markdown', 'value': self.definition_doc(definition, header)}
        return item

    # ----- documentation ----------------------------------------------------------------
    @staticmethod
    def builtin_doc(entry):
        text = fence('\n'.join(entry['sigs']))
        if entry.get('doc'):
            text += '\n\n' + entry['doc']
        return text + '\n\n_OpenCL C built-in. `n` is any vector width 2, 3, 4, 8 or 16._'

    @staticmethod
    def definition_doc(definition, header=None):
        text = fence(definition.detail)
        if definition.doc:
            text += '\n\n' + definition.doc
        if header is not None:
            text += f'\n\n_From {header.path.name}_'
        return text

    @staticmethod
    def describe_bind(bind):
        cls_doc = B.BINDING_TYPES.get(bind.kind, ('', 'Unknown binding type.'))[1]
        flags = [w for w, _, _ in bind.flags]
        lines = [fence(bind.describe()), f'**{bind.kind}** binding — {cls_doc}']
        if bind.data_type:
            lines.append(f'Type: `{bind.data_type}`')
        if flags:
            lines.append('Flags: ' + ', '.join(f'`{f}`' for f in flags))
        methods = B.METHODS.get(bind.binding_class, {})
        if methods:
            lines.append('Methods: ' + ', '.join(f'`.{m}`' for m in methods))
        return '\n\n'.join(lines)

    # ----- hover ------------------------------------------------------------------------
    def hover(self, doc, offset):
        token = self.token_at(doc.tokens, offset)
        if token is None:
            return None
        content = None
        if token.kind == 'directive':
            content = self.directive_hover(doc, token, offset)
        elif token.kind == 'binding':
            content = self.binding_hover(doc, token.text[1:])
        elif token.kind == 'name':
            index = doc.tokens.index(token)
            receiver = self.receiver_binding(doc.tokens, index)
            if receiver:
                cls, methods = self.binding_methods(doc, receiver)
                row = methods.get(token.text) or B.METHODS['attribute'].get(token.text)
                if row:
                    signature, description, requirement = row
                    content = fence(f'@{receiver}.{signature}') + '\n\n' + description
                    if requirement:
                        content += f'\n\n_Requires: {requirement}._'
            else:
                content = self.name_hover(doc, token.text, offset)
        if not content:
            return None
        return {'contents': {'kind': 'markdown', 'value': content}, 'range': doc.view.range(token.start, token.end)}

    def binding_hover(self, doc, name):
        if name in B.KERNEL_MARKERS:
            return fence('@' + name) + '\n\n' + B.KERNEL_MARKERS[name]
        header, bind = self.find_bind(doc, name)
        if bind is not None:
            return self.describe_bind(bind) + (f'\n\n_Declared in {header.path.name}_' if header else '')
        if name in B.GLOBAL_BINDINGS:
            type_, where, d = B.GLOBAL_BINDINGS[name]
            return fence(f'{type_} @{name}') + f'\n\n{d}\n\n_Global binding, valid when running over {where}._'
        return (fence('@' + name) + '\n\nNot declared by a `#bind` in this file. It must come from the node\'s '
                '**Bindings** tab; otherwise the kernel fails to compile in Houdini.')

    def directive_hover(self, doc, token, offset):
        name = directive_name(token.text)
        local = offset - token.start
        word = re.search(r'\S*$', token.text[:local])[0] + re.match(r'\S*', token.text[local:])[0]
        if name == 'bind':
            bind = next((b for b in doc.binds if b.start == token.start), None)
            if bind is None:
                return None
            if bind.name_start <= offset <= bind.name_end:
                return self.describe_bind(bind)
            if word == bind.kind:
                return f'**{word}** binding — ' + B.BINDING_TYPES.get(word, ('', 'Unknown binding type.'))[1]
            if word.lstrip('#') == 'bind':
                return B.DIRECTIVES['bind']
            documentation = B.flag_doc(word)
            if documentation is None:
                decorations = ''.join(c for c in word if c in B.DECORATIONS)
                return '\n\n'.join(B.flag_doc(c) for c in decorations) or None
            return documentation
        if name == 'runover':
            return B.RUNOVER_MODES.get(word, B.DIRECTIVES['runover'])
        if name in ('include', 'import'):
            match = INCLUDE.match(token.text)
            if not match:
                return None
            path = self.headers.resolve(match[3].strip(), match[2], doc.directory, self.search_paths())
            return f'`{path}`' if path else f'Header `{match[3]}` was not found.'
        return None

    def name_hover(self, doc, name, offset):
        found = self.lookup(doc, name, offset)
        if found is None:
            return None
        if found[0] == 'local':
            d = found[1]
            label = {'parameter': '(parameter) ', 'variable': '', 'macro': ''}.get(d.kind, '')
            return fence(label + d.detail) + ('\n\n' + d.doc if d.doc else '')
        if found[0] == 'header':
            return self.definition_doc(found[2], found[1])
        if found[0] == 'builtin':
            return self.builtin_doc(found[1])
        return fence(name) + '\n\n' + found[1]

    # ----- signature help ---------------------------------------------------------------
    def signature_help(self, doc, offset):
        code = self.code_before(doc.tokens, offset)
        depth, i = 0, len(code) - 1
        while i >= 0:
            text = code[i].text
            if text in (')', ']', '}'):
                depth += 1
            elif text in ('(', '[', '{'):
                if depth == 0:
                    if text != '(':
                        return None
                    break
                depth -= 1
            elif text == ';' and depth == 0:
                return None
            i -= 1
        if i <= 0:
            return None
        active, level = 0, 0
        for t in code[i + 1:]:
            if t.text in '([{':
                level += 1
            elif t.text in ')]}':
                level -= 1
            elif t.text == ',' and level == 0:
                active += 1
        callee = code[i - 1]
        signatures = []
        if callee.kind == 'binding':
            cls, methods = self.binding_methods(doc, callee.text[1:])
            row = methods.get('getAt')
            if row:
                signatures.append((f'@{callee.text[1:]}({row[0][len("getAt("):-1]})', row[1]))
        elif callee.kind == 'name':
            receiver = self.receiver_binding(code, i - 1)
            if receiver:
                _, methods = self.binding_methods(doc, receiver)
                row = methods.get(callee.text) or B.METHODS['attribute'].get(callee.text)
                if row and '(' in row[0]:
                    signatures.append((f'@{receiver}.{row[0]}', row[1]))
            else:
                found = self.lookup(doc, callee.text, offset)
                if found and found[0] in ('local', 'header'):
                    d = found[1] if found[0] == 'local' else found[2]
                    if d.kind in ('function', 'kernel', 'macro') and (d.params or '(' in d.detail):
                        label = d.detail if d.kind != 'macro' else f'{d.name}({", ".join(d.params)})'
                        signatures.append((label, d.doc))
                elif found and found[0] == 'builtin':
                    signatures.extend((s, found[1].get('doc', '')) for s in found[1]['sigs'])
        if not signatures:
            return None
        result = []
        for label, documentation in signatures:
            information = {'label': label, 'documentation': {'kind': 'markdown', 'value': documentation or ''}}
            open_at, close_at = label.find('('), label.rfind(')')
            if 0 <= open_at < close_at:
                parameters, start, level = [], open_at + 1, 0
                for j in range(open_at + 1, close_at + 1):
                    ch = label[j]
                    if ch in '([':
                        level += 1
                    elif ch in ')]' and j != close_at:
                        level -= 1
                    if (ch == ',' and level == 0) or j == close_at:
                        begin = start + len(label[start:j]) - len(label[start:j].lstrip())
                        if begin < j:
                            parameters.append({'label': [begin, j]})
                        start = j + 1
                information['parameters'] = parameters
            result.append(information)
        best = next((n for n, s in enumerate(result) if len(s.get('parameters', [])) > active), 0)
        return {'signatures': result, 'activeSignature': best, 'activeParameter': active}

    # ----- definition -------------------------------------------------------------------
    def definition(self, doc, offset):
        token = self.token_at(doc.tokens, offset)
        if token is None:
            return None
        if token.kind == 'directive' and directive_name(token.text) in ('include', 'import'):
            match = INCLUDE.match(token.text)
            path = self.headers.resolve(match[3].strip(), match[2], doc.directory, self.search_paths()) if match else None
            return [self.location(path, None, 0, 0)] if path else None
        if token.kind == 'binding':
            header, bind = self.find_bind(doc, token.text[1:])
            if bind is None:
                return None
            if header is None:
                return [{'uri': doc.uri, 'range': doc.view.range(bind.name_start, bind.name_end)}]
            return [self.location(header.path, Text(header.text), bind.name_start, bind.name_end)]
        if token.kind != 'name' or self.receiver_binding(doc.tokens, doc.tokens.index(token)):
            return None
        found = self.lookup(doc, token.text, offset)
        if found is None:
            return None
        if found[0] == 'local':
            return [{'uri': doc.uri, 'range': doc.view.range(found[1].start, found[1].end)}]
        if found[0] == 'header':
            header, d = found[1], found[2]
            return [self.location(header.path, Text(header.text), d.start, d.end)]
        return None

    @staticmethod
    def location(path, view, start, end):
        if view is None:
            position = {'line': 0, 'character': 0}
            return {'uri': path_uri(path), 'range': {'start': position, 'end': position}}
        return {'uri': path_uri(path), 'range': view.range(start, end)}

    # ----- symbols ----------------------------------------------------------------------
    def document_symbols(self, doc):
        kinds = {'function': S_FUNCTION, 'kernel': S_EVENT, 'macro': S_CONSTANT, 'struct': S_STRUCT, 'typedef': S_STRUCT}
        result = []
        for d in doc.definitions:
            if d.kind not in kinds:
                continue
            whole = doc.view.range(*(d.body or (d.start, d.end)))
            result.append({'name': d.name, 'kind': kinds[d.kind], 'detail': d.detail[:120], 'range': whole,
                           'selectionRange': doc.view.range(d.start, d.end)})
        for bind in doc.binds:
            if bind.name:
                result.append({'name': '@' + bind.name, 'kind': S_VARIABLE, 'detail': bind.describe(),
                               'range': doc.view.range(bind.start, bind.end),
                               'selectionRange': doc.view.range(bind.name_start, bind.name_end)})
        result.sort(key=lambda s: (s['range']['start']['line'], s['range']['start']['character']))
        return result


def read_message(stream):
    headers = {}
    while True:
        line = stream.readline(8192)
        if not line:
            return None
        if line in (b'\r\n', b'\n'):
            break
        if len(line) >= 8192 or b':' not in line:
            raise ValueError('Malformed LSP header')
        key, value = line.decode('ascii').split(':', 1)
        headers[key.lower()] = value.strip()
    size = int(headers.get('content-length', '-1'))
    if not 0 <= size <= 16 * 1024 * 1024:
        raise ValueError('Invalid LSP Content-Length')
    body = stream.read(size)
    if len(body) != size:
        raise ValueError('Truncated LSP message')
    return json.loads(body)


def main():
    output_lock = threading.Lock()

    def send(message):
        body = json.dumps(message, ensure_ascii=False).encode('utf-8')
        with output_lock:
            sys.stdout.buffer.write(('Content-Length: ' + str(len(body)) + '\r\n\r\n').encode('ascii') + body)
            sys.stdout.buffer.flush()

    server = Server(lambda method, params: send({'jsonrpc': '2.0', 'method': method, 'params': params}))
    while True:
        try:
            message = read_message(sys.stdin.buffer)
            if message is None:
                return
            if 'method' not in message:
                continue
            try:
                result = server.handle(message['method'], message.get('params'))
                if 'id' in message:
                    send({'jsonrpc': '2.0', 'id': message['id'], 'result': result})
            except SystemExit:
                return
            except Exception as error:
                if 'id' in message:
                    code = -32601 if isinstance(error, KeyError) else -32603
                    send({'jsonrpc': '2.0', 'id': message['id'], 'error': {'code': code, 'message': str(error)}})
                else:
                    traceback.print_exc(file=sys.stderr)
        except (ValueError, OSError):
            traceback.print_exc(file=sys.stderr)
            return
