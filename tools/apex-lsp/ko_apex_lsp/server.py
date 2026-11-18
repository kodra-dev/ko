"""Language Server Protocol endpoint for APEX Script (``.apex``)."""
import json
import os
import re
import sys
import threading
import traceback
from pathlib import Path
from urllib.parse import urlparse
from urllib.request import url2pathname

from . import analysis, catalog as catalogs
from .catalog import Catalog, name_key, signature, single_type, split_top
from .language import CONSTANTS, DECORATORS, KEYWORDS, OBJECT_METHODS, SPECIAL_ARGUMENTS, SPECIAL_FUNCTIONS

MAX_FILE = 4 * 1024 * 1024
VERSION = re.compile(r'v\d+(?:_\d+)*$')
# A numeric literal being typed at the cursor (not an identifier containing digits).
NUMBER_PREFIX = re.compile(r'(?:^|[^A-Za-z_0-9.])\.?\d[A-Za-z_0-9.]*$')
# LSP CompletionItemKind / SymbolKind values.
K_METHOD, K_FUNCTION, K_VARIABLE, K_CLASS, K_MODULE, K_PROPERTY, K_KEYWORD, K_CONSTANT = 2, 3, 6, 7, 9, 10, 14, 21
S_FUNCTION, S_VARIABLE = 12, 13
CAPABILITIES = {
    'textDocumentSync': {'openClose': True, 'change': 1, 'save': False},
    'completionProvider': {'triggerCharacters': ['.', '@'], 'resolveProvider': True},
    'hoverProvider': True,
    'signatureHelpProvider': {'triggerCharacters': ['(', ','], 'retriggerCharacters': [',']},
    'definitionProvider': True,
    'documentSymbolProvider': True,
}
SPECIALS = {name_key(n): n for n in SPECIAL_FUNCTIONS}


def path_uri(path):
    return Path(path).resolve().as_uri()


def uri_path(uri):
    parsed = urlparse(uri)
    value = url2pathname(parsed.path)
    return Path('//' + parsed.netloc + value if parsed.netloc else value)


def fence(code):
    return '```python\n' + code + '\n```'


class Document:
    def __init__(self, uri, text, version):
        self.uri, self.version = uri, version
        self.update(text)

    def update(self, text):
        self.text = text
        self.view = analysis.Text(text)
        self.tree, self.error = (None, None)
        if len(text.encode('utf-8', errors='replace')) <= MAX_FILE:
            self.tree, self.error = analysis.parse(text)


class Server:
    def __init__(self, notify=lambda *args: None):
        self.notify = notify
        self.documents = {}
        self.analyses = {}            # uri -> last Analysis built from a parseable revision
        self.settings = {}
        self.catalog = Catalog.empty()
        self.catalog_ready = threading.Event()
        self.catalog_ready.set()
        self.catalog_generation = 0
        self.shutdown = False

    # ----- catalogue --------------------------------------------------------------------
    def load_catalog(self):
        self.catalog_generation += 1
        generation = self.catalog_generation
        self.catalog_ready.clear()

        def work():
            loaded, problem = Catalog.empty(), None
            try:
                hfs = catalogs.find_houdini(self.settings)
                source = catalogs.catalog_source(hfs)
                if source is None:
                    problem = ('Houdini was not found' if hfs is None else f'{hfs} has no packages/apex/python3*libs/apexscript.py')
                else:
                    loaded = catalogs.load(source, os.environ.get('KO_VEX_CACHE'))
                    loaded.add_methods(OBJECT_METHODS)
                    self.notify('window/logMessage', {'type': 3, 'message': f'APEX function catalogue: {source}'})
            except Exception as error:  # a broken install must not stop syntax checking
                loaded, problem = Catalog.empty(), f'{type(error).__name__}: {error}'
            finally:
                # Requests wait on this event, so it is set whatever happened above.
                # (Each request builds a fresh Analysis with the current catalogue.)
                if generation == self.catalog_generation:
                    self.catalog = loaded
                    self.catalog_ready.set()
            if problem and generation == self.catalog_generation:
                self.notify('window/showMessage', {'type': 2, 'message':
                            'APEX Script completion and hover are unavailable: ' + problem +
                            '. Set lsp.ko-apex.settings.houdiniPath in Zed (syntax checking still works).'})
                self.notify('window/logMessage', {'type': 2, 'message': 'APEX catalogue unavailable: ' + problem})
        threading.Thread(target=work, name='ko-apex-catalog', daemon=True).start()

    def wait_catalog(self):
        self.catalog_ready.wait(30)
        return self.catalog

    # ----- documents --------------------------------------------------------------------
    def analysis_for(self, doc):
        """Analysis of the current text, or of a repaired copy while the user is typing."""
        catalog = self.wait_catalog()
        if doc.tree is not None:
            result = analysis.Analysis(doc.text, doc.tree, catalog)
            self.analyses[doc.uri] = result
            return result
        fixed = analysis.repaired(doc.text, doc.error)
        if fixed:
            result = analysis.Analysis(fixed[0], fixed[1], catalog)
            self.analyses[doc.uri] = result
            return result
        previous = self.analyses.get(doc.uri)
        if previous is None:
            return analysis.Analysis('', None, catalog)
        previous.catalog = catalog
        return previous

    def publish(self, doc):
        if doc.tree is None and doc.error is None:
            values = [{'range': doc.view.range(0, 0), 'severity': 3, 'source': 'apex-script',
                       'message': 'File exceeds 4 MiB; APEX Script checking is disabled.'}]
        else:
            values = analysis.diagnostics(doc.text, doc.tree, doc.error)
        self.notify('textDocument/publishDiagnostics', {'uri': doc.uri, 'version': doc.version, 'diagnostics': values})

    # ----- protocol ---------------------------------------------------------------------
    def handle(self, method, params):
        p = params or {}
        if method == 'initialize':
            self.settings = dict(p.get('initializationOptions') or {})
            self.load_catalog()
            return {'capabilities': CAPABILITIES, 'serverInfo': {'name': 'ko-apex-lsp', 'version': '0.1.0'}}
        if method in ('initialized', '$/cancelRequest', '$/setTrace', 'textDocument/didSave'):
            return None
        if method == 'shutdown':
            self.shutdown = True
            return None
        if method == 'exit':
            raise SystemExit(0)
        if method == 'workspace/didChangeConfiguration':
            settings = (p.get('settings') or {}).get('ko-apex')
            if isinstance(settings, dict):
                old = {k: self.settings.get(k) for k in ('houdiniPath', 'discoverHoudini')}
                self.settings.update(settings)
                if old != {k: self.settings.get(k) for k in old}:
                    self.load_catalog()
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
            self.analyses.pop(uri, None)
            self.notify('textDocument/publishDiagnostics', {'uri': uri, 'diagnostics': []})
            return None
        if method == 'completionItem/resolve':
            return self.resolve_completion(p)
        doc = self.documents.get((p.get('textDocument') or {}).get('uri'))
        if method.startswith('textDocument/') and doc is None:
            return None
        if method == 'textDocument/completion':
            return self.completion(doc, doc.view.offset(p['position']))
        if method == 'textDocument/hover':
            return self.hover(doc, doc.view.offset(p['position']))
        if method == 'textDocument/signatureHelp':
            return self.signature_help(doc, doc.view.offset(p['position']))
        if method == 'textDocument/definition':
            return self.definition(doc, doc.view.offset(p['position']))
        if method == 'textDocument/documentSymbol':
            return self.document_symbols(doc)
        raise KeyError('Method not supported: ' + method)

    # ----- cursor context ---------------------------------------------------------------
    @staticmethod
    def in_comment_or_string(tokens, offset):
        last = tokens[-1] if tokens else None
        if not last or last.end < offset:
            return False
        if last.kind == 'comment':
            return True
        if last.kind == 'string':
            body = last.text.lstrip('rRbBuUfF')
            quote = body[:3] if body[:3] in ("'''", '"""') else body[:1]
            closed = len(body) >= 2*len(quote) and body.endswith(quote)
            return not closed or last.end > offset
        return False

    @staticmethod
    def chain(tokens, end):
        """Dotted access ending before index ``end``: [(name, ['call'|'index', ...]), ...]."""
        steps, i, after_dot = [], end - 1, False
        while i >= 0:
            suffix = []
            while i >= 0 and tokens[i].text in (')', ']'):
                closer, opener = tokens[i].text, '(' if tokens[i].text == ')' else '['
                depth = 0
                while i >= 0:
                    if tokens[i].text == closer:
                        depth += 1
                    elif tokens[i].text == opener:
                        depth -= 1
                        if depth == 0:
                            break
                    i -= 1
                suffix.insert(0, 'call' if opener == '(' else 'index')
                i -= 1
            if i < 0 or tokens[i].kind != 'name' or tokens[i].text in KEYWORDS:
                # A literal or other expression receiver ("abc".x, (1, 2).y) is not resolved.
                if suffix or after_dot:
                    return []
                break
            steps.append((tokens[i].text, suffix))
            i -= 1
            if i >= 0 and tokens[i].text == '.':
                i -= 1
                after_dot = True
                continue
            break
        return list(reversed(steps))

    def local_offset(self, doc, result, offset):
        return result.text.offset(doc.view.position(offset))

    # ----- resolution -------------------------------------------------------------------
    def first(self, name, result, at):
        """What a leading name refers to: ('ns', path) | ('type', T) | ('class', T) | ...."""
        if name == 'apex':
            return ('ns', 'apex')
        symbol = result.lookup(name, result.scope_at(at))
        if symbol:
            return ('user', symbol) if symbol.kind == 'function' else ('var', symbol)
        if name in CONSTANTS:
            return ('constant', name)
        if name in self.catalog.types:
            return ('class', name)
        if name_key(name) in SPECIALS:
            return ('special', SPECIALS[name_key(name)])
        entry = self.catalog.function(name)
        return ('entry', entry) if entry else None

    def member(self, value, name, called=False):
        if value is None:
            return None
        kind = value[0]
        if kind == 'ns':
            # apex.sop.copytopoints is both a function and a namespace of its own;
            # a call means the function.
            child = self.catalog.child(value[1], name)
            entry = self.catalog.namespace_function(value[1], name)
            if entry and (called or not child):
                return ('entry', entry)
            return ('ns', child) if child else None
        if kind == 'type':
            entry = self.catalog.method(value[1], name)
            if entry:
                return ('entry', entry, value[1])
            # Node attributes are its ports: a.xform_out, b.parent_in.
            return ('type', 'ApexPortID') if value[1] == 'ApexNodeID' else None
        if kind == 'entry' and VERSION.match(name):
            return value  # IntBitMaskToBool.v2_0(): an explicit function version
        return None

    def call_value(self, target, receiver=None):
        """Type produced by calling ``target``."""
        if target is None:
            return None
        kind = target[0]
        if kind == 'class':
            return ('type', target[1])
        if kind == 'entry':
            returns = single_type(target[1].get('returns'))
            if returns:
                return ('type', returns)
            if target[1].get('inplace') and len(target) > 2:
                return ('type', target[2])  # in-place methods chain on the receiver
            return None
        if kind == 'user':
            returns = single_type(target[1].returns)
            return ('type', returns) if returns else None
        if kind == 'special':
            returns = SPECIAL_FUNCTIONS[target[1]][2]
            return ('type', returns) if returns else None
        return None

    def as_value(self, target):
        if target is None:
            return None
        if target[0] == 'var':
            return ('type', target[1].type) if target[1].type else None
        if target[0] == 'constant':
            return ('type', CONSTANTS[target[1]][0])
        return target

    def index_value(self, value):
        if not value or value[0] != 'type':
            return None
        name = value[1]
        if name.endswith('Array'):
            return ('type', name[:-5])
        if name.startswith('Vector'):
            return ('type', 'Float')
        if name.startswith('Matrix'):
            return ('type', 'Vector' + name[-1])
        return None

    def apply(self, target, suffix):
        value = target
        for i, item in enumerate(suffix):
            value = self.call_value(value) if item == 'call' else self.index_value(self.as_value(value))
        return self.as_value(value) if not suffix else value

    def resolve(self, steps, result, at, call_last=False):
        """(value of all steps but the last, target of the last step)."""
        value, target = None, None
        for i, (name, suffix) in enumerate(steps):
            last = i == len(steps) - 1
            called = (suffix[:1] == ['call']) or (last and call_last)
            target = self.first(name, result, at) if i == 0 else self.member(value, name, called)
            if last:
                return value, target
            value = self.apply(target, suffix)
            if value is None:
                return None, None
        return value, target

    # ----- completion -------------------------------------------------------------------
    def entry_item(self, entry, kind):
        return {'label': entry['name'], 'kind': kind, 'detail': signature(entry),
                'data': {'owner': entry['owner'], 'name': entry['name']}}

    def entry_doc(self, entry):
        parts = [fence(signature(entry))]
        if entry.get('doc'):
            parts.append(entry['doc'])
        if entry.get('inplace'):
            parts.append('_In-place: updates its first input._')
        return '\n\n'.join(parts)

    def resolve_completion(self, item):
        data = item.get('data') or {}
        catalog = self.wait_catalog()
        owner, name = data.get('owner'), data.get('name')
        entry = None
        if owner == '':
            entry = catalog.functions.get(name)
        elif owner in catalog.namespaces:
            entry = catalog.namespaces[owner]['functions'].get(name)
        elif owner in catalog.types:
            entry = catalog.types[owner]['methods'].get(name)
        if entry:
            item['documentation'] = {'kind': 'markdown', 'value': self.entry_doc(entry)}
        return item

    def completion(self, doc, offset):
        tokens = analysis.tokenize(doc.text, offset)
        if self.in_comment_or_string(tokens, offset):
            return None
        if NUMBER_PREFIX.search(doc.text[max(0, offset-256):offset]):
            # Typing a number (2, 2.0, .5, 1e5): Zed would fuzzy-match it to Vector2, fit10, ...
            return {'isIncomplete': False, 'items': []}
        base = len(tokens)
        if tokens and tokens[-1].kind == 'name' and tokens[-1].end == offset:
            base -= 1
        result = self.analysis_for(doc)
        at = self.local_offset(doc, result, offset)
        if base > 0 and tokens[base-1].text == '.':
            steps = self.chain(tokens, base-1)
            if not steps:
                return {'isIncomplete': False, 'items': []}
            value, target = self.resolve(steps, result, at)
            value = self.apply(target, steps[-1][1])
            return {'isIncomplete': False, 'items': self.member_items(value)}
        if base > 0 and tokens[base-1].text == '@':
            return {'isIncomplete': False, 'items': [
                {'label': n, 'kind': K_FUNCTION, 'detail': sig, 'documentation': {'kind': 'markdown', 'value': doc_}}
                for n, (sig, doc_) in DECORATORS.items()]}
        items = []
        call = self.enclosing_call(tokens, base)
        if call and base > 0 and tokens[base-1].text in ('(', ','):
            items.extend(self.keyword_items(call, result, at))
        items.extend(self.global_items(result, at))
        return {'isIncomplete': False, 'items': items}

    def member_items(self, value):
        if not value:
            return []
        if value[0] == 'ns':
            entry = self.catalog.namespaces.get(value[1], {})
            items = [{'label': n, 'kind': K_MODULE, 'detail': path} for n, path in entry.get('children', {}).items()]
            items += [self.entry_item(f, K_FUNCTION) for f in entry.get('functions', {}).values()]
            if value[1] == 'apex':
                names = {i['label'] for i in items}
                items += [self.entry_item(f, K_FUNCTION) for n, f in self.catalog.functions.items() if n not in names]
            return items
        if value[0] == 'type':
            methods = self.catalog.types.get(value[1], {}).get('methods', {})
            return [self.entry_item(f, K_METHOD) for f in methods.values()]
        return []

    def global_items(self, result, at):
        items = [{'label': k, 'kind': K_KEYWORD, 'sortText': '9' + k} for k in KEYWORDS]
        items.append({'label': 'apex', 'kind': K_MODULE, 'detail': 'APEX function namespaces'})
        for name, (sig, doc_, _) in SPECIAL_FUNCTIONS.items():
            items.append({'label': name, 'kind': K_FUNCTION, 'detail': sig,
                          'documentation': {'kind': 'markdown', 'value': doc_}})
        for name, (type_name, value) in CONSTANTS.items():
            items.append({'label': name, 'kind': K_CONSTANT, 'detail': f'{type_name} = {value}'})
        for name, entry in self.catalog.types.items():
            if '::' not in name:
                items.append({'label': name, 'kind': K_CLASS, 'detail': 'APEX type'})
        names = {i['label'] for i in items}
        for name, entry in self.catalog.functions.items():
            if name not in names:
                items.append(self.entry_item(entry, K_FUNCTION))
        for symbol in result.visible(at):
            kind = K_FUNCTION if symbol.kind == 'function' else K_VARIABLE
            detail = symbol.detail if symbol.kind == 'function' else symbol.type
            # Local definitions sort first.
            items.append({'label': symbol.name, 'kind': kind, 'detail': detail, 'sortText': '0' + symbol.name})
        return items

    def keyword_items(self, call, result, at):
        callee, _, _ = call
        params = self.parameters(callee, result, at) or []
        items = [{'label': p['name'] + '=', 'kind': K_PROPERTY, 'insertText': p['name'] + '=',
                  'filterText': p['name'], 'detail': p['type'], 'sortText': '00' + p['name']}
                 for p in params if p['name'] and not p['name'].startswith('*')]
        items += [{'label': n + '=', 'kind': K_PROPERTY, 'insertText': n + '=', 'filterText': n,
                   'detail': d, 'sortText': '8' + n} for n, d in SPECIAL_ARGUMENTS.items()]
        return items

    # ----- calls ------------------------------------------------------------------------
    @staticmethod
    def enclosing_call(tokens, end):
        """(callee chain, argument index, keyword name) for the innermost open call."""
        depth, i = 0, end - 1
        while i >= 0:
            text = tokens[i].text
            if text in (')', ']', '}'):
                depth += 1
            elif text in ('(', '[', '{'):
                if depth == 0:
                    if text != '(':
                        return None
                    break
                depth -= 1
            i -= 1
        if i < 0:
            return None
        steps = Server.chain(tokens, i)
        if not steps:
            return None
        index, keyword, level, argument_start = 0, None, 0, i + 1
        for j in range(i + 1, end):
            text = tokens[j].text
            if text in ('(', '[', '{'):
                level += 1
            elif text in (')', ']', '}'):
                level -= 1
            elif text == ',' and level == 0:
                index, argument_start = index + 1, j + 1
        if argument_start + 1 < end and tokens[argument_start].kind == 'name' and tokens[argument_start+1].text == '=':
            keyword = tokens[argument_start].text
        return steps, index, keyword

    def callee_target(self, steps, result, at):
        _, target = self.resolve(steps, result, at, call_last=True)
        return target

    def parameters(self, steps, result, at):
        target = self.callee_target(steps, result, at)
        if not target:
            return None
        if target[0] == 'entry':
            return target[1]['params']
        if target[0] == 'user':
            return target[1].params
        if target[0] == 'special':
            sig = SPECIAL_FUNCTIONS[target[1]][0]
            inner = sig[sig.find('(')+1:sig.rfind(')')]
            return [{'name': p.split(':')[0].split('=')[0].strip(), 'type': p.partition(':')[2].strip(), 'default': ''}
                    for p in split_top(inner)]
        return None

    def signature_help(self, doc, offset):
        tokens = analysis.tokenize(doc.text, offset)
        if self.in_comment_or_string(tokens, offset):
            return None
        call = self.enclosing_call(tokens, len(tokens))
        if not call:
            return None
        steps, index, keyword = call
        result = self.analysis_for(doc)
        at = self.local_offset(doc, result, offset)
        target = self.callee_target(steps, result, at)
        if not target or target[0] not in ('entry', 'user', 'special', 'class'):
            return None
        if target[0] == 'class':
            label, params, documentation = f'{target[1]}(...)', [], self.catalog.types[target[1]].get('doc', '')
        elif target[0] == 'special':
            label, documentation = SPECIAL_FUNCTIONS[target[1]][0], SPECIAL_FUNCTIONS[target[1]][1]
            params = self.parameters(steps, result, at)
        elif target[0] == 'user':
            label, params, documentation = target[1].detail[4:], target[1].params, target[1].doc
        else:
            label, params, documentation = signature(target[1]), target[1]['params'], target[1].get('doc', '')
        information = {'label': label, 'documentation': {'kind': 'markdown', 'value': documentation}}
        offsets, cursor = [], label.find('(') + 1
        for p in params:
            found = label.find(p['name'], cursor)
            if found < 0:
                break
            end = label.find(',', found)
            end = label.rfind(')') if end < 0 else end
            offsets.append([found, max(end, found + len(p['name']))])
            cursor = end
        if len(offsets) == len(params):
            information['parameters'] = [{'label': o} for o in offsets]
        active = index
        names = [p['name'].lstrip('*') for p in params]
        if keyword is not None:
            active = names.index(keyword) if keyword in names else len(names)
        elif params and active >= len(params) and 'VariadicArg' in params[-1].get('type', ''):
            active = len(params) - 1
        information['activeParameter'] = active
        return {'signatures': [information], 'activeSignature': 0, 'activeParameter': active}

    # ----- hover / definition -----------------------------------------------------------
    def word_steps(self, doc, offset):
        """Chain for the identifier under ``offset``, and that identifier's range."""
        tokens = analysis.tokenize(doc.text)
        index = next((i for i, t in enumerate(tokens) if t.start <= offset <= t.end and t.kind == 'name'), None)
        if index is None or tokens[index].text in KEYWORDS:
            return None, None, False
        steps = self.chain(tokens, index + 1)
        called = index + 1 < len(tokens) and tokens[index+1].text == '('
        return steps, tokens[index], called

    def hover(self, doc, offset):
        steps, token, called = self.word_steps(doc, offset)
        if not steps:
            return None
        result = self.analysis_for(doc)
        at = self.local_offset(doc, result, offset)
        _, target = self.resolve(steps, result, at, call_last=called)
        content = self.describe(target)
        if not content:
            return None
        return {'contents': {'kind': 'markdown', 'value': content}, 'range': doc.view.range(token.start, token.end)}

    def describe(self, target):
        if not target:
            return None
        kind = target[0]
        if kind == 'entry':
            return self.entry_doc(target[1])
        if kind == 'ns':
            entry = self.catalog.namespaces.get(target[1], {})
            return (fence(target[1]) + f"\n\nAPEX namespace: {len(entry.get('functions', {}))} functions"
                    + (', ' + ', '.join(sorted(entry['children'])) if entry.get('children') else ''))
        if kind == 'class':
            entry = self.catalog.types.get(target[1], {})
            text = fence(f'class {target[1]}') + '\n\nAPEX type'
            if entry.get('doc'):
                text += '\n\n' + entry['doc']
            return text + f"\n\n{len(entry.get('methods', {}))} methods"
        if kind == 'constant':
            type_name, value = CONSTANTS[target[1]]
            return fence(f'{target[1]}: {type_name} = {value}')
        if kind == 'special':
            sig, documentation, _ = SPECIAL_FUNCTIONS[target[1]]
            return fence(sig) + '\n\n' + documentation
        if kind == 'user':
            return fence(target[1].detail) + ('\n\n' + target[1].doc if target[1].doc else '')
        if kind == 'var':
            symbol = target[1]
            label = '(parameter) ' if symbol.kind == 'parameter' else ''
            return fence(label + symbol.name + (': ' + symbol.type if symbol.type else ''))
        if kind == 'type':
            return fence(target[1])
        return None

    def definition(self, doc, offset):
        steps, _, called = self.word_steps(doc, offset)
        if not steps:
            return None
        result = self.analysis_for(doc)
        at = self.local_offset(doc, result, offset)
        _, target = self.resolve(steps, result, at, call_last=called)
        if not target:
            return None
        if target[0] in ('user', 'var'):
            symbol = target[1]
            return [{'uri': doc.uri, 'range': result.text.range(symbol.start, symbol.end)}]
        source = self.catalog.source
        line = None
        if target[0] == 'entry':
            line = target[1].get('line')
        elif target[0] == 'ns':
            line = self.catalog.namespaces.get(target[1], {}).get('line')
        elif target[0] == 'class':
            line = self.catalog.types.get(target[1], {}).get('line')
        if source is None or not line:
            return None
        position = {'line': line - 1, 'character': 0}
        return [{'uri': path_uri(source), 'range': {'start': position, 'end': position}}]

    def document_symbols(self, doc):
        result = self.analysis_for(doc)

        def entry(symbol):
            children = [entry(c) for c in result.symbols if c.scope is symbol and c.kind != 'parameter']
            selection = result.text.range(symbol.start, symbol.end)
            whole = result.text.range(*symbol.body) if symbol.kind == 'function' else selection
            value = {'name': symbol.name, 'kind': S_FUNCTION if symbol.kind == 'function' else S_VARIABLE,
                     'range': whole, 'selectionRange': selection,
                     'detail': symbol.detail if symbol.kind == 'function' else symbol.type}
            if children:
                value['children'] = children
            return value
        return [entry(s) for s in result.symbols if s.scope is None]


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
        # Catalogue-loader notifications come from another thread.
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
