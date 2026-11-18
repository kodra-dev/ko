"""Dependency-free stdio Language Server Protocol endpoint for VEX."""
from pathlib import Path
import json
import re
import sys
import threading
import traceback

from .compiler import KNOWN_ATTRS, RefactorError, rename
from .diagnostics import Diagnostics
from .validation import Context
from .model import Document, KEYWORDS, TYPES, path_uri, uri_path
from .workspace import Workspace, MAX_FILE

CAPABILITIES = {
    'positionEncoding': 'utf-16',
    'textDocumentSync': {'openClose': True, 'change': 2, 'save': {'includeText': False}},
    'completionProvider': {'resolveProvider': True, 'triggerCharacters': ['@', '"', '<', '/']},
    'hoverProvider': True, 'definitionProvider': True,
    'signatureHelpProvider': {'triggerCharacters': ['(', ',', ';']},
    'documentSymbolProvider': True, 'referencesProvider': True,
    'renameProvider': {'prepareProvider': True},
}


def load_settings(root, overrides):
    config = root / 'vex-lsp.json'
    settings = json.loads(config.read_text(encoding='utf-8-sig')) if config.is_file() else {}
    settings.update(overrides)
    return settings


def location(doc, token):
    return {'uri': doc.uri, 'range': doc.range(token.start, token.end)}


class Server:
    def __init__(self, notify=lambda *args: None, request=lambda *args: None):
        self.notify, self.request = notify, request
        self.workspace = None
        self.options = {}
        self.shutdown = False
        self.lock = threading.RLock()
        self.checker = None
        self.watch_dynamic = False
        self.watch_registration = 0
        self.watch_roots = ()

    def compiler_enabled(self):
        return self.workspace.settings.get('compilerDiagnostics', True) and self.workspace.vcc.is_file()

    def structural(self, doc):
        result = [{'range': doc.range(t.start, t.end), 'severity': 1, 'source': 'vex-lsp', 'message': message}
                  for t, message in doc.errors]
        # With a compiler, it decides which conditional includes are active.
        if not self.compiler_enabled():
            for t, m, uri in self.workspace.includes(doc):
                if not uri:
                    result.append({'range': doc.range(t.start, t.end), 'severity': 2, 'source': 'vex-lsp',
                                   'message': 'Header not found: '+m[2]+'. Check lsp.ko-vex.settings.includePaths in Zed.'})
        return result

    def publish_diagnostics(self, doc, values):
        self.notify('textDocument/publishDiagnostics', {'uri': doc.uri, 'version': doc.version, 'diagnostics': values})

    def checked(self, job, result):
        with self.lock:
            if self.checker is None or not self.checker.current(job):
                return
            doc = self.workspace.open.get(job.uri)
            if doc is None or doc.version != job.version:
                return
            values = result.diagnostics
            # Successful compiler validation also clears structural errors from
            # inactive preprocessor branches. Keep the cheap fallback only when
            # the compilation environment could not be reconstructed.
            if any(d.get('code') == 'validation-unavailable' for d in values):
                values = self.structural(doc)+values
            self.publish_diagnostics(doc, values)
            self.register_watchers()

    def schedule(self, changed=(), immediate=False, all_documents=False, disk_changes=()):
        if self.shutdown:
            return
        if self.compiler_enabled():
            if self.checker is None:
                self.checker = Diagnostics(self.checked)
            context = Context.capture(self.workspace)
            buffers = {uri: (d.text, d.version) for uri, d in self.workspace.open.items()}
            delay = min(5000, max(100, int(self.workspace.settings.get('diagnosticsDebounceMs', 500)))) / 1000
            targets = self.checker.update(context, buffers, set(changed), immediate, all_documents, delay, disk_changes)
        else:
            if self.checker:
                self.checker.close(wait=False)
                self.checker = None
            targets = set(self.workspace.open) if all_documents else set(changed)
        for uri in targets:
            if uri in self.workspace.open:
                doc = self.workspace.open[uri]
                self.publish_diagnostics(doc, self.structural(doc))

    def register_watchers(self):
        if not self.watch_dynamic:
            return
        roots = list(dict.fromkeys([self.workspace.root]+list(self.workspace.include_paths)))
        if self.checker:
            with self.checker.condition:
                dependencies = set().union(*self.checker.dependencies.values()) if self.checker.dependencies else set()
            for uri in dependencies:
                parent = uri_path(uri).parent
                if not any(parent.is_relative_to(root) for root in roots):
                    roots.append(parent)
        roots = [p for p in roots if not any(p != other and p.is_relative_to(other) for other in roots)]
        signature = tuple(sorted(p.as_posix() for p in roots))
        if signature == self.watch_roots:
            return
        self.watch_roots = signature
        if self.watch_registration:
            self.request('client/unregisterCapability', {'unregisterations': [
                {'id': 'ko-vex-files-'+str(self.watch_registration), 'method': 'workspace/didChangeWatchedFiles'}]})
        self.watch_registration += 1
        self.request('client/registerCapability', {'registrations': [{
            'id': 'ko-vex-files-'+str(self.watch_registration), 'method': 'workspace/didChangeWatchedFiles',
            'registerOptions': {'watchers': [{'globPattern': p.as_posix().rstrip('/')+'/**/*.{h,vexh,vfl}', 'kind': 7} for p in roots]}}]})

    def close(self):
        with self.lock:
            checker, self.checker = self.checker, None
        if checker:
            checker.close()

    def handle(self, method, params):
        if method in ('shutdown', 'exit'):
            self.close()
        with self.lock:
            if self.checker and method in ('workspace/didChangeConfiguration', 'textDocument/prepareRename', 'textDocument/rename', 'textDocument/references'):
                with self.checker.suspend():
                    return self._handle(method, params)
            return self._handle(method, params)

    def _handle(self, method, params):
        p = params or {}
        if method == 'initialize':
            root = uri_path(p['rootUri']) if p.get('rootUri') else Path.cwd()
            if not p.get('rootUri') and p.get('workspaceFolders'):
                root = uri_path(p['workspaceFolders'][0]['uri'])
            self.watch_dynamic = p.get('capabilities', {}).get('workspace', {}).get('didChangeWatchedFiles', {}).get('dynamicRegistration', False)
            self.options = p.get('initializationOptions') or {}
            self.workspace = Workspace(root, load_settings(root, self.options))
            return {'capabilities': CAPABILITIES, 'serverInfo': {'name': 'ko-vex-lsp', 'version': '0.1.0'}}
        if method == 'initialized':
            self.register_watchers()
            if self.workspace.builtin_error:
                self.notify('window/logMessage', {'type': 2, 'message': self.workspace.builtin_error})
            else:
                self.notify('window/logMessage', {'type': 3, 'message': 'Ko VEX using Houdini at ' + str(self.workspace.hfs) + '. Override with lsp.ko-vex.settings.houdiniPath.'})
            if not self.workspace.settings.get('compilerDiagnostics', True):
                self.notify('window/logMessage', {'type': 3, 'message': 'Live VEX diagnostics disabled by compilerDiagnostics: false.'})
            return None
        if method == 'shutdown':
            self.shutdown = True
            return None
        if method == 'exit':
            raise SystemExit(0 if self.shutdown else 1)
        if method == 'textDocument/didSave':
            self.schedule([p['textDocument']['uri']], immediate=True)
            return None
        if method == 'workspace/didChangeWatchedFiles':
            changes = p.get('changes', [])
            for change in changes:
                self.workspace.cache.pop(change['uri'], None)
            changed = [change['uri'] for change in changes]
            self.schedule(changed, all_documents=any(change.get('type') in (1, 3) for change in changes), disk_changes=changed)
            return None
        if method in ('$/cancelRequest', '$/setTrace'):
            return None
        if method == 'workspace/didChangeConfiguration':
            if 'settings' in p:
                value = p['settings'] or {}
                self.options = value.get('ko-vex', value)
            self.workspace.configure(load_settings(self.workspace.root, self.options))
            self.schedule(all_documents=True)
            self.register_watchers()
            return None
        if method == 'textDocument/didOpen':
            d = p['textDocument']
            uri_path(d['uri'])
            if len(d['text'].encode('utf-8')) > MAX_FILE:
                raise ValueError('Document exceeds the 4 MiB limit')
            self.workspace.open[d['uri']] = Document(d['uri'], d['text'], d.get('version'))
            self.schedule([d['uri']])
            return None
        if method == 'textDocument/didChange':
            d = p['textDocument']
            doc = self.workspace.document(d['uri'])
            if doc.version is not None and d['version'] <= doc.version:
                return None
            text = doc.text
            for change in p['contentChanges']:
                if 'range' in change:
                    current = Document(d['uri'], text)
                    a, b = current.offset(change['range']['start']), current.offset(change['range']['end'])
                    text = text[:a] + change['text'] + text[b:]
                else:
                    text = change['text']
            if len(text.encode('utf-8')) > MAX_FILE:
                raise ValueError('Document exceeds the 4 MiB limit')
            self.workspace.open[d['uri']] = Document(d['uri'], text, d['version'])
            self.schedule([d['uri']])
            return None
        if method == 'textDocument/didClose':
            uri = p['textDocument']['uri']
            self.workspace.open.pop(uri, None)
            self.workspace.cache.pop(uri, None)
            self.notify('textDocument/publishDiagnostics', {'uri': uri, 'diagnostics': []})
            self.schedule([uri])
            self.register_watchers()
            return None
        doc = self.workspace.document(p['textDocument']['uri']) if 'textDocument' in p else None
        if doc:
            self.workspace.graph(doc)
        offset = doc.offset(p['position']) if doc and 'position' in p else 0
        token = doc.token_at(offset) if doc else None
        if method == 'completionItem/resolve':
            item = dict(p)
            name = item.get('data', {}).get('builtin')
            if name in self.workspace.builtins:
                item['documentation'] = {'kind': 'markdown', 'value': self.workspace.documentation(name)}
            return item
        if method == 'textDocument/completion':
            return self.completion(doc, offset)
        if method == 'textDocument/hover':
            if not token or token.kind not in ('id', 'attribute'):
                return None
            symbol = doc.resolve(token)
            members = self.member_symbols(doc, token)
            macros = [(d, t, definition) for d, t, definition in self.workspace.macros(doc) if t.text == token.text]
            if token.kind == 'attribute':
                typ = self.workspace.receiver_type(doc, [token]) or 'unknown type'
                content = '```vex\n' + typ + ' ' + token.text + '\n```\n\nGeometry binding; availability depends on the execution context.'
            elif members:
                content = '\n\n'.join('```vex\n' + (s.signature or s.type + ' ' + s.name) + '\n```\n\n' + s.doc for _, s in members)
            elif symbol and symbol.kind in (8, 13):
                content = '```vex\n' + symbol.type + ' ' + symbol.name + '\n```'
            elif macros:
                content = '\n\n'.join('```vex\n' + definition + '\n```' for _, _, definition in macros)
                content += '\n\nMacro navigation candidates; conditional branches are not evaluated by the browsing index.'
            else:
                signatures = self.workspace.signatures(token.text, doc)
                if not signatures:
                    return None
                content = '\n\n'.join('```vex\n' + sig + '\n```' for sig, _, _ in signatures[:16])
                descriptions = list(dict.fromkeys(description for _, description, _ in signatures if description))
                content += '\n\n' + '\n\n'.join(descriptions[:3])
                if any(uri for _, _, uri in signatures):
                    content += '\n\nHeader declarations are navigation candidates; conditional branches and overloads are not type-resolved.'
            return {'contents': {'kind': 'markdown', 'value': content}, 'range': doc.range(token.start, token.end)}
        if method == 'textDocument/definition':
            for t, m, uri in self.workspace.includes(doc):
                if t.start <= offset <= t.end and uri:
                    return [{'uri': uri, 'range': {'start': {'line': 0, 'character': 0}, 'end': {'line': 0, 'character': 0}}}]
            if not token or token.kind != 'id':
                return []
            members = self.member_symbols(doc, token)
            if members:
                return [location(d, s.token) for d, s in members]
            symbol = doc.resolve(token)
            if symbol and symbol.kind in (8, 13):
                return [location(doc, symbol.token)]
            found = [location(d, s.token) for d, s in self.workspace.declarations(doc) if s.name == token.text]
            found += [location(d, t) for d, t, _ in self.workspace.macros(doc) if t.text == token.text]
            if found:
                return found
            if token.text in self.workspace.builtins:
                # A real, read-only documentation document, not fictional C source.
                cache = self.workspace.documentation_dir
                cache.mkdir(parents=True, exist_ok=True)
                path = cache / (token.text + '.md')
                signatures = self.workspace.signatures(token.text, doc)
                path.write_text('# ' + token.text + '\n\n' + '\n\n'.join('```vex\n'+s+'\n```' for s, _, _ in signatures)
                                + '\n\n' + self.workspace.documentation(token.text), encoding='utf-8')
                return [{'uri': path_uri(path), 'range': {'start': {'line': 0, 'character': 0}, 'end': {'line': 0, 'character': 0}}}]
            return []
        if method == 'textDocument/signatureHelp':
            return self.signature_help(doc, offset)
        if method == 'textDocument/documentSymbol':
            def entry(s):
                item = {'name': s.name, 'detail': s.signature or s.type, 'kind': s.kind,
                        'range': doc.range(s.token.start, s.body.end if s.body else s.token.end),
                        'selectionRange': doc.range(s.token.start, s.token.end)}
                if s.body:
                    item['children'] = [entry(child) for child in s.body.symbols]
                return item
            return [entry(s) for s in doc.root.symbols]
        if method == 'textDocument/prepareRename':
            return rename(self.workspace, doc, token, '', prepare=True)
        if method == 'textDocument/rename':
            return rename(self.workspace, doc, token, p['newName'])
        if method == 'textDocument/references':
            rename(self.workspace, doc, token, '', prepare=True)
            symbol = doc.resolve(token)
            return [location(doc, t) for t in doc.code if t.kind == 'id' and doc.resolve(t) is symbol
                    and (p.get('context', {}).get('includeDeclaration', True) or t is not symbol.token)]
        raise KeyError('Method not supported: ' + method)

    def member_symbols(self, doc, token):
        if not token or token.kind != 'id':
            return []
        index = next((i for i, t in enumerate(doc.code) if t.start == token.start), -1)
        if index >= 2 and doc.code[index-1].text in ('.', '->'):
            typ = self.workspace.receiver_type(doc, doc.code[:index-1])
            return [(d, s) for d, s in self.workspace.members(doc, typ or '') if s.name == token.text]
        return []

    def completion(self, doc, offset):
        line = doc.text[doc.lines[doc.position(offset)['line']]:offset]
        include = re.match(r'^\s*#\s*include\s*([<"])([^>"\n]*)$', line)
        if include:
            prefix = include[2]
            return {'isIncomplete': False, 'items': [
                {'label': name, 'kind': 19 if name.endswith('/') else 17,
                 'textEdit': {'range': doc.range(offset-len(prefix), offset), 'newText': name}}
                for name in self.workspace.include_completions(doc, prefix, include[1] == '"')]}
        token = doc.token_at(offset)
        if token and token.kind in ('comment', 'string', 'directive'):
            return {'isIncomplete': False, 'items': []}
        prefix_match = re.search(r'[A-Za-z_][A-Za-z_0-9]*$', doc.text[:offset])
        prefix = prefix_match.group() if prefix_match else ''
        attribute = re.search(r'(?:[ifvpu234sd](?:\[\])?)?@([A-Za-z_]\w*)?$', line)
        if attribute:
            prefix = attribute[1] or ''
            names = dict(KNOWN_ATTRS)
            for t in doc.tokens:
                if t.kind == 'attribute':
                    names[t.text.split('@')[1]] = self.workspace.receiver_type(doc, [t]) or 'unknown'
            return {'isIncomplete': False, 'items': [
                {'label': name, 'kind': 10, 'detail': typ + ' geometry binding',
                 'textEdit': {'range': doc.range(offset-len(prefix), offset), 'newText': name}}
                for name, typ in sorted(names.items()) if name.startswith(prefix)]}
        before = [t for t in doc.code if t.end <= offset-len(prefix)]
        if before and before[-1].text in ('.', '->'):
            typ = self.workspace.receiver_type(doc, before[:-1])
            members = self.workspace.members(doc, typ or '')
            result = [{'label': s.name, 'kind': 2 if s.kind == 12 else 5, 'detail': s.signature or s.type,
                       'documentation': {'kind': 'markdown', 'value': s.doc}} for _, s in members if s.name.startswith(prefix)]
            components = {'vector2': 'xy', 'vector': 'xyz', 'vector4': 'xyzw'}.get(typ, '')
            result += [{'label': name, 'kind': 5, 'detail': 'float component'} for name in components if name.startswith(prefix)]
            return {'isIncomplete': False, 'items': result}
        items = {}
        for s in doc.visible(offset):
            items[s.name] = {'label': s.name, 'kind': 3 if s.kind == 12 else 6, 'detail': s.signature or s.type}
        for d, s in self.workspace.declarations(doc):
            items.setdefault(s.name, {'label': s.name, 'kind': 3 if s.kind == 12 else 22 if s.kind == 23 else 6,
                                     'detail': s.signature or s.type,
                                     'documentation': {'kind': 'markdown', 'value': s.doc + '\n\nHeader candidate: ' + uri_path(d.uri).name}})
        for d, t, definition in self.workspace.macros(doc):
            items.setdefault(t.text, {'label': t.text, 'kind': 21, 'detail': definition})
        for name, overloads in self.workspace.builtins.items():
            items.setdefault(name, {'label': name, 'kind': 3, 'detail': str(len(overloads)) + ' built-in overload(s)',
                                    'data': {'builtin': name}})
        for name in KEYWORDS:
            items.setdefault(name, {'label': name, 'kind': 14 if name not in TYPES else 7})
        result = [item for name, item in sorted(items.items()) if name.startswith(prefix)]
        return {'isIncomplete': len(result) > 500, 'items': result[:500]}

    def signature_help(self, doc, offset):
        stack = []
        code = [t for t in doc.code if t.start < offset]
        for i, t in enumerate(code):
            if t.text in ('(', '[', '{'):
                stack.append([t.text, code[i-1].text if i else '', 0, i])
            elif t.text in (')', ']', '}'):
                if stack:
                    stack.pop()
            elif t.text in (',', ';') and stack and stack[-1][0] == '(':
                stack[-1][2] += 1
        call = next((s for s in reversed(stack) if s[0] == '(' and s[1] not in KEYWORDS), None)
        if not call:
            return None
        if call[3] >= 2 and code[call[3]-2].text in ('.', '->'):
            typ = self.workspace.receiver_type(doc, code[:call[3]-2])
            signatures = [(s.signature, s.doc, d.uri) for d, s in self.workspace.members(doc, typ or '') if s.name == call[1] and s.kind == 12]
        else:
            signatures = self.workspace.signatures(call[1], doc)
        items = []
        for sig, documentation, _ in signatures:
            arguments = sig[sig.find('(')+1:sig.rfind(')')]
            # Only split top-level separators (defaults may contain calls/arrays).
            parameters, start, depth = [], 0, 0
            for i, character in enumerate(arguments):
                if character in '([{':
                    depth += 1
                elif character in ')]}':
                    depth -= 1
                elif character in ',;' and depth == 0:
                    parameters.append(arguments[start:i].strip())
                    start = i+1
            if arguments[start:].strip():
                parameters.append(arguments[start:].strip())
            items.append({'label': sig, 'parameters': [{'label': label} for label in parameters],
                          'documentation': {'kind': 'markdown', 'value': documentation}})
        active = next((i for i, item in enumerate(items) if len(item['parameters']) > call[2]), 0)
        return {'signatures': items, 'activeSignature': active, 'activeParameter': call[2]} if items else None


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
    request_id = 0

    def send(message):
        body = json.dumps(message, ensure_ascii=False).encode('utf-8')
        # A result and a background notification must never interleave frames.
        with output_lock:
            sys.stdout.buffer.write(('Content-Length: '+str(len(body))+'\r\n\r\n').encode('ascii')+body)
            sys.stdout.buffer.flush()

    def request(method, params):
        nonlocal request_id
        request_id += 1
        send({'jsonrpc': '2.0', 'id': 'ko-vex-'+str(request_id), 'method': method, 'params': params})

    server = Server(lambda method, params: send({'jsonrpc': '2.0', 'method': method, 'params': params}), request)
    try:
        while True:
            try:
                message = read_message(sys.stdin.buffer)
                if message is None:
                    return
                if 'method' not in message:
                    if message.get('error'):
                        print('Client rejected VEX capability registration: '+str(message['error']), file=sys.stderr)
                    continue
                try:
                    result = server.handle(message['method'], message.get('params'))
                    if 'id' in message:
                        send({'jsonrpc': '2.0', 'id': message['id'], 'result': result})
                except Exception as error:
                    if 'id' in message:
                        code = -32601 if isinstance(error, KeyError) else -32803 if isinstance(error, RefactorError) else -32603
                        send({'jsonrpc': '2.0', 'id': message['id'], 'error': {'code': code, 'message': str(error)}})
                    else:
                        print(str(error), file=sys.stderr)
            except (ValueError, OSError):
                traceback.print_exc(file=sys.stderr)
                return
    finally:
        server.close()
