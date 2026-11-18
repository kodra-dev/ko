"""Offline VEX tooling tests. Optional vcc tests use KO_VEX_TEST_HFS (no hou/UI/provider)."""
from pathlib import Path
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

TOOL = Path(__file__).resolve().parents[2] / 'tools/vex-lsp'
sys.path.insert(0, str(TOOL))
from ko_vex_lsp.model import Document, path_uri, uri_path
from ko_vex_lsp.workspace import Workspace
from ko_vex_lsp.server import Server, main as run_server, read_message
from ko_vex_lsp.compiler import RefactorError, rename, source_for
from ko_vex_lsp.discovery import find_houdini


class SourceTests(unittest.TestCase):
    def doc(self, text):
        return Document(path_uri(Path(tempfile.gettempdir()) / 'vex-test.vfl'), text, 1)

    def test_utf16_roundtrip(self):
        d = self.doc('// 🦊 é\nfloat value = 1;\n')
        for i in range(len(d.text)+1):
            self.assertEqual(d.offset(d.position(i)), i)
        self.assertEqual(d.position(5)['character'], 6)

    def test_uri_roundtrip(self):
        p = Path(tempfile.gettempdir()) / 'unicode é # %.vfl'
        self.assertEqual(uri_path(path_uri(p)), p.resolve())

    def test_shadowing_and_parameters(self):
        d = self.doc('function float f(float x; float y) { float a=x; { float x=y; a+=x; } return x+a; }')
        xs = [t for t in d.code if t.text == 'x']
        self.assertEqual(len(xs), 5)
        self.assertIs(d.resolve(xs[0]), d.resolve(xs[1]))
        self.assertIs(d.resolve(xs[2]), d.resolve(xs[3]))
        self.assertIs(d.resolve(xs[0]), d.resolve(xs[4]))
        self.assertIsNot(d.resolve(xs[0]), d.resolve(xs[2]))

    def test_arrays_multi_declaration_and_comments(self):
        d = self.doc('float a[]=array(1.,2.), b=3; // a\nstring text="a"; b += a[0];')
        self.assertEqual([(s.name, s.type) for s in d.symbols], [('a', 'float[]'), ('b', 'float'), ('text', 'string')])
        self.assertEqual(len([t for t in d.code if t.text == 'a']), 2)

    def test_for_and_foreach_scopes(self):
        d = self.doc('int i=9; for(int i=0; i<2; i++){ printf("%g", i); } i++; foreach(int j; float v; array(1.,2.)){ printf("%g",v); }')
        its = [t for t in d.code if t.text == 'i']
        self.assertIs(d.resolve(its[0]), d.resolve(its[-1]))
        for t in its[1:-1]:
            self.assertIs(d.resolve(t), d.resolve(its[1]))
        self.assertIsNot(d.resolve(its[0]), d.resolve(its[1]))
        self.assertEqual([s.name for s in d.symbols], ['i', 'i', 'j', 'v'])

    def test_members_and_attributes_are_not_locals(self):
        d = self.doc('float weight=1; f@weight=weight; vector P=0; P.x=weight;')
        attribute = next(t for t in d.tokens if t.kind == 'attribute')
        member = next(t for t in d.tokens if t.text == 'x')
        self.assertIsNone(d.resolve(attribute))
        self.assertIsNone(d.resolve(member))

    def test_error_and_unsupported_scope(self):
        self.assertTrue(self.doc('float a=(1;').errors)
        self.assertTrue(self.doc('for(int i=0;i<1;i++) printf("x");').unsupported)
        self.assertTrue(self.doc('struct S { float x; };').unsupported)

    def test_wrapper_preserves_attribute_width(self):
        d = self.doc('v@P *= 2; i@thing = @ptnum;')
        source, lines = source_for(d, 'snippet')
        self.assertIn('_P *= 2; __thing = _ptnum;', source)
        self.assertEqual(lines, 1)
        self.assertIn('export vector __P', source)
        with self.assertRaises(RefactorError):
            source_for(self.doc('@unknown *= 2;'), 'snippet')


class WorkspaceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.inc = self.root / 'inc'
        self.inc.mkdir()
        self.ws = Workspace(self.root, {'includePaths': ['inc'], 'discoverHoudini': False})
        self.doc = Document(path_uri(self.root / 'main.vfl'), '#include <custom.h>\nfloat x = helper(2);', 1)
        (self.inc / 'custom.h').write_text('// Double it\nfunction float helper(float value) { return value*2; }\n#include "nested.h"\n')
        (self.inc / 'nested.h').write_text('#include "custom.h"\nfunction int other(int v){return v;}\n')

    def test_transitive_cycles_and_signature(self):
        self.assertEqual(len(self.ws.graph(self.doc)), 3)
        sigs = self.ws.signatures('helper', self.doc)
        self.assertEqual(len(sigs), 1)
        self.assertIn('float value', sigs[0][0])
        self.assertIn('Double it', sigs[0][1])

    def test_quoted_include_precedence(self):
        (self.root / 'custom.h').write_text('function float nearby(float a){return a;}')
        self.assertEqual(uri_path(self.ws.resolve_include(self.doc, 'custom.h', True)), self.root / 'custom.h')
        self.assertEqual(uri_path(self.ws.resolve_include(self.doc, 'custom.h', False)), self.inc / 'custom.h')

    def test_unsaved_header_and_disk_invalidation(self):
        uri = path_uri(self.inc / 'custom.h')
        self.ws.open[uri] = Document(uri, 'function float unsaved(float a){return a;}', 5)
        self.assertTrue(self.ws.signatures('unsaved', self.doc))
        self.assertFalse(self.ws.signatures('helper', self.doc))
        self.ws.open.pop(uri)
        self.assertTrue(self.ws.signatures('helper', self.doc))
        (self.inc / 'custom.h').write_text('function int changed(int x){return x;}')
        self.assertTrue(self.ws.signatures('changed', self.doc))
        self.assertFalse(self.ws.signatures('helper', self.doc))

    def test_comments_do_not_include(self):
        d = Document(self.doc.uri, '/*\n#include <missing.h>\n*/\n// #include <missing.h>\n')
        self.assertEqual(list(self.ws.includes(d)), [])

    def test_include_completion(self):
        self.assertEqual(self.ws.include_completions(self.doc, 'cust', False), ['custom.h'])


class ProtocolTests(WorkspaceTests):
    def setUp(self):
        super().setUp()
        self.notifications = []
        self.server = Server(lambda method, params: self.notifications.append((method, params)))
        self.server.handle('initialize', {'rootUri': path_uri(self.root), 'initializationOptions': {'includePaths': ['inc'], 'discoverHoudini': False}})
        self.server.handle('textDocument/didOpen', {'textDocument': {'uri': self.doc.uri, 'text': self.doc.text, 'version': 1}})

    def request(self, method, offset=None, **extra):
        doc = self.server.workspace.document(self.doc.uri)
        params = {'textDocument': {'uri': doc.uri}, **extra}
        if offset is not None:
            params['position'] = doc.position(offset)
        return self.server.handle(method, params)

    def test_header_definition_and_function_completion(self):
        found = self.request('textDocument/definition', self.doc.text.index('custom'))
        self.assertEqual(uri_path(found[0]['uri']), self.inc / 'custom.h')
        found = self.request('textDocument/definition', self.doc.text.index('helper')+1)
        self.assertEqual(uri_path(found[0]['uri']), self.inc / 'custom.h')
        result = self.request('textDocument/completion', self.doc.text.index('helper')+3)
        self.assertIn('helper', [item['label'] for item in result['items']])

    def test_hover_and_signature(self):
        hover = self.request('textDocument/hover', self.doc.text.index('helper')+1)
        self.assertIn('Double it', hover['contents']['value'])
        help = self.request('textDocument/signatureHelp', self.doc.text.index('helper(2')+8)
        self.assertIn('helper', help['signatures'][0]['label'])

    def open_text(self, text):
        self.server.handle('textDocument/didChange', {'textDocument': {'uri': self.doc.uri, 'version': 2}, 'contentChanges': [{'text': text}]})

    def test_header_struct_members_and_method_signature(self):
        (self.inc/'custom.h').write_text('struct Particle { vector position; float mass; float scaled(float factor){return mass*factor;} };')
        text = '#include <custom.h>\nParticle item; item.mass = 1; item->scaled(2); item.po'
        self.open_text(text)
        result = self.request('textDocument/completion', len(text))
        self.assertEqual([item['label'] for item in result['items']], ['position'])
        definition = self.request('textDocument/definition', text.index('.mass')+2)
        self.assertEqual(uri_path(definition[0]['uri']), self.inc/'custom.h')
        hover = self.request('textDocument/hover', text.index('.mass')+2)
        self.assertIn('float mass', hover['contents']['value'])
        signature = self.request('textDocument/signatureHelp', text.index('scaled(2')+8)
        self.assertEqual(signature['signatures'][0]['parameters'], [{'label': 'float factor'}])

    def test_vector_attribute_and_macro_completion(self):
        self.open_text('vector pos=0; pos.')
        result = self.request('textDocument/completion', len('vector pos=0; pos.'))
        self.assertEqual([item['label'] for item in result['items']], ['x', 'y', 'z'])
        self.server.workspace.open[self.doc.uri] = Document(self.doc.uri, 'v@P = 0; @pt', 3)
        result = self.request('textDocument/completion', len('v@P = 0; @pt'))
        self.assertIn('ptnum', [item['label'] for item in result['items']])
        (self.inc/'custom.h').write_text('#define MY_FACTOR 2\nconst float custom_global=3;')
        text = '#include <custom.h>\nfloat x=MY_FACTOR + custom_global;'
        self.server.workspace.open[self.doc.uri] = Document(self.doc.uri, text, 4)
        found = self.request('textDocument/definition', text.index('MY_FACTOR')+2)
        self.assertEqual(uri_path(found[0]['uri']), self.inc/'custom.h')
        found = self.request('textDocument/definition', text.index('custom_global')+2)
        self.assertEqual(uri_path(found[0]['uri']), self.inc/'custom.h')

    def test_no_completion_while_typing_a_number(self):
        self.server.workspace.builtins = {'fit10': [{'return': 'float', 'args': ['float']}],
                                          'atan2': [{'return': 'float', 'args': ['float', 'float']}]}
        def complete(text):
            self.server.workspace.open[self.doc.uri] = Document(self.doc.uri, text, 3)
            return [i['label'] for i in self.request('textDocument/completion', len(text))['items']]
        for text in ('float b = a * 2', 'float b = 2.0', 'float b = .5', 'float b = 1e5', 'float b = 2f'):
            self.assertEqual(complete(text), [], text)
        self.assertIn('fit10', complete('float b = fit1'))
        self.assertIn('atan2', complete('float b = atan2'))

    def test_builtin_completion_documentation_is_lazy(self):
        ws = self.server.workspace
        ws.builtins = {'fit': [{'return': 'float', 'args': ['float', 'float']}]}
        self.open_text('fit')
        result = self.request('textDocument/completion', 3)
        self.assertEqual(ws.doc_cache, {})
        item = next(item for item in result['items'] if item['label'] == 'fit')
        resolved = self.server.handle('completionItem/resolve', item)
        self.assertIn('SideFX', resolved['documentation']['value'])

    def test_refused_rename_displays_reason_and_preserves_rpc_failure(self):
        text = 'float weight=1; printf("%g",weight);'
        for method in ('textDocument/prepareRename', 'textDocument/rename'):
            for reason in ("'for' is a VEX keyword; choose a different name.",
                           "'other' already occurs in this compilation unit; choose an unambiguous name.",
                           'Only resolved local variables and function parameters can currently be renamed.'):
                with self.subTest(method=method, reason=reason):
                    def packet(message):
                        body = json.dumps(message).encode('utf-8')
                        return b'Content-Length: ' + str(len(body)).encode() + b'\r\n\r\n' + body
                    params = {'textDocument': {'uri': self.doc.uri}, 'position': {'line': 0, 'character': 7}}
                    if method.endswith('/rename'):
                        params['newName'] = 'for'
                    incoming = io.BytesIO(b''.join(packet(message) for message in [
                        {'jsonrpc': '2.0', 'id': 1, 'method': 'initialize', 'params': {
                            'rootUri': path_uri(self.root), 'initializationOptions': {'discoverHoudini': False}}},
                        {'jsonrpc': '2.0', 'method': 'textDocument/didOpen', 'params': {
                            'textDocument': {'uri': self.doc.uri, 'version': 1, 'text': text}}},
                        {'jsonrpc': '2.0', 'id': 2, 'method': method, 'params': params},
                    ]))
                    outgoing = io.BytesIO()
                    with patch('ko_vex_lsp.server.rename', side_effect=RefactorError(reason)), \
                         patch('ko_vex_lsp.server.sys.stdin', SimpleNamespace(buffer=incoming)), \
                         patch('ko_vex_lsp.server.sys.stdout', SimpleNamespace(buffer=outgoing)):
                        run_server()
                    outgoing.seek(0)
                    messages = []
                    while (message := read_message(outgoing)) is not None:
                        messages.append(message)
                    warnings = [m['params'] for m in messages if m.get('method') == 'window/showMessage']
                    self.assertEqual(warnings, [{'type': 2, 'message': 'VEX rename refused: ' + reason}])
                    self.assertTrue(any(m.get('method') == 'window/logMessage' and reason in m['params']['message'] for m in messages))
                    response = next(m for m in messages if m.get('id') == 2)
                    self.assertEqual(response['error'], {'code': -32803, 'message': reason})
                    self.assertNotIn('result', response)

    def test_successful_rename_does_not_display_refusal_warning(self):
        result = {'documentChanges': []}
        with patch('ko_vex_lsp.server.rename', return_value=result):
            self.assertEqual(self.request('textDocument/rename', self.doc.text.index('x'), newName='renamed'), result)
        self.assertFalse(any(method == 'window/showMessage' for method, _ in self.notifications))

    def test_incremental_changes_and_versions(self):
        offset = self.doc.text.index('helper')
        self.server.handle('textDocument/didChange', {'textDocument': {'uri': self.doc.uri, 'version': 2}, 'contentChanges': [
            {'range': self.doc.range(offset, offset+6), 'text': 'other'}]})
        self.assertIn('other(2)', self.server.workspace.document(self.doc.uri).text)
        self.server.handle('textDocument/didChange', {'textDocument': {'uri': self.doc.uri, 'version': 1}, 'contentChanges': [{'text': 'stale'}]})
        self.assertNotEqual(self.server.workspace.document(self.doc.uri).text, 'stale')

    def test_framing_unicode_and_truncation(self):
        body = json.dumps({'text': '🦊'}, ensure_ascii=False).encode()
        packet = b'Content-Length: ' + str(len(body)).encode() + b'\r\n\r\n' + body
        self.assertEqual(read_message(io.BytesIO(packet))['text'], '🦊')
        with self.assertRaises(ValueError):
            read_message(io.BytesIO(packet[:-1]))

    def test_stdio_process(self):
        def packet(data):
            body = json.dumps(data).encode()
            return b'Content-Length: ' + str(len(body)).encode() + b'\r\n\r\n' + body
        data = b''.join(packet(m) for m in [
            {'jsonrpc': '2.0', 'id': 1, 'method': 'initialize', 'params': {'rootUri': path_uri(self.root), 'initializationOptions': {'discoverHoudini': False}}},
            {'jsonrpc': '2.0', 'id': 2, 'method': 'shutdown'},
            {'jsonrpc': '2.0', 'method': 'exit'}])
        result = subprocess.run([sys.executable, str(TOOL/'run.py')], input=data, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        stream = io.BytesIO(result.stdout)
        self.assertTrue(read_message(stream)['result']['capabilities']['hoverProvider'])
        self.assertEqual(read_message(stream)['id'], 2)


@unittest.skipUnless(os.environ.get('KO_VEX_TEST_HFS'), 'Set KO_VEX_TEST_HFS for local compiler smoke tests')
class CompilerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.ws = Workspace(self.root, {'houdiniPath': os.environ['KO_VEX_TEST_HFS'], 'mode': 'snippet'})

    def refactor(self, text, old, new, occurrence=0):
        doc = Document(path_uri(self.root/'main.vfl'), text, 7)
        token = [t for t in doc.code if t.text == old][occurrence]
        return rename(self.ws, doc, token, new)

    def apply(self, text, result):
        doc = Document(path_uri(self.root/'main.vfl'), text)
        for edit in reversed(result['documentChanges'][0]['edits']):
            a, b = doc.offset(edit['range']['start']), doc.offset(edit['range']['end'])
            text = text[:a] + edit['newText'] + text[b:]
        return text

    def test_installed_builtins_and_docs(self):
        self.assertIn('fit', self.ws.builtins)
        self.assertIn('range', self.ws.documentation('fit').lower())

    def test_case_one_builtin_completion_hover_and_definition(self):
        server = Server()
        self.addCleanup(server.close)
        server.workspace = self.ws
        self.ws.documentation_dir = self.root/'docs'
        text = 'float value = fit(0.5, 0, 1, 0, 10);\nv@P *= value;'
        uri = path_uri(self.root/'case1.vfl')
        server.handle('textDocument/didOpen', {'textDocument': {'uri': uri, 'text': text, 'version': 1}})
        doc = self.ws.document(uri)
        position = doc.position(text.index('fit')+2)
        params = {'textDocument': {'uri': uri}, 'position': position}
        completion = server.handle('textDocument/completion', params)
        self.assertIn('fit', [item['label'] for item in completion['items']])
        hover = server.handle('textDocument/hover', params)
        self.assertIn('fit(', hover['contents']['value'])
        self.assertIn('SideFX', hover['contents']['value'])
        definition = server.handle('textDocument/definition', params)
        self.assertTrue(uri_path(definition[0]['uri']).is_file())
        # Anywhere inside the argument list falls back to the enclosing call.
        name_range = hover['range']
        for offset in (text.index('(')+1, text.index('0.5'), text.index(', 0,')+1, text.index('10)')+2):
            inner = server.handle('textDocument/hover', {'textDocument': {'uri': uri}, 'position': doc.position(offset)})
            self.assertIn('fit(', inner['contents']['value'], offset)
            self.assertEqual(inner['range'], name_range)
        # Variables inside arguments keep their own hover; outside calls has none.
        own = 'float a = 1;\nfloat b = fit(a, 0, 1, 0, 10);\n'
        server.handle('textDocument/didOpen', {'textDocument': {'uri': uri, 'text': own, 'version': 2}})
        doc = self.ws.document(uri)
        inner = server.handle('textDocument/hover', {'textDocument': {'uri': uri}, 'position': doc.position(own.index('(a')+1)})
        self.assertIn('float a', inner['contents']['value'])
        outside = server.handle('textDocument/hover', {'textDocument': {'uri': uri}, 'position': doc.position(own.index('1;'))})
        self.assertIsNone(outside)

    def test_local_rename_ignores_comments_strings_attributes(self):
        text = 'float weight=2; f@weight=weight; printf("weight %g", weight); // weight\n'
        result = self.refactor(text, 'weight', 'strength')
        self.assertEqual(result['documentChanges'][0]['textDocument']['version'], 7)
        self.assertEqual(self.apply(text, result), 'float strength=2; f@weight=strength; printf("weight %g", strength); // weight\n')

    def test_shadowed_variables(self):
        text = 'float x=1; {float x=2; printf("%g",x);} printf("%g",x);'
        result = self.refactor(text, 'x', 'outer')
        self.assertEqual(len(result['documentChanges'][0]['edits']), 2)

    def test_function_parameter(self):
        text = 'function float twice(float value){return value*2;} float output=twice(2);'
        result = self.refactor(text, 'value', 'amount')
        self.assertEqual(len(result['documentChanges'][0]['edits']), 2)

    def test_loop_variable(self):
        text = 'float sum=0; for(int i=0; i<3; i++){sum+=i;} printf("%g",sum);'
        result = self.refactor(text, 'i', 'index')
        self.assertEqual(len(result['documentChanges'][0]['edits']), 4)

    def test_reject_capture_keywords_and_invalid_source(self):
        for text, old, new, reason in [
                ('float a=1,b=2; printf("%g",a+b);', 'a', 'b', "'b' already occurs"),
                ('float a=1;', 'a', 'for', "'for' is a VEX keyword"),
                ('float a=1;', 'a', '1bad', "'1bad' is not a valid VEX identifier"),
                ('float a=missing();', 'a', 'valid', 'undefined function')]:
            with self.subTest(new_name=new), self.assertRaisesRegex(RefactorError, reason):
                self.refactor(text, old, new)

    def test_reject_macro_and_members(self):
        for text in ('#define VALUE 1\nfloat a=VALUE;', 'struct S {float x;}; float a=1;'):
            with self.assertRaises(RefactorError):
                self.refactor(text, 'a', 'b')

    def test_string_dictionary_and_array_attributes(self):
        for attribute in ('s@name="thing";', 'd@meta=set("a",1);', 'v[]@items=array({1,0,0});'):
            with self.subTest(attribute=attribute):
                text = attribute + ' float weight=2; printf("%g",weight);'
                result = self.refactor(text, 'weight', 'strength')
                self.assertEqual(len(result['documentChanges'][0]['edits']), 2)

    def test_unicode_columns_and_crlf(self):
        text = 'string label="é🦊"; float number=1; printf("%s %g",label,number);\r\n'
        result = self.refactor(text, 'number', 'amount')
        self.assertEqual(len(result['documentChanges'][0]['edits']), 2)
        self.assertIn('float amount', self.apply(text, result))

    def test_unsaved_header_snapshot(self):
        path = self.root/'custom.h'
        path.write_text('function float helper(){return 0;}')
        uri = path_uri(path)
        self.ws.open[uri] = Document(uri, 'function float helper(float value){return value;}', 2)
        self.ws.include_paths.insert(0, self.root)
        text = '#include <custom.h>\nfloat result=helper(2); printf("%g",result);'
        result = self.refactor(text, 'result', 'answer')
        self.assertEqual(len(result['documentChanges'][0]['edits']), 2)
        self.assertNotIn('value', path.read_text())

    def test_source_save_compiler_diagnostics(self):
        notifications = []
        server = Server(lambda method, params: notifications.append((method, params)))
        self.addCleanup(server.close)
        server.handle('initialize', {'rootUri': path_uri(self.root), 'initializationOptions': {
            'houdiniPath': os.environ['KO_VEX_TEST_HFS'], 'mode': 'source', 'compilerDiagnostics': True}})
        uri = path_uri(self.root/'main.vfl')
        text = 'cvex test(){float result=missing();}'
        server.handle('textDocument/didOpen', {'textDocument': {'uri': uri, 'text': text, 'version': 1}})
        server.handle('textDocument/didSave', {'textDocument': {'uri': uri}})
        self.assertTrue(server.checker.wait_idle(15))
        diagnostics = notifications[-1][1]['diagnostics']
        error = next(d for d in diagnostics if d['source'] == 'vcc')
        self.assertEqual(error['range']['start']['line'], 0)
        self.assertEqual(error['code'], 1066)
        self.assertIn('missing', error['message'])

    def test_reject_macro_generated_include(self):
        (self.root/'custom.h').write_text('#define HEADER "nested.h"\n#include HEADER\n')
        self.ws.include_paths.insert(0, self.root)
        with self.assertRaisesRegex(RefactorError, 'complete include graph'):
            self.refactor('#include <custom.h>\nfloat value=1;', 'value', 'amount')

    def test_reject_installation_header_and_scope_disagreement(self):
        doc = Document(path_uri(self.ws.hfs/'houdini/vex/include/never-written.h'),
                       'function float f(float value){return value;}', 1)
        with self.assertRaisesRegex(RefactorError, 'read-only'):
            rename(self.ws, doc, next(t for t in doc.code if t.text == 'value'), 'amount')
        text = 'float value=1; {float value=2; printf("%g",value);} printf("%g",value);'
        doc = Document(path_uri(self.root/'main.vfl'), text, 1)
        tokens = [t for t in doc.code if t.text == 'value']
        outer = doc.resolve(tokens[0])
        real_resolve = doc.resolve
        doc.resolve = lambda token: outer if token is tokens[2] else real_resolve(token)
        with self.assertRaisesRegex(RefactorError, 'scope bindings disagree'):
            rename(self.ws, doc, tokens[0], 'amount')
        self.assertEqual(doc.text, text)

    def test_reject_shader_interface_parameter(self):
        self.ws.mode = 'source'
        with self.assertRaisesRegex(RefactorError, 'Externally bound'):
            self.refactor('cvex test(export float value=0){ value+=1; }', 'value', 'amount')

    def test_official_and_custom_headers(self):
        (self.root/'custom.h').write_text('function float helper(float value){return value*2;}')
        self.ws.include_paths.insert(0, self.root)
        text = '#include <kinefx.h>\n#include <custom.h>\nfloat result=helper(2); printf("%g", result);'
        result = self.refactor(text, 'result', 'answer')
        self.assertEqual(len(result['documentChanges'][0]['edits']), 2)


class DiscoveryTests(unittest.TestCase):
    def test_versions_environment_and_explicit_precedence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            base = root/'program files/Side Effects Software'
            for version in ('22.0.99', '22.0.466', '21.0.999'):
                compiler = base/('Houdini '+version)/'bin/vcc.exe'
                compiler.parent.mkdir(parents=True)
                compiler.touch()
            env = {'ProgramFiles': str(base.parent)}
            latest = base/'Houdini 22.0.466'
            self.assertEqual(find_houdini({}, root, env, 'nt'), latest)
            env['HFS'] = str(base/'Houdini 21.0.999')
            self.assertEqual(find_houdini({}, root, env, 'nt'), Path(env['HFS']))
            env['KO_VEX_HOUDINI'] = str(latest)
            self.assertEqual(find_houdini({}, root, env, 'nt'), latest)
            self.assertEqual(find_houdini({'houdiniPath': 'missing-explicit'}, root, env, 'nt'), root/'missing-explicit')
            self.assertIsNone(find_houdini({'discoverHoudini': False}, root, env, 'nt'))

    def test_normal_zed_settings_without_project_file(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'include').mkdir()
            server = Server()
            server.handle('initialize', {'rootUri': path_uri(root), 'initializationOptions': {'discoverHoudini': False}})
            self.assertIn(root/'include', server.workspace.include_paths)
            (root/'custom').mkdir()
            server.handle('workspace/didChangeConfiguration', {'settings': {'ko-vex': {
                'includePaths': ['custom'], 'discoverHoudini': False}}})
            self.assertIn(root/'custom', server.workspace.include_paths)
            self.assertFalse((root/'vex-lsp.json').exists())
            self.assertFalse((root/'.zed').exists())


@unittest.skipUnless(os.name == 'nt' and shutil.which('pwsh') and shutil.which('git'), 'Windows packaging tests')
class PackagingTests(unittest.TestCase):
    def assert_discovery_permission(self, extension):
        import tomllib
        manifest = tomllib.loads((extension/'extension.toml').read_text(encoding='utf-8'))
        runtime = (extension/'src/runtime.rs').read_text(encoding='utf-8')
        program = json.loads(re.search(r'pub const DISCOVERY_PROGRAM: &str = ("[^"]+");', runtime)[1])
        argument_source = re.search(r'pub const DISCOVERY_ARGS:.*?= &\[(.*?)\];', runtime, re.S)[1]
        prefix = [json.loads(value) for value in re.findall(r'"(?:\\.|[^"\\])*"', argument_source.split('include_str!')[0])]
        # Read bytes: include_str! preserves CRLF, unlike Python's read_text().
        script = (extension/'src/discover_runtime.ps1').read_bytes().decode('utf-8')
        capabilities = manifest.get('capabilities', [])
        self.assertEqual(capabilities, [{'kind': 'process:exec', 'command': program, 'args': prefix + [script]}])
        self.assertNotIn('*', capabilities[0]['args'])
        self.assertNotIn('**', capabilities[0]['args'])
        # Prevent silently switching the request back to an absolute PATH result:
        # Zed's capability matcher compares the requested command literally.
        lib = (extension/'src/lib.rs').read_text(encoding='utf-8')
        self.assertIn('zed::process::Command::new(runtime::DISCOVERY_PROGRAM)', lib)
        self.assertIn('.args(runtime::DISCOVERY_ARGS.iter().copied())', lib)

    def test_default_output_is_repo_local_and_gitignored(self):
        with tempfile.TemporaryDirectory(prefix='ko-vex-default-package-') as directory:
            root = Path(directory)
            repo = TOOL.parents[1]
            for relative in ('editors/zed-houdini', 'editors/tree-sitter-vex', 'editors/tree-sitter-opencl',
                             'tools/vex-lsp', 'tools/apex-lsp', 'tools/opencl-lsp'):
                shutil.copytree(repo/relative, root/relative,
                                ignore=shutil.ignore_patterns('target', 'node_modules', '__pycache__'))
            (root/'scripts').mkdir()
            shutil.copy2(repo/'scripts/houdini_package.ps1', root/'scripts/houdini_package.ps1')
            shutil.copy2(repo/'.gitignore', root/'.gitignore')
            subprocess.run(['git', '-C', str(root), 'init', '--quiet'], check=True, capture_output=True)
            # Defaults must follow the script's repository, not the caller's cwd.
            command = [shutil.which('pwsh'), '-NoProfile', '-File', str(root/'scripts/houdini_package.ps1')]
            result = subprocess.run(command, cwd=repo, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)
            generated = root/'build/zed-houdini'
            self.assertTrue((generated/'zed-extension/extension.toml').is_file())
            self.assert_discovery_permission(generated/'zed-extension')
            for relative in ('zed-extension/extension.toml', 'grammar/src/parser.c', 'grammar-opencl/src/parser.c'):
                ignored = subprocess.run(['git', '-C', str(root), 'check-ignore', '--quiet', str(generated/relative)])
                self.assertEqual(ignored.returncode, 0, relative)
            rejected = subprocess.run(command + ['-Destination', str(root/'editors/zed-houdini')],
                                      capture_output=True, text=True, timeout=30)
            self.assertNotEqual(rejected.returncode, 0)
            self.assertFalse((root/'editors/zed-houdini/zed-extension').exists())

    def test_package_is_self_contained_and_requires_no_workspace_setup(self):
        with tempfile.TemporaryDirectory(prefix='ko-vex-package-') as directory:
            root = Path(directory)
            destination = root/'cache with spaces'
            command = [shutil.which('pwsh'), '-NoProfile', '-File', str(TOOL.parents[1]/'scripts/houdini_package.ps1'),
                       '-Destination', str(destination)]
            for attempt in range(2):
                result = subprocess.run(command, capture_output=True, text=True, timeout=30)
                self.assertEqual(result.returncode, 0, result.stderr)
                if attempt == 0:
                    (destination/'zed-extension/server/ko_vex_lsp/stale.py').write_text('obsolete')
                    (destination/'grammar/src/stale.c').write_text('obsolete')
            extension = destination/'zed-extension'
            self.assert_discovery_permission(extension)
            self.assertFalse((extension/'server/ko_vex_lsp/stale.py').exists())
            self.assertFalse((destination/'grammar/src/stale.c').exists())
            manifest = (extension/'extension.toml').read_text()
            self.assertNotIn('@GRAMMAR_', manifest)
            self.assertIn('file:///', manifest)
            self.assertIn('%20', manifest)
            self.assertTrue((extension/'server/ko_vex_lsp/discovery.py').is_file())
            self.assertTrue((extension/'server/ko_apex_lsp/catalog.py').is_file())
            self.assertTrue((extension/'languages/apex/config.toml').is_file())
            self.assertIn('[language_servers.ko-apex]', manifest)
            self.assertTrue((extension/'server/ko_opencl_lsp/builtins.py').is_file())
            self.assertTrue((extension/'languages/opencl/config.toml').is_file())
            self.assertIn('[language_servers.ko-opencl]', manifest)
            self.assertIn('[grammars.houdini_opencl]', manifest)
            self.assertTrue((destination/'grammar-opencl/src/parser.c').is_file())
            # Each grammar has its own cache commit; the manifest pins both revisions.
            import tomllib
            grammars = tomllib.loads(manifest)['grammars']
            self.assertNotEqual(grammars['vex']['rev'], grammars['houdini_opencl']['rev'])
            self.assertTrue(grammars['houdini_opencl']['repository'].endswith('grammar-opencl'))
            # .h stays with C/C++; only .cl is claimed.
            config = tomllib.loads((extension/'languages/opencl/config.toml').read_text(encoding='utf-8'))
            self.assertEqual(config['path_suffixes'], ['cl'])
            self.assertTrue(manifest.startswith('id = "ko-houdini"\n'))
            self.assertNotIn('[grammars.python]', manifest)
            self.assertTrue((extension/'build.rs').is_file())
            self.assertFalse(list(destination.rglob('settings.json')))
            self.assertFalse(list(destination.rglob('vex-lsp.json')))
            # Bootstrap only the packaged copy, with script-directory imports disabled,
            # using the same bootstrap source the extension sends.
            runtime = (extension/'src/runtime.rs').read_text(encoding='utf-8')
            bootstrap = json.loads(re.search(r'pub const BOOTSTRAP: &str = ("(?:\\.|[^"\\])*");', runtime)[1])
            body = json.dumps({'jsonrpc':'2.0','id':1,'method':'initialize','params': {
                'rootUri':path_uri(root), 'initializationOptions':{'discoverHoudini':False}}}).encode()
            for package, name in (('ko_vex_lsp', 'ko-vex-lsp'), ('ko_apex_lsp', 'ko-apex-lsp'),
                                  ('ko_opencl_lsp', 'ko-opencl-lsp')):
                result = subprocess.run([sys.executable, '-I', '-S', '-c', bootstrap, str(extension/'server'), package],
                                        input=b'Content-Length: '+str(len(body)).encode()+b'\r\n\r\n'+body,
                                        capture_output=True, timeout=15)
                self.assertEqual(result.returncode, 0, result.stderr)
                stream, messages = io.BytesIO(result.stdout), []
                while (message := read_message(stream)) is not None:
                    messages.append(message)
                response = next(m for m in messages if m.get('id') == 1)
                self.assertEqual(response['result']['serverInfo']['name'], name)

    def test_native_probe_chooses_supported_houdini_python(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for version in ('22.0.99','22.0.466'):
                hfs = root/('Houdini '+version)
                (hfs/'bin').mkdir(parents=True)
                (hfs/'bin/vcc.exe').touch()
                (hfs/'python313').mkdir()
                (hfs/'python313/python.exe').touch()
            script = TOOL.parents[1]/'editors/zed-houdini/src/discover_runtime.ps1'
            result = subprocess.run([shutil.which('pwsh'), '-NoProfile', '-File', str(script), '-SearchRoot', str(root)],
                                    capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stderr)
            chosen = json.loads(result.stdout)
            self.assertEqual(Path(chosen['houdiniPath']), root/'Houdini 22.0.466')
            self.assertEqual(Path(chosen['python']), root/'Houdini 22.0.466/python313/python.exe')


if __name__ == '__main__':
    unittest.main()
