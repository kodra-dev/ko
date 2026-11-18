"""Offline Houdini OpenCL language-server tests (no hou/UI/provider/GPU).

A fake $HFS/houdini/ocl/include tree stands in for Houdini's headers.
Set KO_VEX_TEST_HFS to also check a real installation's OpenCL corpus.
"""
from pathlib import Path
import io
import json
import os
import sys
import tempfile
import textwrap
import unittest
from types import SimpleNamespace
from unittest.mock import patch

TOOL = Path(__file__).resolve().parents[2] / 'tools/opencl-lsp'
sys.path.insert(0, str(TOOL))
from ko_opencl_lsp import bindings as B
from ko_opencl_lsp.builtins import BUILTINS
from ko_opencl_lsp.headers import find_houdini
from ko_opencl_lsp.server import Server, main as run_server, path_uri, read_message
from ko_opencl_lsp.source import definitions, tokenize

MATRIX_H = textwrap.dedent('''\
    #ifndef __MATRIX_H__
    #define __MATRIX_H__
    #include "typedefines.h"

    // A 3x3 matrix in row-major order
    typedef fpreal3 mat3[3];

    // Set 3x3 matrix A to the identity.
    static void
    mat3identity(mat3 a)
    {
        a[0] = (fpreal3)(1, 0, 0);
    }

    /// Return b * A
    static fpreal3
    mat3vecmul(const mat3 a, const fpreal3 b)
    {
        return b.x * a[0] + b.y * a[1] + b.z * a[2];
    }
    #endif
    ''')
TYPEDEFINES_H = textwrap.dedent('''\
    #ifndef fpreal
    #define fpreal float
    #define fpreal3 float3
    #endif
    #define vload3f vload3
    ''')
INTERPOLATE_H = textwrap.dedent('''\
    #include <matrix.h>
    // Linear interpolation of two values.
    static float lerpf(float a, float b, float t) { return a + (b - a) * t; }
    ''')

SNIPPET = textwrap.dedent('''\
    #include <interpolate.h>
    #bind point &P float3
    #bind vdb surface xformtoworld float
    #bind ramp myramp float
    @KERNEL
    {
        float3 pos = @P;
        mat3 m;
        mat3identity(m);
        pos = mat3vecmul(m, pos);
        float s = @surface.worldSample(pos) * @myramp(0.5f);
        float l = length(pos) + lerpf(0, 1, s);
        @P.set(pos);
    }
    ''')


class Fixture(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='ko-opencl-')
        self.root = Path(self.directory.name)
        self.hfs = self.root / 'Houdini 22.0.1'
        include = self.hfs / 'houdini/ocl/include'
        include.mkdir(parents=True)
        (include / 'matrix.h').write_text(MATRIX_H, encoding='utf-8')
        (include / 'typedefines.h').write_text(TYPEDEFINES_H, encoding='utf-8')
        (include / 'interpolate.h').write_text(INTERPOLATE_H, encoding='utf-8')
        self.notifications = []
        self.server = Server(lambda method, params: self.notifications.append((method, params)))
        self.server.handle('initialize', {'initializationOptions': {'houdiniPath': str(self.hfs)}})

    def tearDown(self):
        self.directory.cleanup()

    def open(self, text, name='case.cl'):
        self.uri = path_uri(self.root / name)
        self.text = text
        self.server.handle('textDocument/didOpen', {'textDocument': {'uri': self.uri, 'text': text, 'version': 1}})
        return self.server.documents[self.uri]

    def at(self, needle, delta=0):
        doc = self.server.documents[self.uri]
        return doc.view.position(self.text.index(needle) + delta)

    def request(self, method, needle, delta=0):
        return self.server.handle(method, {'textDocument': {'uri': self.uri}, 'position': self.at(needle, delta)})

    def labels(self, result):
        return [item['label'] for item in (result or {}).get('items', [])]

    def published(self):
        return [p for m, p in self.notifications if m == 'textDocument/publishDiagnostics'][-1]['diagnostics']


class BindParsingTests(unittest.TestCase):
    def test_decorations_types_and_options(self):
        bind = B.parse_bind('#bind point &?P float3 input=1 val={1, 2, 3}', 0)
        self.assertEqual((bind.kind, bind.name, bind.data_type), ('point', 'P', 'float3'))
        self.assertEqual({w for w, _, _ in bind.flags}, {'write', 'opt', 'input=1', 'val={1,'})
        self.assertEqual(bind.problems, [])
        self.assertEqual(bind.binding_class, 'attribute')
        trailing = B.parse_bind('#bind point v? input=1 float3 opt', 0)
        self.assertEqual((trailing.name, trailing.problems), ('v', []))

    def test_houdini_style_type_flags_are_accepted(self):
        # SideFX's own headers bind float9; float[] / int? are documented forms.
        for flag in ('float9', 'float[]', 'int?', 'fpreal3', 'exint', 'double4', 'any'):
            self.assertEqual(B.parse_bind(f'#bind point x {flag}', 0).problems, [], flag)

    def test_group_and_topology_names(self):
        self.assertEqual(B.parse_bind('#bind prim group:inside int', 0).problems, [])
        self.assertEqual(B.parse_bind('#bind point topo:neighbours int[]', 0).name, 'topo:neighbours')

    def test_problems_and_severities(self):
        def problems(line):
            return [(severity, message) for _, _, severity, message in B.parse_bind(line, 0).problems]
        self.assertEqual(problems('#bind poitn P float3')[0][0], 1)
        self.assertEqual(problems('#bind point')[0][0], 1)
        self.assertEqual(problems('#bind point P val=')[0][0], 1)
        self.assertEqual(problems('#bind point P input=first')[0][0], 2)
        self.assertEqual(problems('#bind layer src border=REPEAT')[0][0], 2)
        self.assertEqual(problems('#bind point P read noread')[0][0], 2)
        # Undocumented words are information only: the help page may not list everything.
        self.assertEqual(problems('#bind point P flot'), [(3, '`flot` is not a documented #bind flag or type.')])
        self.assertEqual(problems('#bind ramp r size=64')[0][0], 3)
        self.assertEqual(problems('#bind point P float3 bvh pointbvh')[0][0], 2)

    def test_offsets_are_absolute(self):
        bind = B.parse_bind('#bind point &mass float', 100)
        self.assertEqual((bind.name_start, bind.name_end), (113, 117))


class SourceTests(unittest.TestCase):
    def test_tokens_keep_bindings_directives_and_comments_apart(self):
        kinds = [(t.kind, t.text) for t in tokenize('#bind point &P\n@P.set(1); // @Q\n/* @R */ "@S"')]
        self.assertIn(('directive', '#bind point &P'), kinds)
        self.assertIn(('binding', '@P'), kinds)
        self.assertNotIn(('binding', '@Q'), kinds)
        self.assertNotIn(('binding', '@R'), kinds)
        self.assertNotIn(('binding', '@S'), kinds)

    def test_definitions_kandr_prototypes_macros_and_typedefs(self):
        found = {(d.name, d.kind) for d in definitions(MATRIX_H + TYPEDEFINES_H)}
        self.assertTrue({('mat3identity', 'function'), ('mat3vecmul', 'function'), ('mat3', 'typedef'),
                         ('fpreal', 'macro'), ('vload3f', 'macro')} <= found, found)
        functions = {d.name: d for d in definitions(MATRIX_H)}
        self.assertEqual(functions['mat3vecmul'].detail, 'static fpreal3 mat3vecmul(const mat3 a, const fpreal3 b)')
        self.assertEqual(functions['mat3vecmul'].doc, 'Return b * A')
        self.assertEqual(functions['mat3vecmul'].params, ['const mat3 a', 'const fpreal3 b'])

    def test_kernels_parameters_and_locals(self):
        text = 'kernel void k(global float *P, int n)\n{\n    float3 v = 0;\n}\n@KERNEL\n{\n    int i = 1;\n}\n'
        found = {(d.name, d.kind) for d in definitions(text)}
        self.assertTrue({('k', 'kernel'), ('P', 'parameter'), ('n', 'parameter'), ('v', 'variable'),
                         ('@KERNEL', 'kernel'), ('i', 'variable')} <= found, found)


class BuiltinCatalogueTests(unittest.TestCase):
    def test_generated_catalogue_has_core_functions(self):
        for name in ('get_global_id', 'vload3', 'vstore3', 'dot', 'cross', 'clamp', 'mix', 'atomic_add', 'select'):
            self.assertIn(name, BUILTINS)
        self.assertIn('size_t get_global_id(uint)', BUILTINS['get_global_id']['sigs'])
        self.assertTrue(BUILTINS['sin']['sigs'][0].startswith('float sin(float)'))
        # Width-specific names keep exact types.
        self.assertIn('float3 vload3(size_t, const float *)', BUILTINS['vload3']['sigs'])


class ServerTests(Fixture):
    def test_binding_completion_offers_declared_global_and_kernel_markers(self):
        self.open(SNIPPET)
        result = self.request('textDocument/completion', '@P;', 1)
        labels = self.labels(result)
        self.assertEqual(labels[:3], ['@P', '@surface', '@myramp'])
        self.assertIn('@ix', labels)
        self.assertIn('@KERNEL', labels)
        item = result['items'][0]
        self.assertEqual(item['textEdit']['newText'], '@P')
        self.assertEqual(item['textEdit']['range']['start'], self.at('@P;'))

    def test_methods_follow_the_binding_class(self):
        self.open(SNIPPET)
        vdb = self.labels(self.request('textDocument/completion', '@surface.', 9))
        self.assertIn('worldSample', vdb)
        self.assertIn('indexToWorld', vdb)
        self.assertNotIn('tupleAt', vdb)
        attribute = self.labels(self.request('textDocument/completion', '@P.set', 3))
        self.assertIn('setAt', attribute)
        self.assertIn('tuplesize', attribute)
        self.assertNotIn('worldSample', attribute)

    def test_directive_completion(self):
        self.open('#bind \n#bind point P \n#runover \n#include <\n#\n')
        self.assertIn('vdb', self.labels(self.request('textDocument/completion', '#bind \n', 6)))
        flags = self.labels(self.request('textDocument/completion', '#bind point P \n', 14))
        self.assertTrue({'float3', 'write', 'nowrite', 'input=', 'opt'} <= set(flags))
        self.assertEqual(self.labels(self.request('textDocument/completion', '#runover', 9))[0], 'attribute')
        self.assertEqual(set(self.labels(self.request('textDocument/completion', '#include <', 10))),
                         {'interpolate.h', 'matrix.h', 'typedefines.h'})
        self.assertIn('bind', self.labels(self.request('textDocument/completion', '#\n', 1)))

    def test_identifier_completion_includes_headers_types_and_builtins(self):
        self.open(SNIPPET)
        labels = set(self.labels(self.request('textDocument/completion', 'mat3identity', 0)))
        # matrix.h is included by default; interpolate.h explicitly.
        self.assertTrue({'mat3identity', 'lerpf', 'fpreal3', 'length', 'get_global_id', 'pos', 'float3'} <= labels)
        resolved = self.server.handle('completionItem/resolve', {'label': 'length', 'data': {'b': 'length'}})
        self.assertIn('float length(floatn)', resolved['documentation']['value'])

    def test_include_trigger_characters_do_not_pop_up_in_code(self):
        self.open('#include <\nif (i < n) x = a / b;\n')
        for needle in ('< n', '/ b'):
            position = self.at(needle, 1)
            result = self.server.handle('textDocument/completion', {
                'textDocument': {'uri': self.uri}, 'position': position,
                'context': {'triggerKind': 2, 'triggerCharacter': needle[0]}})
            self.assertIsNone(result, needle)
        result = self.server.handle('textDocument/completion', {
            'textDocument': {'uri': self.uri}, 'position': self.at('#include <', 10),
            'context': {'triggerKind': 2, 'triggerCharacter': '<'}})
        self.assertIn('matrix.h', self.labels(result))

    def test_directive_items_replace_exactly_what_was_typed(self):
        self.open('#include <matrix.\n#bind point P float[\n#bind point Q nowr\n')
        include = self.request('textDocument/completion', 'matrix.\n', 7)
        item = next(i for i in include['items'] if i['label'] == 'matrix.h')
        self.assertEqual(item['textEdit']['range']['start'], self.at('matrix.'))
        self.assertEqual(item['textEdit']['newText'], 'matrix.h')
        flags = self.request('textDocument/completion', 'float[\n', 6)
        item = next(i for i in flags['items'] if i['label'] == 'float[]')
        self.assertEqual(item['textEdit']['range']['start'], self.at('float['))
        self.assertEqual(item['textEdit']['range']['end'], self.at('float[\n', 6))
        flags = self.request('textDocument/completion', 'nowr', 4)
        item = next(i for i in flags['items'] if i['label'] == 'nowrite')
        self.assertEqual(item['textEdit']['range']['start'], self.at('nowr'))

    def test_configuration_warning_is_shown_once_per_installation(self):
        notifications = []
        server = Server(lambda m, p: notifications.append((m, p)))
        with patch('ko_opencl_lsp.server.find_houdini', return_value=None):
            server.handle('initialize', {'initializationOptions': {}})
            for _ in range(3):
                server.handle('workspace/didChangeConfiguration', {'settings': {'ko-opencl': {'includePaths': []}}})
        self.assertEqual(len([m for m, _ in notifications if m == 'window/showMessage']), 1)

    def test_no_completion_in_comments_numbers_or_struct_members(self):
        self.open('// @P\nfloat x = 1.5;\ns.f\n')
        self.assertIsNone(self.request('textDocument/completion', '// @P', 5))
        self.assertEqual(self.labels(self.request('textDocument/completion', '1.5', 2)), [])
        self.assertIsNone(self.request('textDocument/completion', 's.f', 3))

    def test_hover(self):
        self.open(SNIPPET)
        def hover(needle, delta=1):
            result = self.request('textDocument/hover', needle, delta)
            return result and result['contents']['value']
        self.assertIn('Set 3x3 matrix A to the identity.', hover('mat3identity'))
        self.assertIn('_From matrix.h_', hover('mat3identity'))
        self.assertIn('float length(floatn)', hover('length('))
        self.assertIn('#bind vdb surface xformtoworld float', hover('@surface.'))
        self.assertIn('model/SOP-space', hover('worldSample'))
        self.assertIn('ramp', hover('@myramp('))
        self.assertIn('index space to geometry', hover('xformtoworld'))
        self.assertIn('row-major', hover('mat3 m'))

    def test_hover_on_undeclared_binding_points_to_the_bindings_tab(self):
        self.open('@KERNEL\n{\n    float d = @density;\n}\n')
        value = self.request('textDocument/hover', '@density', 2)['contents']['value']
        self.assertIn('Not declared by a `#bind`', value)
        self.assertIn('**Bindings** tab', value)

    def test_signature_help(self):
        self.open(SNIPPET)
        help_ = self.request('textDocument/signatureHelp', 'pos);\n    float s', 3)
        self.assertEqual(help_['signatures'][0]['label'], 'static fpreal3 mat3vecmul(const mat3 a, const fpreal3 b)')
        self.assertEqual(help_['activeParameter'], 1)
        self.assertEqual(self.request('textDocument/signatureHelp', '0.5f)', 0)['signatures'][0]['label'], '@myramp(pos)')
        self.assertEqual(self.request('textDocument/signatureHelp', 'pos) * @myramp', 0)['signatures'][0]['label'],
                         '@surface.worldSample(xyz)')
        builtin = self.request('textDocument/signatureHelp', 'pos) + lerpf', 0)
        self.assertIn('float length(floatn)', [s['label'] for s in builtin['signatures']])

    def test_definition(self):
        self.open(SNIPPET)
        location = self.request('textDocument/definition', 'mat3identity', 2)[0]
        self.assertTrue(location['uri'].endswith('/matrix.h'))
        self.assertEqual(location['range']['start']['line'], MATRIX_H.splitlines().index('mat3identity(mat3 a)'))
        self.assertEqual(self.request('textDocument/definition', '@P.set', 1)[0]['range']['start'], self.at('P float3'))
        self.assertTrue(self.request('textDocument/definition', '#include', 3)[0]['uri'].endswith('/interpolate.h'))

    def test_document_symbols(self):
        self.open(SNIPPET)
        names = [s['name'] for s in self.server.handle('textDocument/documentSymbol', {'textDocument': {'uri': self.uri}})]
        self.assertEqual(names, ['@P', '@surface', '@myramp', '@KERNEL'])

    def test_clean_snippet_has_no_diagnostics(self):
        self.open(SNIPPET)
        self.assertEqual(self.published(), [])

    def test_diagnostics(self):
        self.open('#include <missing.h>\n#bind poitn P float3\n#runover attribut\n@KERNEL\n{\n    foo(;\n')
        found = {(d['range']['start']['line'], d['severity']) for d in self.published()}
        self.assertTrue({(0, 2), (1, 1), (2, 2), (5, 1)} <= found, found)
        self.assertIn((4, 1), found)   # the unclosed {

    def test_conditionals_soften_include_and_delimiter_problems(self):
        self.open('#ifdef HOST\n#include <math.h>\n#endif\n#ifdef A\nif (x) {\n#else\nif (y) {\n#endif\n}\n')
        severities = {d['severity'] for d in self.published()}
        self.assertNotIn(1, severities)

    def test_unknown_binding_is_never_an_error(self):
        # The node's Bindings multiparm can declare it.
        self.open('@KERNEL\n{\n    @density.set(@temperature);\n}\n')
        self.assertEqual(self.published(), [])

    def test_missing_houdini_degrades_gracefully(self):
        notifications = []
        server = Server(lambda m, p: notifications.append((m, p)))
        with patch('ko_opencl_lsp.server.find_houdini', return_value=None):
            server.handle('initialize', {'initializationOptions': {}})
        warnings = [p for m, p in notifications if m == 'window/showMessage']
        self.assertIn('lsp.ko-opencl.settings.houdiniPath', warnings[0]['message'])
        uri = path_uri(self.root / 'x.cl')
        server.handle('textDocument/didOpen', {'textDocument': {'uri': uri, 'text': '#include <x.h>\n@KERNEL {}\n', 'version': 1}})
        published = [p for m, p in notifications if m == 'textDocument/publishDiagnostics'][-1]
        self.assertEqual(published['diagnostics'], [])   # include checks need the headers

    def test_find_houdini_prefers_setting_then_environment(self):
        self.assertEqual(find_houdini({'houdiniPath': str(self.hfs)}), self.hfs)
        self.assertEqual(find_houdini({}, {'KO_VEX_HOUDINI': str(self.hfs)}), self.hfs)
        self.assertIsNone(find_houdini({'discoverHoudini': False}, {'KO_VEX_HOUDINI': str(self.hfs)}))

    def test_header_index_reuses_cache_and_sees_edits(self):
        self.open(SNIPPET)
        doc = self.server.documents[self.uri]
        first = self.server.header_closure(doc)
        self.assertEqual({h.path.name for h in first}, {'matrix.h', 'typedefines.h', 'interpolate.h'})
        self.assertEqual(first[0].path.name, 'matrix.h')   # the default include comes first
        self.assertIs(self.server.header_closure(doc)[0], first[0])
        header = self.hfs / 'houdini/ocl/include/interpolate.h'
        header.write_text(INTERPOLATE_H + 'static float extra(float x) { return x; }\n', encoding='utf-8')
        os.utime(header, ns=(header.stat().st_atime_ns, header.stat().st_mtime_ns + 10**9))
        names = {d.name for h in self.server.header_closure(doc) for d in h.definitions}
        self.assertIn('extra', names)

    def test_utf16_positions(self):
        self.open('// é😀\n#bind point &P float3\n@KERNEL\n{\n    float3 a = @P; // 😀\n}\n')
        result = self.request('textDocument/hover', '@P; //', 1)
        self.assertEqual(result['range']['start']['character'], 15)


class ProtocolTests(Fixture):
    def test_stdio_round_trip(self):
        def packet(message):
            body = json.dumps(message).encode('utf-8')
            return b'Content-Length: ' + str(len(body)).encode() + b'\r\n\r\n' + body
        uri = path_uri(self.root / 'x.cl')
        incoming = io.BytesIO(b''.join(packet(m) for m in [
            {'jsonrpc': '2.0', 'id': 1, 'method': 'initialize', 'params': {'initializationOptions': {'houdiniPath': str(self.hfs)}}},
            {'jsonrpc': '2.0', 'method': 'textDocument/didOpen', 'params': {'textDocument': {'uri': uri, 'version': 1, 'text': '#bind point P'}}},
            {'jsonrpc': '2.0', 'id': 2, 'method': 'textDocument/hover', 'params': {'textDocument': {'uri': uri}, 'position': {'line': 0, 'character': 12}}},
            {'jsonrpc': '2.0', 'id': 3, 'method': 'workspace/unknownThing', 'params': {}},
            {'jsonrpc': '2.0', 'id': 4, 'method': 'shutdown'},
            {'jsonrpc': '2.0', 'method': 'exit'},
        ]))
        outgoing = io.BytesIO()
        with patch('ko_opencl_lsp.server.sys.stdin', SimpleNamespace(buffer=incoming)), \
             patch('ko_opencl_lsp.server.sys.stdout', SimpleNamespace(buffer=outgoing)):
            run_server()
        outgoing.seek(0)
        messages = []
        while (message := read_message(outgoing)) is not None:
            messages.append(message)
        by_id = {m['id']: m for m in messages if 'id' in m}
        self.assertEqual(by_id[1]['result']['serverInfo']['name'], 'ko-opencl-lsp')
        self.assertIn('#bind point P', by_id[2]['result']['contents']['value'])
        self.assertEqual(by_id[3]['error']['code'], -32601)
        self.assertIsNone(by_id[4]['result'])


@unittest.skipUnless(os.environ.get('KO_VEX_TEST_HFS'), 'set KO_VEX_TEST_HFS to check a real Houdini OpenCL corpus')
class HoudiniCorpusTests(unittest.TestCase):
    def test_houdini_ocl_files_have_no_bind_errors(self):
        hfs = Path(os.environ['KO_VEX_TEST_HFS'])
        server = Server()
        server.handle('initialize', {'initializationOptions': {'houdiniPath': str(hfs)}})
        files = sorted(p for p in (hfs / 'houdini/ocl').rglob('*') if p.suffix in ('.cl', '.h'))
        self.assertGreater(len(files), 50)
        for path in files:
            text = path.read_text(encoding='utf-8', errors='replace')
            server.handle('textDocument/didOpen', {'textDocument': {'uri': path_uri(path), 'text': text, 'version': 1}})
            doc = server.documents[path_uri(path)]
            errors = [d for d in server.diagnostics(doc) if d['severity'] == 1]
            self.assertEqual(errors, [], path)
        # SideFX's commented example of @-binding usage (implicitsurface.h) is clean.
        example = (hfs / 'houdini/ocl/include/implicitsurface.h').read_text(encoding='utf-8')
        example = example[example.index('#include'):example.index('*/')]
        server.handle('textDocument/didOpen', {'textDocument': {'uri': 'file:///example.cl', 'text': example, 'version': 1}})
        problems = [d for d in server.diagnostics(server.documents['file:///example.cl']) if d['severity'] <= 2]
        self.assertEqual([p['message'] for p in problems], ['Header not found: implicit_surface.h. Searched $HFS/houdini/ocl/include.'])


if __name__ == '__main__':
    unittest.main()
