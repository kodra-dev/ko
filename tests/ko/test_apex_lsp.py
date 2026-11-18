"""Offline APEX Script language-server tests (no hou/UI/provider).

The fixture mirrors the layout of Houdini's generated packages/apex/python3.*libs/apexscript.py.
Set KO_VEX_TEST_HFS to also parse a real installation's catalogue.
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

TOOL = Path(__file__).resolve().parents[2] / 'tools/apex-lsp'
sys.path.insert(0, str(TOOL))
from ko_apex_lsp import analysis, catalog
from ko_apex_lsp.language import OBJECT_METHODS
from ko_apex_lsp.server import Server, main as run_server, read_message

STUB = textwrap.dedent('''\
    def sine(*args, **kwargs):
        """
        def sine(radians: Float) -> Float:

            Computes the sine of an angle.

            Returns result: Float
        """

        return None


    def add(*args, **kwargs):
        """
        def add(a: Float | Int | Vector3,
            b: VariadicArg<Int> | VariadicArg<Float> | VariadicArg<Vector3>) -> Float | Int | Vector3:

            Calculates the sum of the input values.

            Returns result: Float | Int | Vector3
        """

        return None


    class namespace_sop_copytopoints:

        def attribs(*args, **kwargs):
            """
            def attribs(geo: Geometry) -> Geometry:

                Documentation will be added at a later point.

                Returns geo: Geometry
            """

            return None


    class namespace_sop:

        copytopoints = namespace_sop_copytopoints

        def box(*args, **kwargs):
            """
            def box(t: Vector3, scale: Float) -> Geometry:

                Creates a box.

                Returns geo: Geometry
            """

            return None

        def copytopoints(*args, **kwargs):
            """
            def copytopoints(geo0: Geometry, geo1: Geometry, pack: Int) -> Geometry:

                Copies geometry in the first input onto the points of the second input.

                Returns geo: Geometry
            """

            return None


    class namespace_geo:

        class IntersectCache:

            def __init__(self):
                """
                """
                pass

        def setPrimAttribValue(*args, **kwargs):
            """
            def setPrimAttribValue(*geo: Geometry,
                elemnum: Int,
                attribname: String,
                value: Float | String) -> tuple[Geometry, Bool]:

                Sets a primitive attribute value on a geometry.

                Returns *geo: Geometry, success: Bool
            """

            return None


    class Float:

        def __init__(self):
            """
            """
            pass


    class Int:

        def __init__(self):
            """
            """
            pass


    class String:

        def __init__(self):
            """
            """
            pass


    class Vector3:

        def __init__(self):
            """
            Returns a new Vector3 from a sequence of floats.
            """
            pass

        def x(self, *args, **kwargs):
            """
            def x(self, index: Int=0) -> Float:

                Extracts a single component of a vector or matrix type.

                Returns value: Float
            """

            return None


    class Geometry:

        def __init__(self):
            """
            """
            pass

        def setPrimAttribValue(self, *args, **kwargs):
            """
            def setPrimAttribValue(self, elemnum: Int,
                attribname: String,
                value: Float | String) -> Bool:

                Sets a primitive attribute value on a geometry.

                Inplace update

                Returns success: Bool
            """

            return None

        def computeTransform(self, *args, **kwargs):
            """
            def computeTransform(self, mode: Int):

                Computes world transforms.

                Inplace update

                Returns None
            """

            return None


    class StringArray:

        def __init__(self):
            """
            """
            pass


    class ApexNodeID:

        def __init__(self):
            """
            """
            pass


    class ApexPortID:

        def __init__(self):
            """
            """
            pass


    class ApexGraphHandle:

        def __init__(self):
            """
            """
            pass

        def addNode(self, *args, **kwargs):
            """
            def addNode(self, name: String, callback: String, color: Vector3) -> ApexNodeID:

                Adds a node to an APEX graph.

                Inplace update

                Returns nodeid: ApexNodeID
            """

            return None


    class void:

        def __init__(self):
            """
            """
            pass


    class apex:

        geo = namespace_geo
        sop = namespace_sop
    ''')


def fixture_catalog():
    loaded = catalog.Catalog(catalog.build(STUB), Path('apexscript.py'))
    loaded.add_methods(OBJECT_METHODS)
    return loaded


def position(text, needle, delta=0):
    index = text.index(needle) + delta
    return analysis.Text(text).position(index)


class CatalogTests(unittest.TestCase):
    def setUp(self):
        self.catalog = fixture_catalog()

    def test_layout_namespaces_types_and_docstring_signatures(self):
        self.assertEqual(set(self.catalog.functions), {'sine', 'add'})
        self.assertEqual(set(self.catalog.namespaces), {'apex', 'apex.geo', 'apex.sop', 'apex.sop.copytopoints'})
        self.assertNotIn('void', self.catalog.types)
        self.assertIn('geo::IntersectCache', self.catalog.types)
        add = self.catalog.function('add')
        self.assertEqual([p['name'] for p in add['params']], ['a', 'b'])
        self.assertEqual(add['params'][1]['type'], 'VariadicArg<Int> | VariadicArg<Float> | VariadicArg<Vector3>')
        self.assertEqual(add['returns'], 'Float | Int | Vector3')
        setter = self.catalog.namespace_function('apex.geo', 'setPrimAttribValue')
        self.assertTrue(setter['params'][0]['inplace'])
        self.assertEqual(setter['params'][0]['name'], 'geo')
        self.assertEqual(setter['returns'], 'tuple[Geometry, Bool]')
        self.assertEqual(self.catalog.namespace_function('apex.sop.copytopoints', 'attribs')['doc'], '')

    def test_methods_drop_self_and_record_inplace(self):
        method = self.catalog.method('Geometry', 'setPrimAttribValue')
        self.assertEqual([p['name'] for p in method['params']], ['elemnum', 'attribname', 'value'])
        self.assertTrue(method['inplace'])
        self.assertEqual(method['doc'], 'Sets a primitive attribute value on a geometry.')
        self.assertEqual(self.catalog.method('Vector3', 'x')['params'][0]['default'], '0')
        self.assertEqual(catalog.signature(method),
                         'Geometry.setPrimAttribValue(elemnum: Int, attribname: String, value: Float | String) -> Bool')

    def test_lookup_rules(self):
        # Case-insensitive first letter (docs write Sine(); the catalogue has sine).
        self.assertIs(self.catalog.function('Sine'), self.catalog.function('sine'))
        # apex.sine(...) is the same function; templated spelling strips the type suffix.
        self.assertIs(self.catalog.namespace_function('apex', 'sine'), self.catalog.function('sine'))
        self.assertIs(self.catalog.method('Geometry', 'setPrimAttribValue_String'),
                      self.catalog.method('Geometry', 'setPrimAttribValue'))
        self.assertIsNone(self.catalog.method('Geometry', 'missing_String'))
        # Object methods missing from the generated stub are added from the docs.
        self.assertEqual(self.catalog.method('ApexNodeID', 'name')['returns'], 'String')
        self.assertEqual(self.catalog.method('ApexPortID', 'connect')['params'][0]['name'], 'dstport')

    def test_single_type(self):
        self.assertEqual(catalog.single_type('Geometry'), 'Geometry')
        self.assertEqual(catalog.single_type('Float[outputy]'), 'Float')
        self.assertEqual(catalog.single_type('Float | Int'), '')
        self.assertEqual(catalog.single_type('tuple[Geometry, Bool]'), '')
        self.assertEqual(catalog.single_type('None'), '')

    def test_cache_is_reused_and_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root/'packages/apex/python3.13libs/apexscript.py'
            source.parent.mkdir(parents=True)
            source.write_text(STUB, encoding='utf-8')
            self.assertEqual(catalog.catalog_source(root), source)
            (root/'cache').mkdir()
            (root/'cache/apex-catalog-old.json').write_text('{}')
            first = catalog.load(source, root/'cache')
            caches = list((root/'cache').glob('apex-catalog-*.json'))
            self.assertEqual(len(caches), 1)
            with patch('ko_apex_lsp.catalog.build', side_effect=AssertionError('cache not used')):
                second = catalog.load(source, root/'cache')
            self.assertEqual(second.functions, first.functions)
            caches[0].write_text('not json')
            self.assertEqual(catalog.load(source, root/'cache').functions, first.functions)

    @unittest.skipUnless(os.environ.get('KO_VEX_TEST_HFS'), 'set KO_VEX_TEST_HFS to parse a real catalogue')
    def test_real_houdini_catalogue(self):
        source = catalog.catalog_source(Path(os.environ['KO_VEX_TEST_HFS']))
        self.assertIsNotNone(source)
        real = catalog.load(source)
        self.assertGreater(len(real.functions), 50)
        self.assertIn('Geometry', real.types)
        self.assertIn('apex.sop', real.namespaces)
        self.assertEqual(catalog.signature(real.function('Sine')), 'sine(radians: Float) -> Float')
        self.assertEqual(real.method('ApexGraphHandle', 'addNode')['returns'], 'ApexNodeID')


class DiagnosticTests(unittest.TestCase):
    def check(self, text):
        tree, error = analysis.parse(text)
        return analysis.diagnostics(text, tree, error)

    def test_unsupported_constructs_use_houdini_messages_and_keyword_ranges(self):
        text = 'x = 1\nwhile x:\n    pass\ndef f():\n    for i in range(3):\n        break\n'
        found = self.check(text)
        self.assertEqual([d['message'] for d in found],
                         ['While is not supported in Apex Script.', 'Break is not supported in Apex Script.'])
        self.assertEqual(found[0]['range'], {'start': {'line': 1, 'character': 0}, 'end': {'line': 1, 'character': 5}})
        self.assertEqual(found[1]['range']['start'], {'line': 5, 'character': 8})
        self.assertTrue(all(d['severity'] == 1 for d in found))

    def test_supported_and_unchecked_constructs_are_not_reported(self):
        text = textwrap.dedent('''\
            @subgraph
            def outer(a: Int) -> Int:
                def inner():
                    nonlocal a
                    return a
                return inner()
            raise error(f'bad {1+2}')
            x = 1 if True else 2
            ''')
        # nonlocal falls through Houdini's visitor (visit_NonLocal never matches ast.Nonlocal).
        self.assertEqual(self.check(text), [])

    def test_imports_warn_except_houdini_autocomplete_line(self):
        found = self.check('from apexscript import *\nimport os\n')
        self.assertEqual([(d['severity'], d['range']['start']['line']) for d in found], [(2, 1)])

    def test_syntax_error_uses_utf16_columns(self):
        text = "s = '🦊' + (1\n"
        found = self.check(text)
        self.assertEqual(len(found), 1)
        self.assertEqual(found[0]['severity'], 1)
        # '(' is the 10th character but the 11th UTF-16 unit (the emoji is two units).
        self.assertEqual(found[0]['range']['start'], {'line': 0, 'character': 11})

    def test_ast_byte_columns_convert_to_utf16(self):
        text = "s = 'é🦊'; break\n"
        tree, _ = analysis.parse('for i in range(1):\n    ' + text)
        found = analysis.diagnostics('for i in range(1):\n    ' + text, tree, None)
        self.assertEqual(found[0]['range']['start'], {'line': 1, 'character': 4 + len("s = 'é🦊'; ") + 1})


class AnalysisTests(unittest.TestCase):
    def setUp(self):
        self.catalog = fixture_catalog()

    def analyze(self, text):
        tree, error = analysis.parse(text)
        self.assertIsNone(error)
        return analysis.Analysis(text, tree, self.catalog)

    def types(self, text):
        result = self.analyze(text)
        return {s.name: s.type for s in result.symbols if s.kind != 'function'}

    def test_literals_constructors_and_bindings(self):
        found = self.types(textwrap.dedent('''\
            i = 3
            f = 1.5
            b = True
            s = f"x {i}"
            v = (1, 2, 3)
            m = (v, (4, 5, 6), (7, 8, 9))
            arr = ['a', 'b']
            d = {'k': 1}
            geo = BindInput(Geometry())
            named = BindInput(Base__dot__skel=Geometry())
            g: ApexGraphHandle = BindInput()
            x, y = BindInput(2.6, 'test')
            k = Float(3)
            first = arr[0]
            comp = v[1]
            n = g.addNode('a', 'TransformObject')
            port = n.xform_out
            wave = sine(1.0)
            box = apex.sop.box(t=v, scale=2.0)
            chained = geo.computeTransform(1)
            '''))
        expected = {'i': 'Int', 'f': 'Float', 'b': 'Bool', 's': 'String', 'v': 'Vector3', 'm': 'Matrix3',
                    'arr': 'StringArray', 'd': 'Dict', 'geo': 'Geometry', 'named': 'Geometry',
                    'g': 'ApexGraphHandle', 'x': 'Float', 'y': 'String', 'k': 'Float', 'first': 'String',
                    'comp': 'Float', 'n': 'ApexNodeID', 'port': 'ApexPortID', 'wave': 'Float',
                    'box': 'Geometry', 'chained': 'Geometry'}
        self.assertEqual({k: found.get(k) for k in expected}, expected)

    def test_static_typing_keeps_first_type_and_loops(self):
        found = self.types(textwrap.dedent('''\
            x = 3
            x = 'string'
            geo = BindInput(Geometry())
            for pt in geo.points():
                pass
            for i, name in enumerate(['a']):
                pass
            for j in range(3):
                pass
            '''))
        self.assertEqual((found['x'], found['pt'], found['i'], found['name'], found['j']),
                         ('Int', 'Int', 'Int', 'String', 'Int'))

    def test_functions_parameters_and_tuple_returns(self):
        result = self.analyze(textwrap.dedent('''\
            def test(a: Vector3, b: Float = 2.0) -> tuple[Vector3, Float[outputy]]:
                c = a * b
                return a, b
            p, q = test((1, 2, 3))
            '''))
        test = next(s for s in result.symbols if s.name == 'test')
        self.assertEqual(test.detail, 'def test(a: Vector3, b: Float=2.0) -> tuple[Vector3, Float[outputy]]')
        local = next(s for s in result.symbols if s.name == 'c')
        self.assertIs(local.scope, test)
        self.assertEqual(local.type, 'Vector3')
        types = {s.name: s.type for s in result.symbols if s.scope is None and s.kind == 'variable'}
        self.assertEqual(types, {'p': 'Vector3', 'q': 'Float'})

    def test_repair_keeps_symbols_while_typing(self):
        text = 'geo = BindInput(Geometry())\ngeo.\nlater = 1\n'
        tree, error = analysis.parse(text)
        self.assertIsNotNone(error)
        source, tree = analysis.repaired(text, error)
        result = analysis.Analysis(source, tree, self.catalog)
        self.assertEqual({s.name for s in result.symbols}, {'geo', 'later'})


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.notifications = []
        self.server = Server(lambda method, params: self.notifications.append((method, params)))
        self.server.catalog = fixture_catalog()
        self.uri = 'file:///C:/apex/case.apex'

    def open(self, text, version=1):
        self.text = text
        method = 'textDocument/didOpen' if self.uri not in self.server.documents else 'textDocument/didChange'
        if method == 'textDocument/didOpen':
            self.server.handle(method, {'textDocument': {'uri': self.uri, 'text': text, 'version': version}})
        else:
            self.server.handle(method, {'textDocument': {'uri': self.uri, 'version': version},
                                        'contentChanges': [{'text': text}]})

    def request(self, method, needle=None, delta=0, at_end=False):
        pos = analysis.Text(self.text).position(len(self.text)) if at_end else position(self.text, needle, delta)
        return self.server.handle(method, {'textDocument': {'uri': self.uri}, 'position': pos})

    def labels(self, result):
        return [item['label'] for item in (result or {}).get('items', [])]

    def test_member_completion_while_the_line_does_not_parse(self):
        base = 'geo = BindInput(Geometry())\ng: ApexGraphHandle = BindInput()\n'
        self.open(base + 'geo.')
        self.assertIn('setPrimAttribValue', self.labels(self.request('textDocument/completion', at_end=True)))
        self.open(base + 'apex.sop.', 2)
        labels = self.labels(self.request('textDocument/completion', at_end=True))
        self.assertEqual(sorted(labels), ['box', 'copytopoints', 'copytopoints'])
        self.open(base + 'g.addNode("a", "b").', 3)
        self.assertIn('setColor', self.labels(self.request('textDocument/completion', at_end=True)))
        self.open(base + 'g.addNode("a", "b").xform_out.', 4)
        self.assertIn('connect', self.labels(self.request('textDocument/completion', at_end=True)))
        self.open(base + 'apex.', 5)
        labels = self.labels(self.request('textDocument/completion', at_end=True))
        self.assertTrue({'geo', 'sop', 'sine', 'add'} <= set(labels))
        self.open(base + '"abc".', 6)
        self.assertEqual(self.labels(self.request('textDocument/completion', at_end=True)), [])

    def test_global_completion_decorators_and_suppression(self):
        self.open('def mine(a: Int):\n    local = a\n    \n')
        labels = self.labels(self.request('textDocument/completion', '    \n', 4))
        self.assertTrue({'BindInput', 'Geometry', 'sine', 'apex', 'M_PI', 'mine', 'local', 'a', 'def'} <= set(labels))
        self.open('@', 2)
        self.assertEqual(set(self.labels(self.request('textDocument/completion', at_end=True))),
                         {'subgraph', 'namespace', 'safeguard_inputs'})
        self.open('# geo.', 3)
        self.assertIsNone(self.request('textDocument/completion', at_end=True))
        self.open('s = "geo.', 4)
        self.assertIsNone(self.request('textDocument/completion', at_end=True))

    def test_no_completion_while_typing_a_number(self):
        for version, text in enumerate(('b = sine(a) * 2', 'b = 2.0', 'b = .5', 'v = (1e5', 'x = [3'), 1):
            self.open(text, version)
            self.assertEqual(self.labels(self.request('textDocument/completion', at_end=True)), [], text)
        self.open('x = Ve', 10)
        self.assertIn('Vector3', self.labels(self.request('textDocument/completion', at_end=True)))
        self.open('v = (1, 2, 3)\nv.x', 11)
        self.assertIn('x', self.labels(self.request('textDocument/completion', at_end=True)))

    def test_keyword_argument_completion_inside_calls(self):
        self.open('apex.sop.box(')
        labels = self.labels(self.request('textDocument/completion', at_end=True))
        self.assertTrue({'t=', 'scale=', '__name=', 'valuetype='} <= set(labels))

    def test_resolve_adds_documentation(self):
        self.open('apex.sop.')
        item = next(i for i in self.request('textDocument/completion', at_end=True)['items'] if i['label'] == 'box')
        resolved = self.server.handle('completionItem/resolve', dict(item))
        self.assertIn('Creates a box.', resolved['documentation']['value'])

    def test_hover(self):
        self.open(textwrap.dedent('''\
            g: ApexGraphHandle = BindInput()
            n = g.addNode('a', 'TransformObject')
            w = Sine(1.0) + M_PI
            b = apex.sop.copytopoints(n, n, 1)
            def test(a: Int):
                return a
            '''))
        value = lambda needle, delta=0: (self.request('textDocument/hover', needle, delta) or {}).get('contents', {}).get('value', '')
        self.assertIn('ApexGraphHandle.addNode(name: String', value('addNode'))
        self.assertIn('sine(radians: Float) -> Float', value('Sine'))
        self.assertIn('M_PI: Float', value('M_PI'))
        self.assertIn('n: ApexNodeID', value('n = g'))
        self.assertIn('copytopoints(geo0: Geometry', value('copytopoints'))
        self.assertIn('apex.sop', value('sop'))
        self.assertIn('BindInput(*defaults', value('BindInput'))
        self.assertIn('def test(a: Int)', value('test('))
        self.assertIn('(parameter) a: Int', value('return a', 7))
        self.assertEqual(value('return'), '')

    def test_signature_help_parameters_keywords_and_variadics(self):
        self.open('geo = BindInput(Geometry())\ngeo.setPrimAttribValue(0, attribname="n", ')
        help = self.request('textDocument/signatureHelp', at_end=True)
        signature = help['signatures'][0]
        self.assertTrue(signature['label'].startswith('Geometry.setPrimAttribValue('))
        self.assertEqual(len(signature['parameters']), 3)
        start, end = signature['parameters'][1]['label']
        self.assertEqual(signature['label'][start:end], 'attribname: String')
        self.assertEqual(help['activeParameter'], 2)
        self.open('x = add(1, 2, 3, ', 2)
        self.assertEqual(self.request('textDocument/signatureHelp', at_end=True)['activeParameter'], 1)
        self.open('apex.sop.box(scale=', 3)
        self.assertEqual(self.request('textDocument/signatureHelp', at_end=True)['activeParameter'], 1)
        self.open('x = (1, ', 4)
        self.assertIsNone(self.request('textDocument/signatureHelp', at_end=True))

    def test_definition_and_document_symbols(self):
        self.open(textwrap.dedent('''\
            geo = BindInput(Geometry())
            def test(a: Int):
                inner = a
                return inner
            y = test(1)
            z = sine(1.0)
            '''))
        location = self.request('textDocument/definition', 'test(1', 1)
        self.assertEqual(location, [{'uri': self.uri, 'range': {'start': {'line': 1, 'character': 4},
                                                                 'end': {'line': 1, 'character': 8}}}])
        stub = self.request('textDocument/definition', 'sine', 1)
        self.assertTrue(stub[0]['uri'].endswith('apexscript.py'))
        self.assertEqual(stub[0]['range']['start']['line'], 0)
        symbols = self.server.handle('textDocument/documentSymbol', {'textDocument': {'uri': self.uri}})
        self.assertEqual([(s['name'], s['kind']) for s in symbols], [('geo', 13), ('test', 12), ('y', 13), ('z', 13)])
        self.assertEqual([c['name'] for c in symbols[1]['children']], ['inner'])

    def test_positions_are_utf16(self):
        self.open("s = '🦊'; w = sine(1.0)\n")
        hover = self.request('textDocument/hover', 'sine', 1)
        # 'sine' starts at character 13; the emoji before it is two UTF-16 units.
        self.assertEqual(hover['range'], {'start': {'line': 0, 'character': 14}, 'end': {'line': 0, 'character': 18}})

    def test_diagnostics_published_on_open_change_and_cleared_on_close(self):
        self.open('while True:\n    pass\n')
        published = [p for m, p in self.notifications if m == 'textDocument/publishDiagnostics']
        self.assertEqual(published[-1]['diagnostics'][0]['message'], 'While is not supported in Apex Script.')
        self.open('x = 1\n', 2)
        published = [p for m, p in self.notifications if m == 'textDocument/publishDiagnostics']
        self.assertEqual((published[-1]['version'], published[-1]['diagnostics']), (2, []))
        self.server.handle('textDocument/didClose', {'textDocument': {'uri': self.uri}})
        self.assertEqual(self.notifications[-1][1], {'uri': self.uri, 'diagnostics': []})

    def test_missing_houdini_reports_once_and_keeps_syntax_checking(self):
        with patch.dict(os.environ, {}, clear=False), \
             patch('ko_apex_lsp.catalog.find_houdini', return_value=None):
            self.server.handle('initialize', {'initializationOptions': {}})
            self.server.wait_catalog()
        warnings = [p for m, p in self.notifications if m == 'window/showMessage']
        self.assertEqual(len(warnings), 1)
        self.assertIn('lsp.ko-apex.settings.houdiniPath', warnings[0]['message'])
        self.open('class X:\n    pass\n')
        self.assertTrue(self.notifications[-1][1]['diagnostics'])


class ProtocolTests(unittest.TestCase):
    def test_main_loop_round_trip(self):
        def packet(message):
            body = json.dumps(message).encode('utf-8')
            return b'Content-Length: ' + str(len(body)).encode() + b'\r\n\r\n' + body
        uri = 'file:///C:/apex/case.apex'
        incoming = io.BytesIO(b''.join(packet(m) for m in [
            {'jsonrpc': '2.0', 'id': 1, 'method': 'initialize', 'params': {'initializationOptions': {'discoverHoudini': False}}},
            {'jsonrpc': '2.0', 'method': 'textDocument/didOpen', 'params': {
                'textDocument': {'uri': uri, 'version': 1, 'text': 'try:\n    pass\nexcept:\n    pass\n'}}},
            {'jsonrpc': '2.0', 'id': 2, 'method': 'textDocument/unknown', 'params': {'textDocument': {'uri': uri}}},
            {'jsonrpc': '2.0', 'id': 3, 'method': 'shutdown'},
            {'jsonrpc': '2.0', 'method': 'exit'},
        ]))
        outgoing = io.BytesIO()
        with patch('ko_apex_lsp.server.sys.stdin', SimpleNamespace(buffer=incoming)), \
             patch('ko_apex_lsp.server.sys.stdout', SimpleNamespace(buffer=outgoing)):
            run_server()
        outgoing.seek(0)
        messages = []
        while (message := read_message(outgoing)) is not None:
            messages.append(message)
        by_id = {m['id']: m for m in messages if 'id' in m}
        self.assertEqual(by_id[1]['result']['serverInfo']['name'], 'ko-apex-lsp')
        self.assertEqual(by_id[2]['error']['code'], -32601)
        self.assertIsNone(by_id[3]['result'])
        diagnostics = next(m['params'] for m in messages if m.get('method') == 'textDocument/publishDiagnostics')
        self.assertEqual(diagnostics['diagnostics'][0]['message'], 'Exception handling is not supported in Apex Script.')


if __name__ == '__main__':
    unittest.main()
