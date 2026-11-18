"""Offline scheduler/process/protocol tests; optional local-vcc semantic regressions."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import io
import json
import os
import queue
import subprocess
import sys
import tempfile
import threading
import time
import unittest

TOOL = Path(__file__).resolve().parents[2]/'tools/vex-lsp'
sys.path.insert(0, str(TOOL))
from ko_vex_lsp.diagnostics import Diagnostics
from ko_vex_lsp.model import path_uri
from ko_vex_lsp.server import Server, main, read_message
from ko_vex_lsp.validation import (Context, Validator, ValidationResult, LightDocument, Cancelled,
                                    Unavailable, run_compiler, parse_messages, CACHE_BYTES)
from ko_vex_lsp.workspace import Workspace


def context(root):
    return Context(Path(root), Path(root)/'vcc', (), 'snippet', 'cvex', 2, (), 50)


class FakeValidator:
    def __init__(self, block=False, dependency=None):
        self.calls = []
        self.entered = threading.Event()
        self.block = block
        self.dependency = dependency
        self.running = self.max_running = 0
        self.closed = False

    def validate(self, uri, ctx, buffers, cancel):
        self.running += 1
        self.max_running = max(self.running, self.max_running)
        self.calls.append((uri, buffers[uri][1], ctx))
        self.entered.set()
        try:
            if self.block and len(self.calls) == 1:
                cancel.wait(3)
            # Intentionally returns even after cancellation: the scheduler must
            # independently suppress these stale diagnostics.
            diagnostics = [{'range': {'start': {'line': 0, 'character': 0}, 'end': {'line': 0, 'character': 1}},
                            'severity': 1, 'source': 'vcc', 'message': 'version '+str(buffers[uri][1])}]
            deps = frozenset([self.dependency]) if self.dependency and uri != self.dependency else frozenset()
            return ValidationResult(diagnostics, deps, {})
        finally:
            self.running -= 1

    def invalidate(self, _uris):
        pass

    def close(self):
        self.closed = True


class SchedulerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.uri = path_uri(self.root/'main.vfl')
        self.ctx = context(self.root)
        self.published = []

    def checker(self, fake):
        checker = Diagnostics(lambda job, result: self.published.append((job, result)), fake)
        self.addCleanup(checker.close)
        return checker

    def test_typing_burst_coalesces_to_one_latest_check(self):
        fake = FakeValidator()
        checker = self.checker(fake)
        for version in range(100):
            checker.update(self.ctx, {self.uri: ('float x='+str(version)+';', version)}, [self.uri], delay=.05)
        self.assertTrue(checker.wait_idle())
        self.assertEqual([version for _, version, _ in fake.calls], [99])
        self.assertEqual(self.published[0][0].version, 99)
        self.assertEqual(fake.max_running, 1)

    def test_inflight_cancel_and_late_result_are_discarded(self):
        fake = FakeValidator(block=True)
        checker = self.checker(fake)
        checker.update(self.ctx, {self.uri: ('old', 1)}, [self.uri], immediate=True)
        self.assertTrue(fake.entered.wait(2))
        checker.update(self.ctx, {self.uri: ('new', 2)}, [self.uri], immediate=True)
        self.assertTrue(checker.wait_idle())
        self.assertEqual([job.version for job, _ in self.published], [2])
        self.assertEqual(fake.max_running, 1)

    def test_close_cancels_and_does_not_publish(self):
        fake = FakeValidator(block=True)
        checker = self.checker(fake)
        checker.update(self.ctx, {self.uri: ('old', 1)}, [self.uri], immediate=True)
        self.assertTrue(fake.entered.wait(2))
        checker.update(self.ctx, {}, [self.uri], immediate=True)
        self.assertTrue(checker.wait_idle())
        self.assertEqual(self.published, [])
        checker.close()
        self.assertTrue(fake.closed)

    def test_reopen_same_version_and_replaced_checker_reject_old_jobs(self):
        checker = self.checker(FakeValidator())
        checker.update(self.ctx, {self.uri: ('old', 1)}, immediate=True)
        self.assertTrue(checker.wait_idle())
        old_job = self.published[-1][0]
        checker.update(self.ctx, {}, [self.uri])
        checker.update(self.ctx, {self.uri: ('different text', 1)}, immediate=True)
        self.assertTrue(checker.wait_idle())
        self.assertFalse(checker.current(old_job))
        replacement = self.checker(FakeValidator())
        replacement.update(self.ctx, {self.uri: ('old', 1)}, immediate=True)
        self.assertTrue(replacement.wait_idle())
        self.assertFalse(replacement.current(old_job))

    def test_configuration_supersedes_running_check(self):
        fake = FakeValidator(block=True)
        checker = self.checker(fake)
        checker.update(self.ctx, {self.uri: ('text', 1)}, immediate=True)
        self.assertTrue(fake.entered.wait(2))
        latest = replace(self.ctx, context='sop')
        checker.update(latest, {self.uri: ('text', 1)}, immediate=True)
        self.assertTrue(checker.wait_idle())
        self.assertEqual([job.context.context for job, _ in self.published], ['sop'])

    def test_only_actual_dependents_rechecked_after_header_edit(self):
        header = path_uri(self.root/'shared.h')
        unrelated = path_uri(self.root/'other.vfl')
        fake = FakeValidator(dependency=header)
        checker = self.checker(fake)
        buffers = {self.uri: ('main', 1), header: ('header', 1), unrelated: ('other', 1)}
        checker.update(self.ctx, buffers, immediate=True)
        self.assertTrue(checker.wait_idle())
        initial = len(fake.calls)
        buffers = {**buffers, header: ('changed', 2)}
        checker.update(self.ctx, buffers, [header], immediate=True)
        self.assertTrue(checker.wait_idle())
        # Both main and other depend on shared.h in this fake compiler.
        self.assertEqual({uri for uri, _, _ in fake.calls[initial:]}, set(buffers))
        initial = len(fake.calls)
        checker.update(self.ctx, {**buffers, unrelated: ('unrelated edit', 2)}, [unrelated], immediate=True)
        self.assertTrue(checker.wait_idle())
        self.assertEqual([uri for uri, _, _ in fake.calls[initial:]], [unrelated])

    def test_all_open_documents_eventually_checked_without_revision_queue_growth(self):
        fake = FakeValidator()
        checker = self.checker(fake)
        buffers = {path_uri(self.root/(str(i)+'.vfl')): ('float x=1;', 1) for i in range(50)}
        checker.update(self.ctx, buffers, immediate=True)
        self.assertTrue(checker.wait_idle())
        self.assertEqual({uri for uri, _, _ in fake.calls}, set(buffers))
        self.assertEqual(fake.max_running, 1)

    def test_disk_snapshot_race_is_retried_without_publishing_old_results(self):
        fake = FakeValidator()
        real = fake.validate
        def validate(*args):
            result = real(*args)
            result.retry = len(fake.calls) == 1
            return result
        fake.validate = validate
        checker = self.checker(fake)
        checker.update(self.ctx, {self.uri: ('text', 1)}, immediate=True, delay=.01)
        self.assertTrue(checker.wait_idle())
        self.assertEqual(len(fake.calls), 2)
        self.assertEqual(len(self.published), 1)

    def test_suspend_stops_background_compiler_and_resumes_after_refactor(self):
        fake = FakeValidator(block=True)
        checker = self.checker(fake)
        checker.update(self.ctx, {self.uri: ('text', 1)}, immediate=True)
        self.assertTrue(fake.entered.wait(2))
        with checker.suspend():
            self.assertEqual(fake.running, 0)
        self.assertTrue(checker.wait_idle())
        self.assertEqual(len(self.published), 1)


class ProcessTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.ctx = context(self.temp.name)

    def test_cancel_kills_and_reaps_the_child(self):
        children = []
        original = subprocess.Popen
        def spawn(*args, **kwargs):
            child = original(*args, **kwargs)
            children.append(child)
            return child
        event = threading.Event()
        timer = threading.Timer(.12, event.set)
        timer.start()
        self.addCleanup(timer.cancel)
        with patch('ko_vex_lsp.validation.subprocess.Popen', side_effect=spawn):
            with self.assertRaises(Cancelled):
                run_compiler([sys.executable, '-c', 'import time; time.sleep(20)'], self.ctx, event,
                             time.monotonic()+10, self.temp.name)
        self.assertIsNotNone(children[0].poll())

    def test_timeout_and_output_limits(self):
        with self.assertRaisesRegex(Unavailable, 'time limit'):
            run_compiler([sys.executable, '-c', 'import time; time.sleep(20)'], self.ctx, threading.Event(),
                         time.monotonic()+.12, self.temp.name)
        with self.assertRaisesRegex(Unavailable, 'output limit'):
            run_compiler([sys.executable, '-c', 'import os; os.write(1,b"x"*10000)'], self.ctx, threading.Event(),
                         time.monotonic()+3, self.temp.name, output_limit=128)


    def test_fast_exiting_binary_output_is_bounded_and_removed(self):
        output = Path(self.temp.name)/'check.vex'
        with patch('ko_vex_lsp.validation.MAX_PREPROCESSED', 128):
            with self.assertRaisesRegex(Unavailable, 'compiled output'):
                run_compiler([sys.executable, '-c', 'from pathlib import Path; import sys; Path(sys.argv[1]).write_bytes(b"x"*256)', str(output)],
                             self.ctx, threading.Event(), time.monotonic()+3, self.temp.name)
        self.assertFalse(output.exists())


class ValidationUnitTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.uri = path_uri(self.root/'main.vfl')
        self.ctx = context(self.root)

    def test_no_ast_dump_and_unchanged_content_cache(self):
        calls = []
        def runner(command, *_):
            calls.append(command)
            return 0, '', ''
        validator = Validator(runner)
        self.addCleanup(validator.close)
        for version in (1, 2):
            result = validator.validate(self.uri, self.ctx, {self.uri: ('float x=1;', version)}, threading.Event())
            self.assertEqual(result.diagnostics, [])
        self.assertEqual(len(calls), 1)
        self.assertNotIn('dumpast', calls[0])
        self.assertNotIn('-d', calls[0])
        self.assertLessEqual(validator.workspace.cache_bytes, CACHE_BYTES)
        self.assertEqual(validator.workspace.buffers, {})

    def test_only_changed_headers_are_reparsed_and_mirrored(self):
        header = self.root/'helpers.h'
        header.write_text('function float helper(float x){return x;}')
        validator = Validator(lambda *_: (0, '', ''))
        self.addCleanup(validator.close)
        ctx = replace(self.ctx, include_paths=(self.root,))
        text = '#include <helpers.h>\nhelper(1);'
        validator.validate(self.uri, ctx, {self.uri: (text, 1)}, threading.Event())
        mirror = validator.mirror_path(path_uri(header))
        original_stamp = mirror.stat().st_mtime_ns
        original_model = validator.workspace.cache[path_uri(header)][1]
        validator.validate(self.uri, ctx, {self.uri: (text+'\n', 2)}, threading.Event())
        self.assertEqual(mirror.stat().st_mtime_ns, original_stamp)
        self.assertIs(validator.workspace.cache[path_uri(header)][1], original_model)
        header.write_text('function string helper(string text){return text;}')
        validator.validate(self.uri, ctx, {self.uri: (text, 3)}, threading.Event())
        self.assertIn('string', mirror.read_text())

    def test_header_cache_is_bounded_and_changed_disk_snapshot_requests_retry(self):
        headers = []
        for index in range(6):
            path = self.root/(str(index)+'.h')
            path.write_text('function float h'+str(index)+'(float value){return value;}')
            headers.append(path)
        text = '\n'.join('#include <'+path.name+'>' for path in headers)+'\nh0(1);'
        changed = False
        def runner(*_):
            nonlocal changed
            if not changed:
                headers[0].write_text('function float h0(float value){return value*2;}')
                changed = True
            return 0, '', ''
        validator = Validator(runner)
        self.addCleanup(validator.close)
        with patch('ko_vex_lsp.validation.CACHE_FILES', 2):
            result = validator.validate(self.uri, replace(self.ctx, include_paths=(self.root,)), {self.uri: (text, 1)}, threading.Event())
            self.assertTrue(result.retry)
            self.assertLessEqual(len(validator.workspace.cache), 2)
        self.assertEqual(validator.workspace.buffer_aliases, {})

    def test_watcher_invalidates_headers_even_with_preserved_mtime_and_size(self):
        header = self.root/'helpers.h'
        header.write_text('float old;')
        validator = Validator(lambda *_: (0, '', ''))
        self.addCleanup(validator.close)
        ctx = replace(self.ctx, include_paths=(self.root,))
        buffers = {self.uri: ('#include <helpers.h>\n', 1)}
        validator.validate(self.uri, ctx, buffers, threading.Event())
        stamp = header.stat()
        header.write_text('float new;')
        os.utime(header, ns=(stamp.st_atime_ns, stamp.st_mtime_ns))
        validator.invalidate([path_uri(header)])
        validator.validate(self.uri, ctx, buffers, threading.Event())
        self.assertIn('new', validator.mirror_path(path_uri(header)).read_text())

    def test_model_budget_returns_information_without_starting_compiler(self):
        calls = []
        validator = Validator(lambda *args: calls.append(args) or (0, '', ''))
        self.addCleanup(validator.close)
        with patch('ko_vex_lsp.validation.MAX_MODEL_BYTES', 300):
            result = validator.validate(self.uri, self.ctx, {self.uri: ('f@one=1; f@two=2;', 1)}, threading.Event())
        self.assertEqual(result.diagnostics[0]['code'], 'validation-unavailable')
        self.assertEqual(calls, [])

    def test_unknown_attributes_are_explicit_not_guessed(self):
        validator = Validator(lambda *_: (0, '', ''))
        self.addCleanup(validator.close)
        result = validator.validate(self.uri, self.ctx, {self.uri: ('@custom *= 2;', 1)}, threading.Event())
        self.assertEqual(result.diagnostics[0]['code'], 'validation-unavailable')
        self.assertEqual(result.diagnostics[0]['severity'], 3)
        ctx = replace(self.ctx, attributes=(('custom', 'vector'),))
        self.assertEqual(validator.validate(self.uri, ctx, {self.uri: ('@custom *= 2;', 1)}, threading.Event()).diagnostics, [])

    def test_utf16_columns_warnings_headers_and_eof(self):
        text = 'string text="é🦊"; float value=missing();\r\n'
        doc = LightDocument(self.uri, text)
        paths = {self.uri: self.root/'mirror.vfl'}
        start = len(text[:text.index('missing')].encode('utf-8'))+1
        diagnostic = parse_messages(str(paths[self.uri])+':1:'+str(start)+'-'+str(start+6)+': Warning 2001: example', doc, [doc], paths)[0]
        self.assertEqual(diagnostic['range']['start'], doc.position(text.index('missing')))
        self.assertEqual(diagnostic['severity'], 2)
        eof = parse_messages(str(paths[self.uri])+':99:1: Error 1088: expected semicolon', doc, [doc], paths)[0]
        self.assertEqual(eof['range']['start'], doc.position(len(text)))
        header = LightDocument(path_uri(self.root/'header.h'), 'bad();')
        paths[header.uri] = self.root/'mirror.h'
        error = parse_messages(str(paths[header.uri])+':1:1-3: Error 1001: bad header', doc, [doc, header], paths)[0]
        self.assertEqual(error['relatedInformation'][0]['location']['uri'], header.uri)


class ProtocolDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.uri = path_uri(self.root/'main.vfl')
        self.notifications, self.requests = [], []
        self.server = Server(lambda method, params: self.notifications.append((method, params)),
                             lambda method, params: self.requests.append((method, params)))
        self.addCleanup(self.server.close)
        self.server.handle('initialize', {'rootUri': path_uri(self.root),
            'initializationOptions': {'discoverHoudini': False},
            'capabilities': {'workspace': {'didChangeWatchedFiles': {'dynamicRegistration': True}}}})
        self.server.workspace.vcc = self.root/'fake-vcc'
        self.server.workspace.vcc.touch()
        self.fake = FakeValidator()
        self.factory = patch('ko_vex_lsp.server.Diagnostics', side_effect=lambda publish: Diagnostics(publish, self.fake))
        self.factory.start()
        self.addCleanup(self.factory.stop)

    def open(self):
        self.server.handle('textDocument/didOpen', {'textDocument': {'uri': self.uri, 'version': 1, 'text': 'float value=1;'}})
        self.server.handle('textDocument/didSave', {'textDocument': {'uri': self.uri}})
        self.assertTrue(self.server.checker.wait_idle())

    def test_default_live_checks_publish_versions_and_clear_on_close(self):
        self.open()
        self.assertEqual(self.notifications[-1][1]['version'], 1)
        self.assertEqual(self.notifications[-1][1]['diagnostics'][0]['message'], 'version 1')
        self.server.handle('textDocument/didClose', {'textDocument': {'uri': self.uri}})
        self.assertEqual(self.notifications[-1][1]['diagnostics'], [])

    def test_hover_does_not_wait_for_compilation(self):
        self.fake.block = True
        self.server.handle('textDocument/didOpen', {'textDocument': {'uri': self.uri, 'version': 1, 'text': 'float value=1;'}})
        self.server.handle('textDocument/didSave', {'textDocument': {'uri': self.uri}})
        self.assertTrue(self.fake.entered.wait(2))
        before = time.monotonic()
        hover = self.server.handle('textDocument/hover', {'textDocument': {'uri': self.uri}, 'position': {'line': 0, 'character': 7}})
        self.assertIn('float value', hover['contents']['value'])
        self.assertLess(time.monotonic()-before, .5)

    def test_watched_files_recheck_known_dependents(self):
        header = path_uri(self.root/'helpers.h')
        self.fake.dependency = header
        self.open()
        initial = len(self.fake.calls)
        self.server.handle('workspace/didChangeWatchedFiles', {'changes': [{'uri': header, 'type': 2}]})
        self.assertTrue(self.server.checker.wait_idle())
        self.assertEqual(len(self.fake.calls), initial+1)

    def test_concurrent_notifications_and_responses_have_atomic_lsp_frames(self):
        class SlowBuffer:
            def __init__(self):
                self.bytes = io.BytesIO()
            def write(self, value):
                for start in range(0, len(value), 7):
                    self.bytes.write(value[start:start+7])
                    time.sleep(.00005)
                return len(value)
            def flush(self):
                pass
        class Producer:
            def __init__(self, notify, _request):
                self.notify, self.threads = notify, []
            def handle(self, *_):
                for version in range(8):
                    thread = threading.Thread(target=self.notify, args=('textDocument/publishDiagnostics', {'version': version, 'message': '🦊'}))
                    self.threads.append(thread)
                    thread.start()
                return {'ok': True}
            def close(self):
                for thread in self.threads:
                    thread.join(3)
        body = json.dumps({'jsonrpc': '2.0', 'id': 1, 'method': 'initialize', 'params': {}}).encode()
        incoming = io.BytesIO(b'Content-Length: '+str(len(body)).encode()+b'\r\n\r\n'+body)
        outgoing = SlowBuffer()
        with patch('ko_vex_lsp.server.Server', Producer), \
             patch('ko_vex_lsp.server.sys.stdin', SimpleNamespace(buffer=incoming)), \
             patch('ko_vex_lsp.server.sys.stdout', SimpleNamespace(buffer=outgoing)):
            main()
        outgoing.bytes.seek(0)
        messages = []
        while (message := read_message(outgoing.bytes)) is not None:
            messages.append(message)
        self.assertEqual(len(messages), 9)
        self.assertTrue(any(m.get('result') == {'ok': True} for m in messages))

    def test_watch_registration_and_disable_clear_diagnostics(self):
        self.server.handle('initialized', {})
        self.assertEqual(self.requests[0][0], 'client/registerCapability')
        self.open()
        checker = self.server.checker
        self.server.handle('workspace/didChangeConfiguration', {'settings': {'ko-vex': {
            'discoverHoudini': False, 'compilerDiagnostics': False}}})
        self.assertIsNone(self.server.checker)
        self.assertTrue(checker.closed)
        self.assertEqual(self.notifications[-1][1]['diagnostics'], [])


@unittest.skipUnless(os.environ.get('KO_VEX_TEST_HFS'), 'Set KO_VEX_TEST_HFS for local vcc diagnostics tests')
class NativeDiagnosticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hfs = Path(os.environ['KO_VEX_TEST_HFS'])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.uri = path_uri(self.root/'main.vfl')
        self.ctx = Context(self.root, self.hfs/'bin/vcc.exe',
                           (self.root, self.hfs/'packages/kinefx/vex/include', self.hfs/'houdini/vex/include'),
                           'snippet', 'cvex', 8, (), 50)
        self.validator = Validator()
        self.addCleanup(self.validator.close)

    def validate(self, text, extras=None, ctx=None):
        return self.validator.validate(self.uri, ctx or self.ctx, {self.uri: (text, 1), **(extras or {})}, threading.Event()).diagnostics

    def test_exact_user_report_argument_error_and_fix(self):
        prefix = '#include <kinefx.h>\n'
        errors = self.validate(prefix+'getpointlocaltransform(0,"0");')
        self.assertEqual(errors[0]['code'], 1066)
        self.assertIn('Candidates', errors[0]['message'])
        self.assertEqual(errors[0]['range']['start']['line'], 1)
        self.assertEqual(self.validate(prefix+'getpointlocaltransform("0",0);'), [])

    def test_unknown_function_arity_and_missing_semicolon(self):
        for code in ('float x=missing();', '#include <kinefx.h>\ngetpointlocaltransform();', 'float x=1'):
            with self.subTest(code=code):
                self.assertTrue(any(d['severity'] == 1 for d in self.validate(code)))

    def test_unsaved_header_changes_override_disk_and_invalidate_result_cache(self):
        header = self.root/'helpers.h'
        header.write_text('function float helper(float value){return value;}')
        text = '#include <helpers.h>\nhelper(1);'
        self.assertEqual(self.validate(text), [])
        errors = self.validate(text, {path_uri(header): ('function string helper(string value){return value;}', 2)})
        self.assertTrue(any(d['severity'] == 1 for d in errors))
        self.assertEqual(self.validate(text), [])

    def test_macros_conditionals_and_active_header_source_mapping(self):
        text = '#define GEO "0"\n#if 0\n#include <absent.h>\ninvalid(\n#endif\n#include <kinefx.h>\ngetpointlocaltransform(GEO,"0");'
        errors = self.validate(text)
        self.assertEqual(errors[0]['code'], 1066)
        self.assertEqual(errors[0]['range']['start']['line'], 6)
        self.assertNotIn('absent', errors[0]['message'])
        self.assertEqual(self.validate(text.replace('GEO,"0"', 'GEO,0')), [])

    def test_macro_include_uses_unsaved_overlay(self):
        header = self.root/'helpers.h'
        header.write_text('function float helper(float value){return value;}')
        text = '#define HEADER "helpers.h"\n#include HEADER\nhelper(1);'
        errors = self.validate(text, {path_uri(header): ('function string helper(string value){return value;}', 2)})
        self.assertTrue(any(d['severity'] == 1 for d in errors))

    def test_native_warnings_unicode_and_unused_function_errors(self):
        warnings = self.validate('string label="é🦊"; float value; printf("%s %g",label,value);')
        self.assertTrue(any(d['severity'] == 2 and d['code'] == 2018 for d in warnings))
        self.assertTrue(any(d['code'] == 1066 for d in self.validate('function float helper(float x){return missing(x);} float y=1;')))

    def test_unsaved_header_uri_alias_is_used(self):
        header = self.root/'helpers.h'
        header.write_text('function float helper(float value){return value;}')
        alias = path_uri(header).replace('helpers.h', 'helpers%2Eh')
        errors = self.validate('#include <helpers.h>\nhelper(1);', {alias: ('function string helper(string value){return value;}', 2)})
        self.assertTrue(any(d['severity'] == 1 for d in errors))

    def test_server_live_open_edit_and_disk_header_invalidation(self):
        messages = []
        server = Server(lambda method, params: messages.append((method, params)))
        self.addCleanup(server.close)
        server.handle('initialize', {'rootUri': path_uri(self.root), 'initializationOptions': {
            'houdiniPath': str(self.hfs), 'includePaths': [str(self.root)], 'diagnosticsDebounceMs': 100}})
        header = self.root/'helpers.h'
        header.write_text('function float helper(float value){return value;}')
        text = '#include <helpers.h>\nhelper("wrong");'
        server.handle('textDocument/didOpen', {'textDocument': {'uri': self.uri, 'text': text, 'version': 1}})
        self.assertTrue(server.checker.wait_idle(15))
        self.assertTrue(any(d['severity'] == 1 for d in messages[-1][1]['diagnostics']))
        server.handle('textDocument/didChange', {'textDocument': {'uri': self.uri, 'version': 2}, 'contentChanges': [{'text': text.replace('"wrong"', '1')} ]})
        self.assertTrue(server.checker.wait_idle(15))
        self.assertEqual(messages[-1][1]['version'], 2)
        self.assertEqual(messages[-1][1]['diagnostics'], [])
        header.write_text('function string helper(string value){return value;}')
        server.handle('workspace/didChangeWatchedFiles', {'changes': [{'uri': path_uri(header), 'type': 2}]})
        self.assertTrue(server.checker.wait_idle(15))
        self.assertTrue(any(d['severity'] == 1 for d in messages[-1][1]['diagnostics']))

    def test_stdio_background_results_and_requests_share_the_live_connection(self):
        process = subprocess.Popen([sys.executable, str(TOOL/'run.py')], cwd=self.root,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        received, history = queue.Queue(), []
        def read():
            try:
                while (message := read_message(process.stdout)) is not None:
                    received.put(message)
            except (ValueError, OSError):
                pass
        reader = threading.Thread(target=read, daemon=True)
        reader.start()
        def send(method, params=None, request_id=None):
            message = {'jsonrpc': '2.0', 'method': method, 'params': params or {}}
            if request_id is not None:
                message['id'] = request_id
            body = json.dumps(message).encode('utf-8')
            process.stdin.write(b'Content-Length: '+str(len(body)).encode()+b'\r\n\r\n'+body)
            process.stdin.flush()
        def wait(predicate):
            deadline = time.monotonic()+15
            while True:
                found = next((m for m in history if predicate(m)), None)
                if found is not None:
                    return found
                remaining = deadline-time.monotonic()
                self.assertGreater(remaining, 0, history)
                try:
                    history.append(received.get(timeout=remaining))
                except queue.Empty:
                    self.fail('No expected background LSP result: '+str(history))
        try:
            send('initialize', {'rootUri': path_uri(self.root), 'initializationOptions': {
                'houdiniPath': str(self.hfs), 'diagnosticsDebounceMs': 100}}, 1)
            self.assertIn('result', wait(lambda m: m.get('id') == 1))
            send('initialized')
            text = '#include <kinefx.h>\ngetpointlocaltransform(0,"0");'
            send('textDocument/didOpen', {'textDocument': {'uri': self.uri, 'version': 1, 'text': text}})
            send('textDocument/hover', {'textDocument': {'uri': self.uri}, 'position': {'line': 1, 'character': 4}}, 2)
            self.assertIn('getpointlocaltransform', wait(lambda m: m.get('id') == 2)['result']['contents']['value'])
            wait(lambda m: m.get('method') == 'textDocument/publishDiagnostics' and
                 m['params'].get('version') == 1 and any(d.get('code') == 1066 for d in m['params']['diagnostics']))
            corrected = text.replace('0,"0"', '"0",0')+'\nfloat value; printf("%g",value);'
            send('textDocument/didChange', {'textDocument': {'uri': self.uri, 'version': 2}, 'contentChanges': [{'text': corrected}]})
            warning = wait(lambda m: m.get('method') == 'textDocument/publishDiagnostics' and
                 m['params'].get('version') == 2 and any(d.get('code') == 2018 for d in m['params']['diagnostics']))
            self.assertFalse(any(d['severity'] == 1 for d in warning['params']['diagnostics']))
            send('shutdown', request_id=3)
            wait(lambda m: m.get('id') == 3)
            send('exit')
            process.stdin.close()
            self.assertEqual(process.wait(timeout=5), 0)
        finally:
            if process.poll() is None:
                process.kill()
            process.wait()
            if not process.stdin.closed:
                process.stdin.close()
            reader.join(2)
            process.stdout.close()
            process.stderr.close()

    def test_source_mode_header_errors_and_attributes(self):
        for code in ('v@P*=2; float value=missing();', 's@name="hello"; float value=missing();', 'd@meta=set("a",1);float value=missing();'):
            self.assertEqual(self.validate(code)[0]['code'], 1066)
        source = 'cvex test(){float result=missing();}'
        self.assertEqual(self.validate(source, ctx=replace(self.ctx, mode='source'))[0]['code'], 1066)
        header = self.root/'helpers.h'
        header.write_text('function float helper(float x){return missing(x);}')
        errors = self.validate('#include <helpers.h>\nhelper(1);')
        self.assertEqual(errors[0]['relatedInformation'][0]['location']['uri'], path_uri(header))
