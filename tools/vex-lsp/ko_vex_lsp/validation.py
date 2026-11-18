"""Bounded, cancellable validation without AST dumps or executing VEX code.

Owned by one diagnostics worker. Mutable indexes/mirrors are never shared with
completion/refactoring. The common literal-include case needs one vcc process;
main-file preprocessing uses vcc -E before moving expanded headers outside CVEX.
"""
from array import array
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
import hashlib
import os
import re
import subprocess
import tempfile
import time

from .compiler import ATTR_TYPES, KNOWN_ATTRS
from .model import Document, TOKEN, Token, path_uri, uri_key, uri_path
from .workspace import Workspace, MAX_FILE

MAX_OUTPUT = 1024 * 1024
MAX_PREPROCESSED = 16 * 1024 * 1024
CACHE_BYTES = 16 * 1024 * 1024
CACHE_FILES = 128
MAX_MODEL_BYTES = 32 * 1024 * 1024
CONTEXTS = {'cvex', 'sop', 'surface', 'displacement', 'light', 'shadow', 'fog', 'chop', 'cop2', 'image3d'}


class Cancelled(Exception):
    pass


class Unavailable(Exception):
    pass


class LightDocument:
    """Only positions, directives and attributes; no scope tree or identifier index."""
    position = Document.position
    range = Document.range

    def __init__(self, uri, text, version=None, cancel=None):
        self.uri, self.text, self.version = uri, text, version
        encoded = text.encode('utf-8')
        self.digest = hashlib.sha256(encoded).digest()
        text_cost = len(encoded)*2
        del encoded
        def line_offsets():
            yield 0
            for index, match in enumerate(re.finditer('\n', text)):
                if index % 512 == 0:
                    if cancel is not None and cancel.is_set():
                        raise Cancelled()
                    if text_cost+(index+2)*4 > MAX_MODEL_BYTES:
                        raise Unavailable('diagnostics line index exceeds 32 MiB')
                yield match.end()
        self.lines = array('I', line_offsets())
        self.cost = text_cost + len(self.lines)*self.lines.itemsize
        self.tokens = []
        self.is_source = False
        previous = []
        for index, m in enumerate(TOKEN.finditer(text)):
            if index % 512 == 0 and cancel is not None and cancel.is_set():
                raise Cancelled()
            kind = m.lastgroup
            if kind in ('directive', 'attribute'):
                self.cost += 200
                if self.cost > MAX_MODEL_BYTES:
                    raise Unavailable('diagnostics source index exceeds 32 MiB')
                self.tokens.append(Token(m.group(), m.start(), m.end(), kind))
            if kind not in ('directive', 'comment', 'string'):
                if m.group() == '(' and len(previous) == 2 and previous[0] in CONTEXTS:
                    self.is_source = True
                previous = (previous + [m.group()])[-2:]
        self.declared_struct_types = set()

    def reindex_types(self, _):
        pass


@dataclass(frozen=True)
class Context:
    root: Path
    vcc: Path
    include_paths: tuple
    mode: str
    context: str
    timeout: float
    attributes: tuple
    max_problems: int

    @classmethod
    def capture(cls, workspace):
        settings = workspace.settings
        return cls(workspace.root, workspace.vcc, tuple(workspace.include_paths), workspace.mode,
                   workspace.context, workspace.timeout, tuple(sorted(settings.get('attributeTypes', {}).items())),
                   min(200, max(1, int(settings.get('diagnosticsMaxProblems', 50)))))


def file_stamp(uri):
    try:
        s = uri_path(uri).stat()
        return s.st_mtime_ns, s.st_size
    except OSError:
        return None


class SnapshotWorkspace(Workspace):
    def __init__(self):
        # Deliberately do not invoke Workspace.configure / enumerate builtins.
        self.cache = OrderedDict()
        self.cache_bytes = 0
        self.buffers = {}
        self.stamps = {}

    def reset(self, context, buffers, cancel):
        self.root, self.vcc, self.include_paths = context.root, context.vcc, list(context.include_paths)
        self.mode, self.context, self.timeout = context.mode, context.context, context.timeout
        self.buffers, self.cancel = buffers, cancel
        self.buffer_aliases = {uri_key(uri): value for uri, value in buffers.items()}
        self.stamps = {}
        self.snapshot_seen, self.snapshot_cost = set(), 0
        self.open = buffers  # Workspace.resolve_include only tests membership.

    def resolve_include(self, doc, name, quoted):
        for base in ([uri_path(doc.uri).parent] if quoted else [])+self.include_paths:
            candidate = (base/name).resolve()
            uri = path_uri(candidate)
            if candidate.is_file() or uri_key(uri) in self.buffer_aliases:
                return uri
        return None

    def account(self, uri, doc):
        if uri not in self.snapshot_seen:
            self.snapshot_seen.add(uri)
            self.snapshot_cost += doc.cost
            if self.snapshot_cost > MAX_MODEL_BYTES:
                raise Unavailable('diagnostics snapshot index exceeds 32 MiB')

    def document(self, uri):
        if self.cancel.is_set():
            raise Cancelled()
        opened = self.buffers.get(uri, self.buffer_aliases.get(uri_key(uri)))
        stamp = ('open', opened[1]) if opened else file_stamp(uri)
        if stamp is None:
            raise FileNotFoundError(uri)
        self.stamps[uri] = stamp
        old = self.cache.get(uri)
        if old and old[0] == stamp:
            self.account(uri, old[1])
            self.cache.move_to_end(uri)
            self.stamps[uri] = stamp
            return old[1]
        text, version = opened if opened else (None, None)
        if text is None:
            if stamp[1] > MAX_FILE:
                raise ValueError('Header exceeds 4 MiB')
            text = uri_path(uri).read_text(encoding='utf-8-sig')
            if file_stamp(uri) != stamp:
                raise Cancelled('Header changed while reading')
        doc = LightDocument(uri, text, version, self.cancel)
        self.account(uri, doc)
        if old:
            self.cache_bytes -= old[1].cost
            del self.cache[uri]
        if doc.cost <= CACHE_BYTES:
            self.cache[uri] = (stamp, doc)
            self.cache_bytes += doc.cost
        while self.cache_bytes > CACHE_BYTES or len(self.cache) > CACHE_FILES:
            _, (_, removed) = self.cache.popitem(last=False)
            self.cache_bytes -= removed.cost
        self.stamps[uri] = stamp
        return doc

    def clear_buffers(self):
        # Do not retain all open text snapshots between checks.
        self.buffers = self.open = {}
        self.buffer_aliases = {}
        # Open-buffer models are cheap to reconstruct after the debounce; keeping
        # closed versions here would retain otherwise unreachable document text.
        for uri in [uri for uri, (stamp, _) in self.cache.items() if stamp[0] == 'open']:
            self.cache_bytes -= self.cache.pop(uri)[1].cost


@dataclass
class ValidationResult:
    diagnostics: list
    dependencies: frozenset
    stamps: dict
    retry: bool = False


def unavailable(doc, message):
    span = doc.range(0, 0) if doc is not None else {'start': {'line': 0, 'character': 0}, 'end': {'line': 0, 'character': 0}}
    return {'range': span, 'severity': 3, 'source': 'vcc', 'code': 'validation-unavailable',
            'message': 'VEX validation unavailable: ' + message}


def run_compiler(command, context, cancel, deadline, directory, output_limit=MAX_OUTPUT):
    """File-backed capture bounds RAM; kill/reap promptly on edits, limits or timeout."""
    if cancel.is_set():
        raise Cancelled()
    flags = getattr(subprocess, 'CREATE_NO_WINDOW', 0) | getattr(subprocess, 'BELOW_NORMAL_PRIORITY_CLASS', 0)
    with tempfile.TemporaryFile() as stdout, tempfile.TemporaryFile() as stderr:
        process = subprocess.Popen(command, cwd=context.root, stdin=subprocess.DEVNULL,
                                   stdout=stdout, stderr=stderr, creationflags=flags)
        try:
            output = Path(directory) / 'check.vex'
            if os.name != 'nt' and hasattr(os, 'setpriority'):
                try:
                    os.setpriority(os.PRIO_PROCESS, process.pid, 10)
                except OSError:
                    pass
            while process.poll() is None:
                if cancel.wait(0.04):
                    raise Cancelled()
                if time.monotonic() >= deadline:
                    raise Unavailable('compiler time limit reached; increase compilerTimeout if needed')
                if os.fstat(stdout.fileno()).st_size > output_limit or os.fstat(stderr.fileno()).st_size > MAX_OUTPUT:
                    raise Unavailable('compiler output limit reached')
                if output.exists() and output.stat().st_size > MAX_PREPROCESSED:
                    raise Unavailable('compiled output exceeds 16 MiB')
            if cancel.is_set():
                raise Cancelled()
            if output.exists() and output.stat().st_size > MAX_PREPROCESSED:
                raise Unavailable('compiled output exceeds 16 MiB')
            stdout.seek(0)
            stderr.seek(0)
            out, err = stdout.read(output_limit + 1), stderr.read(MAX_OUTPUT + 1)
            if len(out) > output_limit or len(err) > MAX_OUTPUT:
                raise Unavailable('compiler output limit reached')
            return process.returncode, out.decode('utf-8', 'replace'), err.decode('utf-8', 'replace')
        finally:
            if process.poll() is None:
                process.kill()
            process.wait()
            if output.exists() and output.stat().st_size > MAX_PREPROCESSED:
                output.unlink(missing_ok=True)


def _normalized(path):
    # vcc sometimes reports Windows 8.3 paths or doubled backslashes.
    normalized = str(Path(str(path).replace('\\\\', '\\')).resolve()).replace('\\', '/')
    return normalized.casefold() if os.name == 'nt' else normalized


def parse_messages(message, doc, documents, paths, shift=0, imprecise=False, limit=50):
    lookup = {_normalized(paths[d.uri]): d for d in documents}
    lookup.update({_normalized(str(uri_path(d.uri))): d for d in documents})
    pattern = re.compile(r'^(.+?):(\d+)(?::(\d+)(?:-(\d+))?)?:\s*(Error|Warning)\s*(\d+)?\s*:\s*(.*)$')
    result, seen = [], set()
    for line in message.splitlines():
        match = pattern.match(line.strip())
        if not match:
            # Compiler overload notes belong to the preceding diagnostic, not
            # unrelated full-file errors; keep them bounded and readable.
            if result and line.strip() and len(result[-1]['message']) < 4000:
                result[-1]['message'] += '\n' + line.strip()[:1000]
            continue
        target = lookup.get(_normalized(match[1]))
        if target is None:
            # Still expose failures in a native macro-resolved include, with a
            # real related location. Do not read arbitrary files just for errors.
            row = max(0, int(match[2])-1)
            uri = path_uri(match[1].replace('\\\\', '\\'))
            span = {'start': {'line': row, 'character': 0}, 'end': {'line': row, 'character': 0}}
        else:
            row = int(match[2])-1-(shift if target is doc else 0)
            if row < 0:
                continue  # Generated parameter declarations have no user span.
            if row >= len(target.lines):
                if target is not doc:
                    continue
                # A missing final semicolon/closing delimiter is reported at the
                # generated closing brace. Map that syntax error to user EOF.
                row = len(target.lines)-1
                span, uri = doc.range(len(doc.text), len(doc.text)), doc.uri
            else:
                span = None
            start = target.lines[row]
            raw = target.text[start:].split('\n', 1)[0].rstrip('\r')
            if imprecise or not match[3]:
                a, b = start, start+len(raw)
            else:
                offset = lambda n: start + len(raw.encode('utf-8')[:n].decode('utf-8', 'ignore'))
                a, b = offset(int(match[3])-1), offset(int(match[4] or match[3]))
            if span is None:
                span, uri = target.range(a, b), target.uri
        diagnostic = {'range': span if uri == doc.uri else doc.range(0, 0),
                      'severity': 1 if match[5] == 'Error' else 2, 'source': 'vcc',
                      'message': match[7][:4000], 'code': int(match[6]) if match[6] else match[5].lower()}
        if uri != doc.uri:
            diagnostic['message'] = uri_path(uri).name + ': ' + diagnostic['message']
            diagnostic['relatedInformation'] = [{'location': {'uri': uri, 'range': span}, 'message': match[7][:4000]}]
        key = (str(diagnostic['range']), diagnostic['code'], diagnostic['message'])
        if key not in seen:
            result.append(diagnostic)
            seen.add(key)
        if len(result) >= limit:
            break
    return result


def replace_spans(text, spans, cancel=None):
    """Linear string slices, not an eight-byte pointer per source character."""
    parts, position = [], 0
    for index, (start, end, replacement) in enumerate(spans):
        if index % 512 == 0 and cancel is not None and cancel.is_set():
            raise Cancelled()
        parts.extend((text[position:start], replacement))
        position = end
    if not parts:
        return text
    parts.append(text[position:])
    return ''.join(parts)


def attributes(doc, context, cancel=None):
    bindings, replacements = {}, []
    configured = dict(context.attributes)
    for index, t in enumerate(doc.tokens):
        if index % 512 == 0 and cancel is not None and cancel.is_set():
            raise Cancelled()
        if t.kind != 'attribute':
            continue
        prefix, name = t.text.split('@')
        typ = ATTR_TYPES.get(prefix.replace('[]', '')) if prefix else configured.get(name, KNOWN_ATTRS.get(name))
        if not typ:
            raise Unavailable('unknown type for '+t.text+'; add a VEX type prefix or configure attributeTypes')
        if prefix and '[]' in prefix:
            typ += '[]'
        if not isinstance(typ, str) or not re.fullmatch(r'(?:int|float|vector2|vector|vector4|matrix2|matrix3|matrix|string|dict)(?:\[\])?', typ):
            raise Unavailable('unsupported attributeTypes entry for '+name)
        replacement = '_'*(len(prefix)+1)+name
        if replacement in bindings and bindings[replacement] != typ:
            raise Unavailable('inconsistent types for '+t.text)
        bindings[replacement] = typ
        replacements.append((t.start, t.end, replacement))
    if bindings:
        for index, match in enumerate(TOKEN.finditer(doc.text)):
            if index % 512 == 0 and cancel is not None and cancel.is_set():
                raise Cancelled()
            if match.lastgroup == 'id' and match.group() in bindings:
                raise Unavailable('generated attribute name conflicts with '+match.group())
    parameters = []
    for name, typ in bindings.items():
        default = '{}' if typ == 'dict' or typ.endswith('[]') else '""' if typ == 'string' else '0'
        parameters.append('export '+typ.replace('[]', '')+' '+name+('[]' if typ.endswith('[]') else '')+' = '+default)
    return replace_spans(doc.text, replacements, cancel), '; '.join(parameters)


class Validator:
    def __init__(self, runner=run_compiler):
        self.workspace = SnapshotWorkspace()
        self.runner = runner
        self.results = OrderedDict()
        self.temporary = tempfile.TemporaryDirectory(prefix='ko-vex-diagnostics-')
        self.directory = Path(self.temporary.name)
        self.written = {}
        self.written_bytes = 0
        self.context = None

    def close(self):
        self.workspace.clear_buffers()
        self.workspace.cache.clear()
        self.results.clear()
        self.written.clear()
        self.temporary.cleanup()

    def invalidate(self, uris):
        """Watcher events are authoritative even if size/mtime were preserved."""
        keys = None if uris is None else {uri_key(uri) for uri in uris}
        for uri in list(self.workspace.cache):
            if keys is None or uri_key(uri) in keys:
                self.workspace.cache_bytes -= self.workspace.cache.pop(uri)[1].cost

    def mirror_path(self, uri):
        # Keep parent relationships for macro-based quoted includes in overlays.
        original = uri_path(uri)
        relative = str(original).replace('\\', '/').lstrip('/').replace(':', '')
        return self.directory / 'mirror' / relative

    def write(self, path, text):
        digest = hashlib.sha256(text.encode('utf-8')).digest()
        previous = self.written.get(path)
        if previous is None or previous[0] != digest:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text, encoding='utf-8', newline='')
            size = len(text.encode('utf-8'))
            self.written_bytes += size-(previous[1] if previous else 0)
            self.written[path] = (digest, size)

    def validate(self, uri, context, buffers, cancel):
        if self.context != context:
            # Configuration changes invalidate both compiler results and mirrors.
            self.results.clear()
            self.workspace.cache.clear()
            self.workspace.cache_bytes = 0
            self.context = context
        self.workspace.reset(context, buffers, cancel)
        try:
            try:
                result = self._validate(uri, context, cancel)
            except Unavailable as error:
                result = ValidationResult([unavailable(None, str(error))],
                    frozenset(dep for dep in self.workspace.stamps if dep != uri), dict(self.workspace.stamps))
            if any(stamp[0] != 'open' and file_stamp(dep) != stamp for dep, stamp in self.workspace.stamps.items()):
                result.retry = True
            return result
        finally:
            self.workspace.clear_buffers()
            # Disk mirrors are an optimization, not an unbounded historical store.
            if len(self.written) > 512 or self.written_bytes > 32*1024*1024:
                self.written.clear()
                self.written_bytes = 0
                for path in self.directory.glob('**/*'):
                    if path.is_file():
                        path.unlink()

    def _validate(self, uri, context, cancel):
        doc = self.workspace.document(uri)
        graph = self.workspace.graph(doc)
        dependencies = frozenset(d.uri for d in graph if d.uri != uri)
        stamps = dict(self.workspace.stamps)
        if cancel.is_set():
            raise Cancelled()
        if any('limit' in issue or 'exceeds' in issue or 'Cannot index' in issue for issue in graph.issues):
            return ValidationResult([unavailable(doc, '; '.join(graph.issues[:2]))], dependencies, stamps)
        cacheable = all(issue.startswith('Header not found:') for issue in graph.issues)
        key = (context, file_stamp(path_uri(context.vcc)), tuple((d.uri, d.digest) for d in graph))
        if cacheable and key in self.results:
            self.results.move_to_end(key)
            return ValidationResult(self.results[key], dependencies, stamps)
        paths = {d.uri: self.mirror_path(d.uri) for d in graph}
        complex_preprocessing = any(t.kind == 'directive' and not re.match(r'^\s*#\s*include\s*[<"]', t.text) for t in doc.tokens)
        complex_preprocessing |= any('Macro-generated' in issue for issue in graph.issues)
        # Only ambiguous preprocessing needs extra open-header overlays. Ordinary
        # jobs retain/check only actual dependencies, not every open header.
        if complex_preprocessing or any('Macro-generated' in issue for issue in graph.issues):
            for open_uri in self.workspace.buffers:
                if open_uri not in paths and uri_path(open_uri).suffix in ('.h', '.vexh', '.vfl'):
                    paths[open_uri] = self.mirror_path(open_uri)
        documents = list(graph)
        for open_uri in paths:
            if open_uri not in {d.uri for d in documents}:
                documents.append(self.workspace.document(open_uri))
        if sum(len(d.text.encode('utf-8')) for d in documents) > MAX_PREPROCESSED:
            return ValidationResult([unavailable(doc, 'open include overlay exceeds 16 MiB')], dependencies, stamps)
        dependencies = frozenset(d.uri for d in documents if d.uri != uri)
        paths[uri] = self.directory/'main.vfl'  # Main needs no relative macro includes: add its mirror parent below.
        aliases = {uri_key(original): path for original, path in paths.items()}
        for d in documents:
            text = d.text
            for token, match, resolved in reversed(list(self.workspace.includes(d))):
                if resolved:
                    target = paths.get(resolved, aliases.get(uri_key(resolved), uri_path(resolved)))
                    replacement = '#include "'+target.as_posix()+'"'
                    replacement += '\n' * token.text.count('\n')
                    text = text[:token.start]+replacement+text[token.end:]
            self.write(paths[d.uri], text)
        deadline = time.monotonic()+context.timeout
        options = ['-q', '--fmessage-limit', str(context.max_problems), '-c', context.context]
        include_paths = list(dict.fromkeys([self.mirror_path(uri).parent, uri_path(uri).parent] +
                         [self.mirror_path(path_uri(p/'dummy.h')).parent for p in context.include_paths] +
                         list(context.include_paths) + [uri_path(d.uri).parent for d in documents[1:]]))
        for path in include_paths:
            options += ['-I', str(path)]
        source_mode = context.mode == 'source' or doc.is_source or uri_path(uri).suffix in ('.h', '.vexh')
        shift, imprecise = 0, False
        try:
            if not source_mode or any('Macro-generated' in issue for issue in graph.issues):
                if complex_preprocessing:
                    preprocessing_started = time.time_ns()
                    status, expanded, errors = self.runner([str(context.vcc), '-E']+options+[str(paths[uri])],
                                                         context, cancel, deadline, self.directory, MAX_PREPROCESSED)
                    parsed = parse_messages(errors, doc, documents, paths, limit=context.max_problems)
                    if status:
                        return ValidationResult(parsed or [unavailable(doc, errors[-1500:])], dependencies, stamps)
                    global_lines, body_lines, destination = [], [], None
                    aliases = {_normalized(path): original for original, path in paths.items()}
                    actual_dependencies = set()
                    marker = re.compile(r'^#(?:line)?\s+(\d+)\s+"(.*)"')
                    main_key = _normalized(paths[uri])
                    for index, line in enumerate(expanded.splitlines(keepends=True)):
                        if index % 512 == 0:
                            if cancel.is_set():
                                raise Cancelled()
                            if time.monotonic() >= deadline:
                                raise Unavailable('preprocessing time limit reached')
                        m = marker.match(line)
                        if m:
                            normalized = _normalized(m[2])
                            destination = body_lines if normalized == main_key else global_lines
                            original = aliases.get(normalized)
                            if original is None and not m[2].startswith('<'):
                                original = path_uri(m[2].replace('\\\\', '\\'))
                                stamp = file_stamp(original)
                                if stamp is not None:
                                    self.workspace.stamps[original] = stamp
                                    if stamp[0] > preprocessing_started:
                                        return ValidationResult([], dependencies, dict(self.workspace.stamps), retry=True)
                            if original is not None and original != uri:
                                actual_dependencies.add(original)
                        if destination is not None:
                            destination.append(line)
                    dependencies = frozenset(actual_dependencies)
                    if source_mode:
                        generated = expanded
                    else:
                        body = LightDocument(uri, ''.join(body_lines), doc.version, cancel)
                        text, params = attributes(body, context, cancel)
                        generated = ''.join(global_lines)+'\ncvex __ko_vex_diagnostics('+params+') {\n'+text+'\n}\n'
                    imprecise = True  # Macro expansion can change columns; use correct full source lines.
                else:
                    original = LightDocument(uri, paths[uri].read_text(encoding='utf-8'), doc.version, cancel)
                    text, params = attributes(original, context, cancel)
                    blanks, includes = [], []
                    for t in original.tokens:
                        if t.kind == 'directive':
                            includes.append(t.text)
                            blanks.append((t.start, t.end, ''.join('\n' if c == '\n' else ' ' for c in t.text)))
                    prefix = '\n'.join(includes+['cvex __ko_vex_diagnostics('+params+') {'])+'\n'
                    generated, shift = prefix+replace_spans(text, blanks, cancel)+'\n}\n', prefix.count('\n')
                self.write(paths[uri], generated)
            command = [str(context.vcc), '-z']+(['-d'] if source_mode else [])+options+['-o', str(self.directory/'check.vex'), str(paths[uri])]
            (self.directory/'check.vex').unlink(missing_ok=True)
            status, _, errors = self.runner(command, context, cancel, deadline, self.directory)
            diagnostics = parse_messages(errors, doc, documents, paths, shift, imprecise, context.max_problems)
            if status and not diagnostics:
                diagnostics = [unavailable(doc, errors[-1500:] or 'compiler failed without a mapped diagnostic')]
        except Unavailable as error:
            diagnostics = [unavailable(doc, str(error))]
        # Never publish a snapshot compiled while a disk dependency was changing.
        if any(stamp[0] != 'open' and file_stamp(dep) != stamp for dep, stamp in self.workspace.stamps.items()):
            return ValidationResult([], dependencies, stamps, retry=True)
        if cacheable and not any(d.get('code') == 'validation-unavailable' for d in diagnostics):
            self.results[key] = diagnostics
            self.results.move_to_end(key)
            while len(self.results) > 16:
                self.results.popitem(last=False)
        return ValidationResult(diagnostics, dependencies, dict(self.workspace.stamps))
