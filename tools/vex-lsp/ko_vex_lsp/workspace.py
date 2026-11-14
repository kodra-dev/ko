"""Bounded, lazy include graph and installation-backed builtin documentation."""
from pathlib import Path
import hashlib
import json
import os
import re
import subprocess
import tempfile
import zipfile

from .model import Document, INCLUDE, Token, path_uri, uri_path
from .discovery import find_houdini

MAX_FILE = 4 * 1024 * 1024
MAX_HEADERS = 256
MAX_GRAPH_BYTES = 16 * 1024 * 1024


class IncludeGraph(list):
    def __init__(self):
        super().__init__()
        self.issues = []


class Workspace:
    def __init__(self, root, settings=None):
        self.root = Path(root).resolve()
        self.open = {}
        self.cache = {}
        self.configure(settings or {})

    def configure(self, settings):
        self.settings = settings
        self.hfs = find_houdini(settings, self.root) or self.root / '.vex-no-houdini'
        self.context = settings.get('context', 'cvex')
        self.mode = settings.get('mode', 'snippet')
        self.timeout = min(30, max(1, settings.get('compilerTimeout', 8)))
        self.vcc = self.hfs / 'bin' / ('vcc.exe' if os.name == 'nt' else 'vcc')
        cache = Path(os.environ.get('KO_VEX_CACHE') or str(Path(tempfile.gettempdir()) / 'ko-vex-lsp/docs'))
        self.documentation_dir = cache / hashlib.sha256(str(self.hfs).encode('utf-8')).hexdigest()[:12]
        self.include_paths = []
        for p in settings.get('includePaths', []):
            path = Path(os.path.expandvars(p)).expanduser()
            self.include_paths.append((self.root / path).resolve() if not path.is_absolute() else path.resolve())
        # Conventional project-local libraries need no per-workspace setup.
        self.include_paths.extend([self.root / 'vex/include', self.root / 'include'])
        # HOUDINI_PATH is useful only when already expanded by Houdini/a shell.
        # Do not execute package scripts or interpret Houdini's &, @, = tokens.
        for p in os.environ.get('HOUDINI_PATH', '').split(os.pathsep):
            if p and not any(c in p for c in '&@=$'):
                self.include_paths.append(Path(p).resolve() / 'vex/include')
        for p in sorted((self.hfs / 'packages').glob('*/vex/include')):
            self.include_paths.append(p.resolve())
        self.include_paths.append((self.hfs / 'houdini/vex/include').resolve())
        self.include_paths = list(dict.fromkeys(p for p in self.include_paths if p.is_dir()))
        self.builtins = {}
        self.builtin_error = ''
        self.doc_cache = {}
        self.cache.clear()
        if self.vcc.is_file():
            try:
                result = self.run([str(self.vcc), '--list-context-json=' + self.context])
                self.builtins = json.loads(result.stdout)['functions']
            except (OSError, ValueError, KeyError, subprocess.TimeoutExpired) as e:
                self.builtin_error = str(e)
        else:
            self.builtin_error = 'Houdini was not found. Set lsp.ko-vex.settings.houdiniPath in Zed to enable builtins and compiler validation.'

    def run(self, command):
        return subprocess.run(command, capture_output=True, encoding='utf-8', errors='replace',
                              timeout=self.timeout, cwd=self.root,
                              creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))

    def document(self, uri):
        if uri in self.open:
            return self.open[uri]
        path = uri_path(uri)
        stat = path.stat()
        if stat.st_size > MAX_FILE:
            raise ValueError('File exceeds the 4 MiB index limit')
        stamp = (stat.st_mtime_ns, stat.st_size)
        if uri not in self.cache or self.cache[uri][0] != stamp:
            self.cache[uri] = stamp, Document(uri, path.read_text(encoding='utf-8-sig'))
        return self.cache[uri][1]

    def resolve_include(self, doc, name, quoted):
        paths = ([uri_path(doc.uri).parent] if quoted else []) + self.include_paths
        for base in paths:
            path = (base / name).resolve()
            if path.is_file() or path_uri(path) in self.open:
                return path_uri(path)
        return None

    def includes(self, doc):
        for token in doc.tokens:
            if token.kind != 'directive':
                continue
            m = INCLUDE.match(token.text)
            if m:
                yield token, m, self.resolve_include(doc, m[2], m[1] == '"')

    def graph(self, doc):
        result, seen, pending = IncludeGraph(), set(), [doc]
        total_bytes = 0
        while pending and len(result) < MAX_HEADERS:
            current = pending.pop()
            if current.uri in seen:
                continue
            seen.add(current.uri)
            total_bytes += len(current.text.encode('utf-8'))
            if total_bytes > MAX_GRAPH_BYTES:
                result.issues.append('Include graph exceeds 16 MiB')
                break
            result.append(current)
            children = []
            for token in current.tokens:
                if token.kind == 'directive' and re.match(r'^\s*#\s*include\b', token.text) and not INCLUDE.match(token.text):
                    result.issues.append('Macro-generated include cannot be indexed: ' + token.text.strip())
            for _, match, uri in self.includes(current):
                if not uri:
                    result.issues.append('Header not found: ' + match[2])
                elif uri not in seen:
                    try:
                        children.append(self.document(uri))
                    except (OSError, ValueError) as error:
                        result.issues.append('Cannot index header: ' + str(error))
            pending.extend(reversed(children))
        if pending:
            result.issues.append('Include graph traversal limit reached')
        types = set().union(*(d.declared_struct_types for d in result))
        for d in result:
            d.reindex_types(types)
        return result

    def declarations(self, doc):
        return [(d, s) for d in self.graph(doc) for s in d.root.symbols]

    def functions(self, doc):
        return [(d, s) for d, s in self.declarations(doc) if s.kind in (12, 23)]

    def macros(self, doc):
        result = []
        for d in self.graph(doc):
            for t in d.tokens:
                if t.kind == 'directive':
                    m = re.match(r'^\s*#\s*define\s+([A-Za-z_]\w*)', t.text)
                    if m:
                        start = t.start + m.start(1)
                        result.append((d, Token(m[1], start, start+len(m[1]), 'id'), t.text.strip()))
        return result

    def members(self, doc, type_name):
        base = type_name.split('[', 1)[0]
        return [(d, member) for d, s in self.functions(doc)
                if s.kind == 23 and s.name == base and s.body
                for member in s.body.symbols]

    def receiver_type(self, doc, tokens):
        """Conservative structural inference: locals/attributes, fields and arrays."""
        from .compiler import ATTR_TYPES, KNOWN_ATTRS
        if not tokens:
            return None
        last = tokens[-1]
        if last.text == ']':
            depth = 0
            for i in range(len(tokens)-1, -1, -1):
                depth += tokens[i].text == ']'
                depth -= tokens[i].text == '['
                if depth == 0:
                    typ = self.receiver_type(doc, tokens[:i])
                    return (typ if any(t.text == ':' for t in tokens[i+1:-1]) else typ.split('[', 1)[0]) if typ and '[' in typ else None
        if len(tokens) >= 3 and tokens[-2].text in ('.', '->'):
            receiver = self.receiver_type(doc, tokens[:-2])
            found = {s.type for _, s in self.members(doc, receiver or '') if s.name == last.text}
            return next(iter(found)) if len(found) == 1 else None
        if last.kind == 'attribute':
            prefix, name = last.text.split('@')
            typ = ATTR_TYPES.get(prefix.replace('[]', '')) if prefix else KNOWN_ATTRS.get(name)
            return typ + ('[]' if '[]' in prefix else '') if typ else None
        if last.kind == 'id':
            s = doc.resolve(last)
            if s:
                return s.type
            found = {s.type for d, s in self.declarations(doc) if s.name == last.text and s.kind == 13}
            return next(iter(found)) if len(found) == 1 else None
        return None

    def signatures(self, name, doc):
        results = [(s.signature or s.type + ' ' + s.name, s.doc, d.uri) for d, s in self.functions(doc) if s.name == name]
        for overload in self.builtins.get(name, []):
            args = list(overload.get('args', []))
            if overload.get('variadic'):
                args.append('...')
            sig = overload.get('return', 'void') + ' ' + name + '(' + '; '.join(args) + ')'
            results.append((sig, self.documentation(name), None))
        return results

    def documentation(self, name):
        if name in self.doc_cache:
            return self.doc_cache[name]
        content = ''
        archive = self.hfs / 'houdini/help/vex.zip'
        if archive.is_file():
            try:
                with zipfile.ZipFile(archive) as z:
                    for key in ('vex/functions/' + name + '.txt', 'functions/' + name + '.txt'):
                        if key in z.namelist():
                            raw = z.read(key).decode('utf-8', 'replace')
                            # Convert the small subset of SideFX wiki markup used in API
                            # pages. Keep usage signatures and code, including #include.
                            lines = []
                            in_code = False
                            for line in raw.splitlines():
                                if re.match(r'^#[A-Za-z_]+:', line) or line.startswith('#!'):
                                    continue
                                if line.strip() == '{{{':
                                    lines.append('```vex')
                                    in_code = True
                                elif line.strip() == '}}}':
                                    lines.append('```')
                                    in_code = False
                                elif line.startswith('@'):
                                    lines.append('### ' + line[1:].replace('_', ' ').title())
                                elif not in_code:
                                    line = re.sub(r'^:usage:\s*', '', line)
                                    line = re.sub(r'^=+\s*(.*?)\s*=+$', r'# \1', line)
                                    line = line.replace('"""', '')
                                    line = re.sub(r'<<([^>]+)>>', r'`\1`', line)
                                    line = re.sub(r'\[Vex:([\w]+)\]', r'[\1](https://www.sidefx.com/docs/houdini/vex/functions/\1.html)', line)
                                    lines.append(line.removeprefix('    '))
                                else:
                                    lines.append(line)
                            content = '\n'.join(lines)[:12000].strip()
                            break
            except (OSError, zipfile.BadZipFile):
                pass
        content += '\n\n[SideFX documentation](https://www.sidefx.com/docs/houdini/vex/functions/' + name + '.html)'
        self.doc_cache[name] = content.strip()
        return self.doc_cache[name]

    def include_completions(self, doc, prefix, quoted):
        paths = ([uri_path(doc.uri).parent] if quoted else []) + self.include_paths
        results = set()
        parent, _, leaf = prefix.rpartition('/')
        for base in paths:
            directory = base / parent
            if directory.is_dir():
                for p in directory.iterdir():
                    if p.name.startswith(leaf) and (p.is_dir() or p.suffix in ('.h', '.vfl')):
                        results.add((parent+'/' if parent else '') + p.name + ('/' if p.is_dir() else ''))
        return sorted(results)[:500]
