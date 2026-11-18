"""Houdini installation discovery and the $HFS/houdini/ocl/include header index."""
import os
import re
from pathlib import Path

from .source import INCLUDE, definitions, directive_name, tokenize

MAX_HEADER = 4 * 1024 * 1024
MAX_FILES = 256
# Houdini's OpenCL snippets include matrix.h (and through it typedefines.h) by default.
DEFAULT_INCLUDES = ('matrix.h',)


def include_root(hfs):
    return Path(hfs) / 'houdini' / 'ocl' / 'include' if hfs else None


def find_houdini(settings, environ=None):
    """Explicit setting, then the extension's discovered/HFS variables, then installs."""
    environ = os.environ if environ is None else environ
    explicit = settings.get('houdiniPath')
    if explicit:
        return Path(os.path.expandvars(explicit)).expanduser()
    if settings.get('discoverHoudini', True) is False:
        return None

    def valid(path):
        root = include_root(path)
        return root is not None and (root / 'matrix.h').is_file()
    for key in ('KO_VEX_HOUDINI', 'HFS'):
        if environ.get(key) and valid(Path(environ[key])):
            return Path(environ[key])
    candidates = []
    if os.name == 'nt':
        for base in dict.fromkeys(environ.get(k) for k in ('ProgramW6432', 'ProgramFiles') if environ.get(k)):
            candidates.extend((Path(base) / 'Side Effects Software').glob('Houdini *'))
    else:
        candidates.extend(Path('/opt').glob('hfs*'))
        candidates.extend(Path('/Applications/Houdini').glob('Houdini*/Frameworks/Houdini.framework/Versions/Current/Resources'))

    def version(path):
        found = re.findall(r'(\d+)\.(\d+)(?:\.(\d+))?', str(path))
        return tuple(int(n or 0) for n in found[-1]) if found else (0, 0, 0)
    return max((p for p in candidates if valid(p)), key=lambda p: (version(p), str(p)), default=None)


class Header:
    def __init__(self, path, text):
        self.path = path
        self.text = text
        self.tokens = tokenize(text)
        self.definitions = [d for d in definitions(text, self.tokens) if d.kind not in ('variable', 'parameter')]
        self.includes = [(m[2], m[3].strip()) for t in self.tokens if t.kind == 'directive'
                         and directive_name(t.text) in ('include', 'import') for m in [INCLUDE.match(t.text)] if m]
        self.binds = [t for t in self.tokens if t.kind == 'directive' and directive_name(t.text) == 'bind']


class HeaderIndex:
    def __init__(self):
        self.cache = {}   # path -> (mtime, size, Header)

    def header(self, path):
        try:
            stat = path.stat()
        except OSError:
            return None
        cached = self.cache.get(path)
        if cached and cached[0] == stat.st_mtime_ns and cached[1] == stat.st_size:
            return cached[2]
        if stat.st_size > MAX_HEADER:
            return None
        try:
            text = path.read_text(encoding='utf-8', errors='replace')
        except OSError:
            return None
        header = Header(path, text)
        if len(self.cache) > 4 * MAX_FILES:
            self.cache.clear()
        self.cache[path] = (stat.st_mtime_ns, stat.st_size, header)
        return header

    @staticmethod
    def resolve(name, quote, directory, search):
        """Path for #include <name> / "name", or None."""
        candidates = ([Path(directory)] if quote == '"' and directory else []) + [Path(p) for p in search if p]
        for base in candidates:
            path = base / name
            if path.is_file():
                return path.resolve()
        return None

    def closure(self, includes, directory, search, defaults=DEFAULT_INCLUDES):
        """Headers reachable from ``includes`` [(quote, name)] plus the defaults, in order."""
        seen, ordered = set(), []
        pending = [('<', n, None) for n in defaults] + [(q, n, directory) for q, n in includes]
        while pending and len(ordered) < MAX_FILES:
            quote, name, base = pending.pop(0)
            path = self.resolve(name, quote, base, search)
            if path is None or path in seen:
                continue
            seen.add(path)
            header = self.header(path)
            if header is None:
                continue
            ordered.append(header)
            pending.extend((q, n, path.parent) for q, n in header.includes)
        return ordered

    def files(self, search):
        """Relative header paths under the include roots, for #include completion."""
        result = []
        for root in search:
            root = Path(root)
            if not root.is_dir():
                continue
            for path in sorted(root.rglob('*.h')):
                result.append(path.relative_to(root).as_posix())
                if len(result) >= 2000:
                    return result
        return result
