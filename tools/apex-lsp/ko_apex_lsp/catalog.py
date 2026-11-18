"""Offline APEX function catalogue, read from Houdini's shipped ``apexscript.py``.

Houdini generates that module for its own snippet-editor autocomplete. It is parsed
with ``ast`` and never imported or executed. The real signatures are in docstrings
(``VariadicArg<Int>``, unions, ``*geo`` in-place ports) that are not valid Python,
so they are parsed tolerantly here.
"""
import ast
import hashlib
import json
import os
import re
from pathlib import Path

CACHE_VERSION = 1
NO_DOCS = 'Documentation will be added at a later point.'
_TYPE_NAME = re.compile(r'^[A-Za-z_][\w:]*$')


def find_houdini(settings, environ=None):
    """Explicit setting, then the extension's discovered/HFS variables, then installs."""
    environ = os.environ if environ is None else environ
    explicit = settings.get('houdiniPath')
    if explicit:
        return Path(os.path.expandvars(explicit)).expanduser()
    if settings.get('discoverHoudini', True) is False:
        return None
    for key in ('KO_VEX_HOUDINI', 'HFS'):
        if environ.get(key) and catalog_source(Path(environ[key])):
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
    valid = [p for p in candidates if catalog_source(p)]
    return max(valid, key=lambda p: (version(p), str(p)), default=None)


def catalog_source(hfs):
    """``$HFS/packages/apex/python3.*libs/apexscript.py`` for the newest Python."""
    if hfs is None:
        return None
    found = [p for p in (Path(hfs) / 'packages/apex').glob('python3*libs/apexscript.py') if p.is_file()]

    def minor(path):
        digits = re.findall(r'\d+', path.parent.name)
        return int(digits[-1]) if digits else 0
    return max(found, key=minor, default=None)


def split_top(text, separator=','):
    """Split on separators outside (), [] and <> nesting."""
    parts, depth, start = [], 0, 0
    for i, char in enumerate(text):
        if char in '([<':
            depth += 1
        elif char in ')]>' and depth:
            depth -= 1
        elif char == separator and depth == 0:
            parts.append(text[start:i])
            start = i+1
    parts.append(text[start:])
    return [p.strip() for p in parts if p.strip()]


def parse_param(text):
    inplace = text.startswith('*') and not text.startswith('**')
    text = text.lstrip('*')
    name, _, rest = text.partition(':')
    annotation, _, default = rest.partition('=')
    return {'name': name.strip(), 'type': annotation.strip(), 'default': default.strip(), 'inplace': inplace}


def parse_docstring(doc, method=False):
    """Return (params, returns, description, inplace) from a generated docstring."""
    lines = doc.strip('\n').splitlines()
    header, depth, index = [], 0, 0
    for index, line in enumerate(lines):
        stripped = line.strip()
        if not header and not stripped.startswith('def '):
            break
        header.append(stripped)
        depth += stripped.count('(') - stripped.count(')')
        if depth <= 0 and stripped.endswith(':'):
            break
    else:
        index = len(lines)
    params, returns = [], ''
    text = ' '.join(header)
    open_paren = text.find('(')
    if text.startswith('def ') and open_paren > 0:
        level, close = 0, -1
        for i in range(open_paren, len(text)):
            level += {'(': 1, ')': -1}.get(text[i], 0)
            if level == 0:
                close = i
                break
        if close > 0:
            params = [parse_param(p) for p in split_top(text[open_paren+1:close])]
            tail = text[close+1:].strip()
            if tail.startswith('->'):
                returns = tail[2:].strip().rstrip(':').strip()
    if method and params and params[0]['name'] == 'self':
        params = params[1:]
    body = lines[index+1:] if header else lines
    inplace = any(line.strip() == 'Inplace update' for line in body)
    kept = []
    for line in body:
        stripped = line.strip()
        if stripped == 'Inplace update' or stripped.startswith('Returns '):
            continue
        kept.append(line)
    description = '\n'.join(kept).strip()
    if description == NO_DOCS:
        description = ''
    # Collapse the generator's 80-column wrapping back into paragraphs.
    description = '\n\n'.join(' '.join(part.split()) for part in re.split(r'\n\s*\n', description) if part.strip())
    return params, returns, description, inplace


def _function(node, owner, method):
    doc = ast.get_docstring(node, clean=True) or ''
    params, returns, description, inplace = parse_docstring(doc, method)
    return {'name': node.name, 'owner': owner, 'params': params, 'returns': returns,
            'doc': description, 'inplace': inplace, 'line': node.lineno}


def build(source_text):
    """Catalogue dict: functions (global), namespaces, types."""
    tree = ast.parse(source_text)
    classes = {n.name: n for n in tree.body if isinstance(n, ast.ClassDef)}
    functions = {n.name: _function(n, '', False) for n in tree.body if isinstance(n, ast.FunctionDef)}
    namespaces, types = {}, {}

    def add_type(name, node):
        doc = ast.get_docstring(next((m for m in node.body if isinstance(m, ast.FunctionDef) and m.name == '__init__'), node)) or ''
        methods = {m.name: _function(m, name, True) for m in node.body
                   if isinstance(m, ast.FunctionDef) and m.name != '__init__'}
        types[name] = {'doc': ' '.join(doc.split()), 'methods': methods, 'line': node.lineno}

    def add_namespace(path, node, seen):
        if path in namespaces or node.name in seen:
            return
        seen = seen | {node.name}
        entry = {'functions': {}, 'children': {}, 'line': node.lineno}
        namespaces[path] = entry
        for member in node.body:
            if isinstance(member, ast.FunctionDef):
                entry['functions'][member.name] = _function(member, path, False)
            elif isinstance(member, ast.ClassDef):
                # Object types scoped to a namespace, e.g. geo::IntersectCache.
                add_type(path.split('.', 1)[-1].replace('.', '::') + '::' + member.name, member)
            elif (isinstance(member, ast.Assign) and len(member.targets) == 1
                  and isinstance(member.targets[0], ast.Name) and isinstance(member.value, ast.Name)
                  and member.value.id in classes):
                child = path + '.' + member.targets[0].id
                entry['children'][member.targets[0].id] = child
                add_namespace(child, classes[member.value.id], seen)

    if 'apex' in classes:
        add_namespace('apex', classes['apex'], set())
    for name, node in classes.items():
        if name != 'apex' and not name.startswith('namespace_') and name != 'void':
            add_type(name, node)
    return {'version': CACHE_VERSION, 'functions': functions, 'namespaces': namespaces, 'types': types}


def load(source, cache_dir=None):
    """Parse ``source`` (cached as JSON keyed by path, mtime and size)."""
    source = Path(source)
    stat = source.stat()
    key = hashlib.sha1(f'{CACHE_VERSION}|{source.resolve()}|{stat.st_mtime_ns}|{stat.st_size}'.encode()).hexdigest()[:16]
    cache = Path(cache_dir) / f'apex-catalog-{key}.json' if cache_dir else None
    if cache and cache.is_file():
        try:
            data = json.loads(cache.read_text(encoding='utf-8'))
            if data.get('version') == CACHE_VERSION:
                return Catalog(data, source)
        except (OSError, ValueError):
            pass
    data = build(source.read_text(encoding='utf-8'))
    if cache:
        try:
            cache.parent.mkdir(parents=True, exist_ok=True)
            temporary = cache.with_suffix('.tmp')
            temporary.write_text(json.dumps(data, separators=(',', ':')), encoding='utf-8')
            os.replace(temporary, cache)
            # One retained catalogue per cache directory; old Houdini builds are dropped.
            for old in cache.parent.glob('apex-catalog-*.json'):
                if old != cache:
                    old.unlink(missing_ok=True)
        except OSError:
            pass
    return Catalog(data, source)


def name_key(name):
    """APEX matches function names with a case-insensitive first letter (Sine == sine)."""
    return name[:1].lower() + name[1:]


class Catalog:
    def __init__(self, data, source=None):
        self.source = source
        self.functions = data['functions']
        self.namespaces = data['namespaces']
        self.types = data['types']
        self._globals = {name_key(n): f for n, f in self.functions.items()}
        self._ns_index = {p: {name_key(n): f for n, f in e['functions'].items()} for p, e in self.namespaces.items()}
        self._method_index = {t: {name_key(n): f for n, f in e['methods'].items()} for t, e in self.types.items()}

    @classmethod
    def empty(cls):
        return cls({'functions': {}, 'namespaces': {}, 'types': {}})

    def add_methods(self, methods):
        """Merge hand-listed object methods {type: {name: (params, returns, maps_to)}}."""
        for type_name, entries in methods.items():
            if type_name not in self.types:
                continue
            existing = self.types[type_name]['methods']
            for name, (params, returns, maps_to) in entries.items():
                if name_key(name) in self._method_index[type_name]:
                    continue
                entry = {'name': name, 'owner': type_name, 'params': [parse_param(p) for p in split_top(params)],
                         'returns': returns, 'doc': f'Maps to `{maps_to}`.', 'inplace': False, 'line': None}
                existing[name] = entry
                self._method_index[type_name][name_key(name)] = entry

    def _lookup(self, index, name):
        found = index.get(name_key(name))
        if found is None and '_' in name:
            # Templated spellings: setPrimAttribValue_String -> setPrimAttribValue.
            base, _, suffix = name.rpartition('_')
            if suffix in self.types or suffix.rstrip('_') in self.types:
                found = index.get(name_key(base))
        return found

    def function(self, name):
        return self._lookup(self._globals, name)

    def namespace_function(self, path, name):
        found = self._lookup(self._ns_index.get(path, {}), name)
        # apex.sine(...) is the same as sine(...).
        return found if found or path != 'apex' else self.function(name)

    def method(self, type_name, name):
        return self._lookup(self._method_index.get(type_name, {}), name)

    def child(self, path, name):
        return self.namespaces.get(path, {}).get('children', {}).get(name)


def single_type(annotation):
    """The one APEX type an annotation names, or '' for unions/tuples/unknown."""
    annotation = (annotation or '').strip()
    if not annotation.startswith(('tuple[', 'list[')):
        annotation = re.sub(r'\[[^\]]*\]$', '', annotation)  # Float[outputy] -> Float
    return annotation if _TYPE_NAME.match(annotation) and annotation != 'None' else ''


def signature(entry):
    params = ', '.join(('*' if p['inplace'] else '') + p['name'] + (': ' + p['type'] if p['type'] else '')
                       + ('=' + p['default'] if p['default'] else '') for p in entry['params'])
    owner = entry['owner']
    prefix = '' if not owner else owner + '.'
    returns = ' -> ' + entry['returns'] if entry['returns'] else ''
    return f'{prefix}{entry["name"]}({params}){returns}'
