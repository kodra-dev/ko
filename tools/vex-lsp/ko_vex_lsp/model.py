"""VEX source model. Positions are UTF-16 LSP positions, never byte offsets.

The tolerant index is for browsing/completion. Refactoring additionally requires
compiler validation; this is deliberately not advertised as a VEX type checker.
"""
from dataclasses import dataclass, field
from pathlib import Path
import bisect
import re
from urllib.parse import urlparse
from urllib.request import url2pathname

TYPES = set('void int float vector2 vector vector4 matrix2 matrix3 matrix string dict bsdf'.split())
QUALIFIERS = {'const', 'export', 'uniform', 'varying'}
KEYWORDS = set('function struct if else for foreach while do break continue return cvex sop surface displacement light shadow fog chop cop2 image3d'.split()) | TYPES | QUALIFIERS
TOKEN = re.compile(r'(?P<comment>//[^\n]*|/\*[\s\S]*?(?:\*/|\Z))|(?P<directive>^[ \t]*\#(?:[^\n\\]|\\[^\n]|\\\n)*)|(?P<string>"(?:\\[\s\S]|[^"\\])*"|\x27(?:\\[\s\S]|[^\x27\\])*\x27)|(?P<attribute>(?:[ifvpu234sd]\[\]|[ifvpu234sd])?@[A-Za-z_]\w*)|(?P<id>[A-Za-z_]\w*)|(?P<number>(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?[fF]?)|(?P<op>->|::|==|!=|<=|>=|\+=|-=|\*=|/=|\+\+|--|&&|\|\||<<|>>|[^\s])', re.M)
INCLUDE = re.compile(r'^\s*#\s*include\s*([<"])([^>"\n]+)[>"]', re.M)


def path_uri(path):
    return Path(path).resolve().as_uri()


def uri_path(uri):
    parsed = urlparse(uri)
    if parsed.scheme != 'file':
        raise ValueError('Only file: documents are supported')
    value = url2pathname(parsed.path)
    if parsed.netloc:
        value = '//' + parsed.netloc + value
    return Path(value).resolve()


@dataclass(frozen=True)
class Token:
    text: str
    start: int
    end: int
    kind: str


@dataclass(eq=False)
class Scope:
    start: int
    end: int
    parent: object = None
    symbols: list = field(default_factory=list)


@dataclass(eq=False)
class Symbol:
    name: str
    token: Token
    type: str
    kind: int
    scope: Scope
    signature: str = ''
    doc: str = ''
    body: object = None


class Document:
    def __init__(self, uri, text, version=None):
        self.uri, self.text, self.version = uri, text, version
        self.lines = [0] + [m.end() for m in re.finditer('\n', text)]
        self.tokens = [Token(m.group(), m.start(), m.end(), m.lastgroup) for m in TOKEN.finditer(text)]
        self.code = [t for t in self.tokens if t.kind not in ('comment', 'directive')]
        self.code_ends = [t.end for t in self.code]
        self.comments = [t for t in self.tokens if t.kind == 'comment']
        self.comment_ends = [t.end for t in self.comments]
        self.starts = [t.start for t in self.tokens]
        self.pairs = {}
        stack = []
        self.errors = []
        for i, t in enumerate(self.code):
            if t.text in ('(', '[', '{'):
                stack.append(i)
            elif t.text in (')', ']', '}'):
                if stack and self.code[stack[-1]].text == {')': '(', ']': '[', '}': '{'}[t.text]:
                    j = stack.pop()
                    self.pairs[j] = i
                    self.pairs[i] = j
                else:
                    self.errors.append((t, 'Unmatched closing delimiter'))
        self.errors.extend((self.code[i], 'Unclosed delimiter') for i in stack)
        self.declared_struct_types = {self.code[i+1].text for i, t in enumerate(self.code[:-1]) if t.text == 'struct'}
        self.struct_types = None
        self.reindex_types(self.declared_struct_types)

    def reindex_types(self, types):
        types = set(types) | self.declared_struct_types
        if types == self.struct_types:
            return
        self.struct_types = types
        self.root = Scope(0, len(self.text))
        self.scopes = [self.root]
        self.symbols = []
        self.unsupported = False
        self._parse(0, len(self.code), self.root)

    def offset(self, position):
        line = position['line']
        if line >= len(self.lines):
            return len(self.text)
        start = self.lines[max(0, line)]
        end = self.lines[line+1] if line+1 < len(self.lines) else len(self.text)
        units = position['character']
        for i in range(start, end):
            if units <= 0:
                return i
            units -= 2 if ord(self.text[i]) > 0xffff else 1
        return end

    def position(self, offset):
        line = bisect.bisect_right(self.lines, offset) - 1
        return {'line': line, 'character': len(self.text[self.lines[line]:offset].encode('utf-16-le')) // 2}

    def range(self, start, end):
        return {'start': self.position(start), 'end': self.position(end)}

    def token_at(self, offset):
        i = bisect.bisect_right(self.starts, offset) - 1
        return self.tokens[i] if i >= 0 and offset <= self.tokens[i].end else None

    def scope_at(self, offset):
        return min((s for s in self.scopes if s.start <= offset <= s.end), key=lambda s: s.end-s.start, default=self.root)

    def resolve(self, token, name=None):
        if token.kind != 'id':
            return None
        # Member names are not local variables.
        previous_index = bisect.bisect_right(self.code_ends, token.start) - 1
        previous = self.code[previous_index] if previous_index >= 0 else None
        if previous and previous.text in ('.', '->', '::'):
            return None
        name = name or token.text
        scope = self.scope_at(token.start)
        while scope:
            candidates = [s for s in scope.symbols if s.name == name and (s.token.start <= token.start or s.kind == 12)]
            if candidates:
                return candidates[-1]
            scope = scope.parent
        return None

    def visible(self, offset):
        result = {}
        scope = self.scope_at(offset)
        while scope:
            for s in reversed(scope.symbols):
                if s.token.start <= offset or s.kind == 12:
                    result.setdefault(s.name, s)
            scope = scope.parent
        return list(result.values())

    def _scope(self, start, end, parent):
        scope = Scope(start, end, parent)
        self.scopes.append(scope)
        return scope

    def _add(self, token, typ, kind, scope, signature='', body=None):
        before = self.text[:token.start]
        # Adjacent comment block, not a randomly distant comment.
        doc = ''
        comment_index = bisect.bisect_right(self.comment_ends, token.start) - 1
        if comment_index >= 0:
            c = self.comments[comment_index]
            between = before[c.end:]
            if between.count('\n') <= 2 and not any(x in between for x in ';{}'):
                doc = re.sub(r'^\s*(?:///?|/\*+|\*+/|\*) ?', '', c.text, flags=re.M).strip()
        s = Symbol(token.text, token, typ, kind, scope, signature, doc, body)
        scope.symbols.append(s)
        self.symbols.append(s)
        return s

    def _declaration(self, i, end, scope, parameters=False):
        start = i
        if i < end and self.code[i].text == 'function':
            i += 1
        while i < end and self.code[i].text in QUALIFIERS:
            i += 1
        if i >= end or self.code[i].text not in TYPES | self.struct_types | {'cvex', 'sop', 'surface', 'displacement'}:
            return None
        typ = self.code[i].text
        i += 1
        while i+1 < end and self.code[i].text == '[' and self.code[i+1].text == ']':
            typ += '[]'
            i += 2
        if i >= end or self.code[i].kind != 'id':
            return None
        name = self.code[i]
        i += 1
        if i < end and self.code[i].text == '(' and i in self.pairs:
            close = self.pairs[i]
            after = close + 1
            if after < end and self.code[after].text in ('{', ';'):
                body_end = self.pairs.get(after, after)
                sig = self.text[self.code[start].start:self.code[close].end]
                s = self._add(name, typ, 12, scope, sig)
                if self.code[after].text == '{':
                    fn = self._scope(self.code[i].end, self.code[body_end].end, scope)
                    s.body = fn
                    self._parse(i+1, close, fn, parameters=True)
                    self._parse(after+1, body_end, fn)
                    return body_end + 1
                return after + 1
        while True:
            suffix = ''
            while i < end and self.code[i].text == '[' and i in self.pairs:
                suffix += self.text[self.code[i].start:self.code[self.pairs[i]].end]
                i = self.pairs[i] + 1
            self._add(name, typ+suffix, 13, scope)
            while i < end and self.code[i].text not in (',', ';', ')'):
                if self.code[i].text in ('(', '[', '{') and i in self.pairs:
                    i = self.pairs[i] + 1
                else:
                    i += 1
            if i < end and self.code[i].text == ',':
                i += 1
                if i < end and self.code[i].text in TYPES | QUALIFIERS | self.struct_types:
                    return i
                if i < end and self.code[i].kind == 'id':
                    name = self.code[i]
                    i += 1
                    continue
            return i+1 if i < end else end

    def _parse(self, start, end, scope, parameters=False):
        i = start
        while i < end:
            t = self.code[i]
            if t.text == 'struct':
                self.unsupported = True  # member rename needs type resolution
                if i+2 < end and self.code[i+2].text == '{' and i+2 in self.pairs:
                    close = self.pairs[i+2]
                    member_scope = self._scope(self.code[i+2].start, self.code[close].end, scope)
                    self._add(self.code[i+1], 'struct', 23, scope, body=member_scope)
                    self._parse(i+3, close, member_scope)
                    for member in member_scope.symbols:
                        if member.kind == 13:
                            member.kind = 8
                    i = close + 1
                    continue
            if t.text in ('for', 'foreach') and i+1 < end and self.code[i+1].text == '(' and i+1 in self.pairs:
                close = self.pairs[i+1]
                after = close+1
                if after < end and self.code[after].text == '{' and after in self.pairs:
                    finish = self.pairs[after]
                    loop = self._scope(self.code[i+1].end, self.code[finish].end, scope)
                    self._parse(i+2, close, loop, parameters=True)
                    self._parse(after+1, finish, loop)
                    i = finish+1
                    continue
                self.unsupported = True  # unbraced loop scope is not modeled
            if t.text == '{' and i in self.pairs:
                close = self.pairs[i]
                child = self._scope(t.start, self.code[close].end, scope)
                self._parse(i+1, close, child)
                i = close+1
                continue
            # Declarations are recognized at statement/parameter boundaries only.
            boundary = i == start or self.code[i-1].text in (';', '{', '}', ':') or (parameters and self.code[i-1].text == ',')
            if boundary:
                following = self._declaration(i, end, scope, parameters)
                if following is not None:
                    i = following
                    continue
            i += 1
