"""Tokens, positions and definitions for OpenCL source text (no compiler)."""
import bisect
import re
from dataclasses import dataclass, field

KEYWORDS = ('if else for while do switch case default return break continue goto sizeof typedef struct '
            'union enum const volatile restrict static inline extern register unsigned signed kernel __kernel '
            'global __global local __local constant __constant private __private generic __generic '
            'read_only __read_only write_only __write_only read_write __read_write __attribute__').split()
QUALIFIERS = set(KEYWORDS[KEYWORDS.index('const'):])
CONTROL = {'if', 'for', 'while', 'switch', 'return', 'sizeof', 'do', 'else', 'case', 'goto'}
SCALARS = 'char uchar short ushort int uint long ulong half float double bool'.split()
TYPES = (['void', 'size_t', 'ptrdiff_t', 'intptr_t', 'uintptr_t', 'image1d_t', 'image2d_t', 'image3d_t',
          'image1d_array_t', 'image2d_array_t', 'image1d_buffer_t', 'sampler_t', 'event_t']
         + SCALARS + [f'{t}{n}' for t in SCALARS if t != 'bool' for n in (2, 3, 4, 8, 16)])
# Houdini's own types from typedefines.h / matrix.h, documented in vex/ocl.
HOUDINI_TYPES = {
    'fpreal': 'Floating point whose precision follows the node or geometry (float or double).',
    'exint': 'Integer whose precision follows the node or geometry (int or long).',
    'mat2': '2x2 matrix (a `fpreal4`), from matrix.h.',
    'mat3': '3x3 matrix: an array of three `fpreal3` rows, so 12 floats in memory, from matrix.h.',
    'mat32': '3x2 matrix: an array of three `fpreal2`, from matrix.h.',
    'mat4': '4x4 matrix (a `fpreal16`), from matrix.h.',
    'mat3d': '3x3 double matrix, from matrix.h.',
}
for _n in (2, 3, 4, 8, 16):
    HOUDINI_TYPES[f'fpreal{_n}'] = f'{_n}-component `fpreal` vector (float{_n} or double{_n}).'
for _n in (2, 3, 4):
    HOUDINI_TYPES[f'exint{_n}'] = f'{_n}-component `exint` vector (int{_n} or long{_n}).'
TYPE_NAMES = set(TYPES) | set(HOUDINI_TYPES)

TOKEN = re.compile(r'''
    (?P<comment>//(?:[^\n\\]|\\.|\\\n)*|/\*.*?(?:\*/|\Z))
  | (?P<directive>(?m:^)[ \t]*\#(?:[^\n\\]|\\[^\n]|\\\n)*)
  | (?P<string>"(?:[^"\\\n]|\\.)*"?|'(?:[^'\\\n]|\\.)*'?)
  | (?P<binding>@[A-Za-z_]\w*)
  | (?P<number>(?:0[xX][0-9a-fA-F]+|(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)[A-Za-z]*)
  | (?P<name>[A-Za-z_]\w*)
  | (?P<op><<=|>>=|->|\+\+|--|<<|>>|<=|>=|==|!=|&&|\|\||[-+*/%&|^!~=<>?:;,.(){}\[\]@#\\])
''', re.S | re.X)


@dataclass(frozen=True)
class Token:
    kind: str
    text: str
    start: int
    end: int


def tokenize(text, limit=None):
    tokens = []
    for match in TOKEN.finditer(text, 0, len(text) if limit is None else limit):
        kind = match.lastgroup
        value = match[0]
        start = match.start()
        if kind == 'directive':
            stripped = value.lstrip()
            start += len(value) - len(stripped)
            value = stripped
        tokens.append(Token(kind, value, start, match.end()))
    return tokens


class Text:
    """Offsets <-> LSP (line, UTF-16 character) positions."""

    def __init__(self, text):
        self.text = text
        self.lines = [0] + [m.end() for m in re.finditer('\n', text)]

    def position(self, offset):
        offset = max(0, min(offset, len(self.text)))
        line = bisect.bisect_right(self.lines, offset) - 1
        start = self.lines[line]
        return {'line': line, 'character': len(self.text[start:offset].encode('utf-16-le')) // 2}

    def range(self, start, end):
        return {'start': self.position(start), 'end': self.position(end)}

    def offset(self, position):
        line = min(max(position.get('line', 0), 0), len(self.lines) - 1)
        start = self.lines[line]
        end = self.lines[line + 1] - 1 if line + 1 < len(self.lines) else len(self.text)
        units, i = position.get('character', 0), start
        while i < end and units > 0:
            units -= 2 if ord(self.text[i]) > 0xFFFF else 1
            i += 1
        return i


def directive_name(token_text):
    match = re.match(r'#\s*([A-Za-z_]\w*)', token_text)
    return match[1] if match else ''


INCLUDE = re.compile(r'#\s*(include|import)\s*([<"])([^>"\n]*)[>"]?')
DEFINE = re.compile(r'#\s*define\s+([A-Za-z_]\w*)(\([^)]*\))?[ \t]*(.*)', re.S)


@dataclass
class Definition:
    name: str
    kind: str              # function | macro | struct | typedef | variable | parameter | kernel | enumerator
    start: int             # name offsets
    end: int
    detail: str = ''       # prototype / #define line
    doc: str = ''
    params: list = field(default_factory=list)   # parameter texts
    body: tuple = None     # (start, end) of the whole definition
    scope: tuple = None    # (start, end) in which a local is visible


def _doc_before(tokens, index, text):
    """Comment text directly above tokens[index] (no blank line in between)."""
    parts, i, anchor = [], index - 1, tokens[index].start
    while i >= 0 and tokens[i].kind == 'comment':
        between = text[tokens[i].end:anchor]
        if between.count('\n') > 1 or between.strip():
            break
        parts.insert(0, tokens[i].text)
        anchor = tokens[i].start
        i -= 1
    lines = []
    for part in parts:
        if part.startswith('//'):
            lines.append(part[2:].lstrip('/!<').strip())
        else:
            for line in part[2:-2].splitlines():
                lines.append(line.strip().lstrip('*').strip())
    return '\n'.join(l for l in lines if l and not set(l) <= set('-=*/ ')).strip()


def _match(code, i, opener, closer):
    depth = 0
    while i < len(code):
        if code[i].text == opener:
            depth += 1
        elif code[i].text == closer:
            depth -= 1
            if depth == 0:
                return i
        i += 1
    return len(code) - 1


def _split_params(code, open_index, close_index, text):
    params, depth, start = [], 0, open_index + 1
    for i in range(open_index + 1, close_index + 1):
        t = code[i].text
        if t in '([{':
            depth += 1
        elif t in ')]}' and i != close_index:
            depth -= 1
        if (t == ',' and depth == 0) or i == close_index:
            if start < i:
                params.append((' '.join(text[code[start].start:code[i - 1].end].split()), code[start:i]))
            start = i + 1
    return params


def definitions(text, tokens=None):
    """Functions, kernels, macros, structs, typedefs and declared variables of ``text``."""
    tokens = tokenize(text) if tokens is None else tokens
    result = []
    for t in tokens:
        if t.kind == 'directive' and directive_name(t.text) == 'define':
            m = DEFINE.match(t.text)
            if m:
                name_at = t.start + t.text.index(m[1], t.text.index('define') + 6)
                result.append(Definition(m[1], 'macro', name_at, name_at + len(m[1]),
                                         detail=' '.join(t.text.replace('\\\n', ' ').split()),
                                         doc=_doc_before(tokens, tokens.index(t), text),
                                         params=[p.strip() for p in m[2][1:-1].split(',')] if m[2] else []))
    code_index = [i for i, t in enumerate(tokens) if t.kind not in ('comment', 'directive')]
    code = [tokens[i] for i in code_index]
    i, depth, scopes = 0, 0, []
    statement_start = 0
    while i < len(code):
        t = code[i]
        if t.text == '{':
            depth += 1
            scopes.append(t.start)
            statement_start = i + 1
        elif t.text == '}':
            depth = max(0, depth - 1)
            if scopes:
                scopes.pop()
            statement_start = i + 1
        elif t.text == ';':
            statement_start = i + 1
        elif t.kind == 'binding' and t.text in ('@KERNEL', '@WRITEBACK') and i + 1 < len(code) and code[i + 1].text == '{':
            end = _match(code, i + 1, '{', '}')
            result.append(Definition(t.text, 'kernel', t.start, t.end, detail=t.text, body=(t.start, code[end].end)))
        elif t.kind == 'name' and t.text in ('struct', 'union', 'enum') and i + 1 < len(code) and code[i + 1].kind == 'name':
            if i + 2 < len(code) and code[i + 2].text == '{':
                n = code[i + 1]
                end = _match(code, i + 2, '{', '}')
                result.append(Definition(n.text, 'struct', n.start, n.end, detail=f'{t.text} {n.text}',
                                         doc=_doc_before(tokens, code_index[i], text), body=(t.start, code[end].end)))
        elif t.kind == 'name' and t.text == 'typedef':
            end = i
            level = 0
            while end < len(code) and not (code[end].text == ';' and level == 0):
                level += code[end].text in '({['
                level -= code[end].text in ')}]'
                end += 1
            names = [c for c in code[i:end] if c.kind == 'name' and c.text not in QUALIFIERS]
            if end < len(code) and names:
                # typedef struct {...} Name;  /  typedef fpreal3 mat3[3];
                last = names[-1]
                result.append(Definition(last.text, 'typedef', last.start, last.end,
                                         detail=' '.join(text[t.start:code[end].end].split())[:200],
                                         doc=_doc_before(tokens, code_index[i], text)))
        elif (t.kind == 'name' and t.text not in KEYWORDS and i + 1 < len(code) and code[i + 1].text == '('
              and i > statement_start and depth == 0):
            prefix = code[statement_start:i]
            if prefix and all(p.kind == 'name' or p.text in ('*',) for p in prefix) and not any(p.text in CONTROL for p in prefix):
                close = _match(code, i + 1, '(', ')')
                after = close + 1
                while after < len(code) and code[after].text == '__attribute__':
                    after = _match(code, after + 1, '(', ')') + 1
                if after < len(code) and code[after].text in ('{', ';'):
                    params = _split_params(code, i + 1, close, text)
                    proto = ' '.join(text[prefix[0].start:code[close].end].split())
                    kind = 'kernel' if any(p.text in ('kernel', '__kernel') for p in prefix) else 'function'
                    body_end = code[_match(code, after, '{', '}')].end if code[after].text == '{' else code[after].end
                    definition = Definition(t.text, kind, t.start, t.end, detail=proto,
                                            doc=_doc_before(tokens, code_index[statement_start], text),
                                            params=[p for p, _ in params if p != 'void'], body=(prefix[0].start, body_end))
                    result.append(definition)
                    if code[after].text == '{':
                        for _, ptoks in params:
                            names = [p for p in ptoks if p.kind == 'name' and p.text not in QUALIFIERS and p.text not in TYPE_NAMES]
                            if len(names) >= 1 and len([p for p in ptoks if p.kind == 'name']) >= 2:
                                n = names[-1]
                                result.append(Definition(n.text, 'parameter', n.start, n.end,
                                                         detail=' '.join(text[ptoks[0].start:ptoks[-1].end].split()),
                                                         scope=(code[after].start, body_end)))
                    i = close
        elif (t.kind == 'name' and i > statement_start and i + 1 < len(code) and code[i + 1].text in ('=', ';', ',', '[')
              and t.text not in KEYWORDS):
            prefix = code[statement_start:i]
            names = [p for p in prefix if p.kind == 'name']
            if (names and all(p.kind == 'name' or p.text == '*' for p in prefix)
                    and not any(p.text in CONTROL or p.text in ('struct', 'union', 'enum', 'typedef') for p in prefix)
                    and (names[-1].text in TYPE_NAMES or len(names) >= 1)):
                scope = (scopes[-1], len(text)) if scopes else (0, len(text))
                result.append(Definition(t.text, 'variable', t.start, t.end,
                                         detail=' '.join(text[prefix[0].start:t.end].split()), scope=scope))
        i += 1
    return result
