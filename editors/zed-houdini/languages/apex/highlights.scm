; APEX Script on Zed's built-in tree-sitter-python grammar.
; Later patterns take precedence in Zed, so general rules come first.

(identifier) @variable

(attribute
  attribute: (identifier) @property)

; APEX types are CamelCase (Float, Vector3, Geometry, ApexGraphHandle).
((identifier) @type
  (#match? @type "^[A-Z][A-Za-z0-9_]*$"))

; Constants: M_PI, RED, ...
((identifier) @constant
  (#match? @constant "^[A-Z][A-Z0-9_]*$"))

(type (identifier) @type)
(generic_type (identifier) @type)

(comment) @comment
(string) @string
(escape_sequence) @string.escape

(interpolation
  "{" @punctuation.special
  "}" @punctuation.special) @embedded

[(integer) (float)] @number
[(true) (false)] @boolean
(none) @constant.builtin

(call
  function: (identifier) @function.call)

(call
  function: (attribute
    attribute: (identifier) @function.method.call))

; apex.sop.box(...): the apex namespace root.
((identifier) @namespace
  (#eq? @namespace "apex"))

(function_definition
  name: (identifier) @function.definition)

(function_definition
  parameters: (parameters
    [
      (identifier) @variable.parameter
      (typed_parameter (identifier) @variable.parameter)
      (default_parameter name: (identifier) @variable.parameter)
      (typed_default_parameter name: (identifier) @variable.parameter)
    ]))

(keyword_argument
  name: (identifier) @property)

(decorator
  "@" @attribute
  [
    (identifier) @attribute
    (call function: (identifier) @attribute)
  ])

; Special functions and graph-building built-ins.
((call
  function: (identifier) @function.builtin)
  (#any-of? @function.builtin
    "BindInput" "BindOutput" "HoudiniVersion" "BeginMetadata" "EndMetadata" "InputMetadata"
    "OutputMetadata" "InvokeOutputs" "SetGraph" "setGraph" "StickyNote" "_portalias" "print"
    "warning" "error" "len" "range" "enumerate" "addToFolder" "bindMultiparm" "addToMultiparm"))

[
  "-" "-=" "!=" "*" "**" "**=" "*=" "/" "/=" "+" "+=" "->" "<" "<=" "=" "==" ">" ">=" "%" "|"
] @operator

[
  "and"
  "in"
  "is"
  "not"
  "or"
] @keyword.operator

[
  "def"
  "return"
  "if"
  "elif"
  "else"
  "for"
  "pass"
  "raise"
  "import"
  "from"
  "while"
  "class"
  "try"
  "with"
  "global"
  "break"
  "continue"
  "assert"
  "del"
  "yield"
] @keyword

["." "," ":"] @punctuation.delimiter
["(" ")" "[" "]" "{" "}"] @punctuation.bracket
