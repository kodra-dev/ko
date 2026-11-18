(comment) @comment
(string) @string
(char_literal) @string
(number) @number
(primitive_type) @type.builtin
(type_identifier) @type
(qualifier) @keyword
(macro_qualifier) @keyword
(preprocessor) @preproc
(include_directive) @preproc
(bind_directive) @preproc
(kernel_marker) @keyword
(binding) @variable.special
(identifier) @variable
(function_definition name: (identifier) @function)
(call_expression function: (expression (identifier) @function))
(call_expression function: (expression (member_expression member: (identifier) @function.method)))
(member_expression member: (identifier) @property)
(macro_invocation name: (identifier) @function.macro)
(designated_initializer field: (identifier) @property)
(struct_definition name: (identifier) @type)
(struct_body_type name: (identifier) @type)
(enum_definition name: (identifier) @type)
(enumerator name: (identifier) @constant)
((identifier) @constant
  (#match? @constant "^[A-Z][A-Z0-9_]+$"))
["struct" "union" "enum" "typedef" "if" "else" "for" "while" "do" "switch" "case" "default"
 "return" "break" "continue" "goto" "sizeof" "__attribute__"] @keyword
["=" "+=" "-=" "*=" "/=" "%=" "&=" "|=" "^=" "<<=" ">>=" "+" "-" "*" "/" "%" "==" "!=" "<" ">"
 "<=" ">=" "&&" "||" "!" "~" "&" "|" "^" "<<" ">>" "++" "--" "?" ":" "->"] @operator
["(" ")" "[" "]" "{" "}"] @punctuation.bracket
[";" "," "."] @punctuation.delimiter
