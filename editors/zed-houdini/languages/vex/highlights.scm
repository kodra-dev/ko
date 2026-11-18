(comment) @comment
(string) @string
(number) @number
(primitive_type) @type.builtin
(type_identifier) @type
(qualifier) @keyword
(preprocessor) @preproc
(attribute) @variable.special
(identifier) @variable
(function_definition name: (identifier) @function)
(call_expression function: (expression (identifier) @function))
(member_expression member: (identifier) @property)
(struct_definition name: (identifier) @type)
["function" "struct" "if" "else" "for" "foreach" "while" "do" "return" "break" "continue"] @keyword
["=" "+=" "-=" "*=" "/=" "%=" "+" "-" "*" "/" "%" "==" "!=" "<" ">" "<=" ">=" "&&" "||" "!" "~" "&" "|" "^" "<<" ">>" "++" "--" "?" ":" "->"] @operator
["(" ")" "[" "]" "{" "}"] @punctuation.bracket
[";" "," "."] @punctuation.delimiter
