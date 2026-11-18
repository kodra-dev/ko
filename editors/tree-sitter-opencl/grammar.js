// Structural grammar for Houdini OpenCL snippets and kernels: OpenCL C plus
// Houdini's @-bindings, @KERNEL/@WRITEBACK blocks and #bind directives.
// The language server, not this grammar, resolves names and validates bindings.
//
// Preprocessor lines are "extras" (allowed anywhere), because Houdini kernels put
// #ifdef HAS_name inside parameter lists, expressions and initializers.
const commaSep1 = rule => seq(rule, repeat(seq(',', rule)));
const PREC = { assign: 1, conditional: 2, or: 3, and: 4, bitor: 5, xor: 6, bitand: 7,
  equal: 8, relational: 9, shift: 10, add: 11, multiply: 12, cast: 13, unary: 14, postfix: 15 };
module.exports = grammar({
  name: 'houdini_opencl',
  extras: $ => [/\s/, /\\\r?\n/, $.comment, $.bind_directive, $.include_directive, $.preprocessor],
  word: $ => $.identifier,
  conflicts: $ => [
    [$.type_specifier, $.expression],
    [$.macro_invocation, $.expression],
  ],
  rules: {
    source_file: $ => repeat($._item),
    _item: $ => choice($.kernel_block, $.function_definition, $.struct_definition, $.enum_definition,
      $.typedef_declaration, $.macro_invocation, $._statement),
    _statement: $ => choice($.declaration, $.block, $.if_statement, $.for_statement, $.while_statement,
      $.do_statement, $.switch_statement, $.case_label, $.return_statement, $.jump_statement,
      $.expression_statement, ';'),
    block: $ => seq('{', repeat(choice($._statement, $.struct_definition, $.enum_definition, $.typedef_declaration)), '}'),

    // @KERNEL { ... } and @WRITEBACK { ... } expand to generated kernels.
    kernel_block: $ => seq(field('name', $.kernel_marker), field('body', $.block)),
    kernel_marker: _ => token(prec(2, choice('@KERNEL', '@WRITEBACK'))),
    // A top-level macro that expands to definitions: INSTANTIATE(float, 3)
    macro_invocation: $ => prec.dynamic(-1, seq(field('name', $.identifier), $.macro_arguments)),
    macro_arguments: $ => seq('(', repeat(choice($.macro_arguments, $._macro_token)), ')'),
    _macro_token: $ => choice($.identifier, $.primitive_type, $.qualifier, $.number, $.string,
      ',', '*', '&', '+', '-', '/', '=', '.', '[', ']', '{', '}', ';', '<', '>', '#', '##'),

    type_specifier: $ => choice($.primitive_type, alias($.identifier, $.type_identifier),
      seq(choice('struct', 'union', 'enum'), alias($.identifier, $.type_identifier))),
    primitive_type: _ => token(choice(
      /(u?(char|short|int|long)|half|float|double|fpreal|exint|bool)(2|3|4|8|16)?/,
      /(unsigned|signed)([ \t]+(char|short|int|long([ \t]+long)?))?/, /long[ \t]+long/,
      'void', 'size_t', 'ptrdiff_t', 'intptr_t', 'uintptr_t',
      'image1d_t', 'image2d_t', 'image3d_t', 'image2d_array_t', 'image1d_array_t', 'image1d_buffer_t',
      'sampler_t', 'event_t', /mat(2|3|4|32|3d)/)),
    qualifier: _ => choice('const', 'volatile', 'restrict', 'static', 'inline', 'extern', 'register',
      'kernel', '__kernel', 'global', '__global', 'local', '__local',
      'constant', '__constant', 'private', '__private', 'generic', '__generic',
      'read_only', '__read_only', 'write_only', '__write_only', 'read_write', '__read_write'),
    attribute_specifier: $ => seq('__attribute__', '(', '(', optional(commaSep1($.expression)), ')', ')'),
    _specifier: $ => choice($.qualifier, $.attribute_specifier),
    pointer: $ => prec.right(seq('*', repeat($.qualifier))),
    // An all-caps macro that expands to a qualifier: float *RESTRICT data
    macro_qualifier: $ => $.identifier,
    array_suffix: $ => seq('[', optional($.expression), ']'),

    function_definition: $ => seq(repeat($._specifier), field('type', $.type_specifier), repeat($._specifier),
      repeat($.pointer), field('name', $.identifier), field('parameters', $.parameter_list),
      repeat($.attribute_specifier), choice(field('body', $.block), ';')),
    parameter_list: $ => seq('(', optional(commaSep1(choice($.parameter, '...'))), ')'),
    parameter: $ => seq(repeat($._specifier), $.type_specifier, repeat($._specifier), repeat($.pointer),
      optional(seq(optional($.macro_qualifier), field('name', $.identifier))), repeat($.array_suffix)),

    declaration: $ => seq(repeat($._specifier), $.type_specifier, repeat($._specifier), commaSep1($.declarator), ';'),
    declarator: $ => seq(repeat($.pointer), optional($.macro_qualifier), field('name', $.identifier), repeat($.array_suffix),
      repeat($.attribute_specifier), optional(seq('=', $._init_value))),
    _init_value: $ => choice($.expression, $.initializer),
    typedef_declaration: $ => seq('typedef', repeat($._specifier),
      choice($.type_specifier, $.struct_body_type, $.enum_body_type), repeat($.attribute_specifier),
      commaSep1($.declarator), ';'),
    struct_body_type: $ => seq(choice('struct', 'union'), repeat($.attribute_specifier),
      optional(field('name', $.identifier)), $.field_list),
    enum_body_type: $ => seq('enum', optional(field('name', $.identifier)), $.enumerator_list),
    struct_definition: $ => seq(repeat($._specifier), choice('struct', 'union'), repeat($.attribute_specifier),
      optional(field('name', $.identifier)), field('body', $.field_list), repeat($.attribute_specifier),
      optional(commaSep1($.declarator)), ';'),
    field_list: $ => seq('{', repeat(choice($.declaration, $.struct_definition)), '}'),
    enum_definition: $ => seq('enum', optional(field('name', $.identifier)), field('body', $.enumerator_list), ';'),
    enumerator_list: $ => seq('{', optional(seq(commaSep1($.enumerator), optional(','))), '}'),
    enumerator: $ => seq(field('name', $.identifier), optional(seq('=', $.expression))),

    if_statement: $ => prec.right(seq('if', '(', $._for_expression, ')', $._statement, optional(seq('else', $._statement)))),
    for_statement: $ => seq('for', '(', choice($.declaration, seq(optional($._for_expression), ';')),
      optional($._for_expression), ';', optional($._for_expression), ')', $._statement),
    _for_expression: $ => choice($.expression, $.comma_expression),
    while_statement: $ => seq('while', '(', $._for_expression, ')', $._statement),
    do_statement: $ => seq('do', $._statement, 'while', '(', $._for_expression, ')', ';'),
    switch_statement: $ => seq('switch', '(', $._for_expression, ')', $.block),
    case_label: $ => seq(choice(seq('case', $.expression), 'default'), ':'),
    return_statement: $ => seq('return', optional($._for_expression), ';'),
    jump_statement: $ => seq(choice('break', 'continue', seq('goto', $.identifier)), ';'),
    expression_statement: $ => seq($._for_expression, ';'),

    expression: $ => choice($.identifier, $.binding, $.number, $.string, $.char_literal, $.concatenated_string,
      $.parenthesized_expression, $.call_expression, $.index_expression, $.member_expression,
      $.unary_expression, $.binary_expression, $.assignment_expression, $.conditional_expression,
      $.cast_expression, $.sizeof_expression),
    // (float3)(1, 2, 3) is a cast of a parenthesized comma list: OpenCL's vector literal.
    parenthesized_expression: $ => seq('(', $._for_expression, ')'),
    comma_expression: $ => prec.right(seq($.expression, repeat1(seq(',', $.expression)))),
    initializer: $ => seq('{', optional(seq(commaSep1($.initializer_item), optional(','))), '}'),
    initializer_item: $ => choice(prec(PREC.assign + 1, $.expression), $.initializer, $.designated_initializer),
    designated_initializer: $ => seq(repeat1(choice(seq('.', field('field', $.identifier)), seq('[', $.expression, ']'))),
      '=', choice(prec(PREC.assign + 1, $.expression), $.initializer)),
    argument_list: $ => seq('(', optional(commaSep1(prec(PREC.assign + 1, $.expression))), ')'),
    call_expression: $ => prec(PREC.postfix, seq(field('function', $.expression), field('arguments', $.argument_list))),
    index_expression: $ => prec(PREC.postfix, seq($.expression, '[', $._for_expression, ']')),
    member_expression: $ => prec(PREC.postfix, seq($.expression, choice('.', '->'), field('member', $.identifier))),
    cast_expression: $ => prec(PREC.cast, seq('(', repeat($.qualifier), $.type_specifier, repeat($.pointer), ')',
      choice($.expression, $.initializer))),
    sizeof_expression: $ => prec.right(PREC.unary, seq('sizeof', choice(seq('(', $.type_specifier, repeat($.pointer), ')'), $.expression))),
    unary_expression: $ => choice(
      prec(PREC.unary, seq(choice('!', '~', '-', '+', '++', '--', '*', '&'), $.expression)),
      prec(PREC.postfix, seq($.expression, choice('++', '--')))),
    binary_expression: $ => choice(...[
      [PREC.multiply, ['*', '/', '%']], [PREC.add, ['+', '-']], [PREC.shift, ['<<', '>>']],
      [PREC.relational, ['<', '<=', '>', '>=']], [PREC.equal, ['==', '!=']], [PREC.bitand, ['&']],
      [PREC.xor, ['^']], [PREC.bitor, ['|']], [PREC.and, ['&&']], [PREC.or, ['||']]
    ].map(([p, ops]) => prec.left(p, seq($.expression, ops.length === 1 ? ops[0] : choice(...ops), $.expression)))),
    assignment_expression: $ => prec.right(PREC.assign, seq($.expression,
      choice('=', '+=', '-=', '*=', '/=', '%=', '&=', '|=', '^=', '<<=', '>>='), $.expression)),
    conditional_expression: $ => prec.right(PREC.conditional, seq($.expression, '?', $.expression, ':', $.expression)),

    // @P, @mass, @ix: a Houdini binding. Methods are ordinary member/call syntax: @P.set(x).
    binding: _ => token(seq('@', /[A-Za-z_][A-Za-z_0-9]*/)),
    identifier: _ => /[A-Za-z_][A-Za-z_0-9]*/,
    number: _ => token(choice(/0[xX][0-9a-fA-F]+[uUlL]*/,
      /([0-9]+(\.[0-9]*)?|\.[0-9]+)([eE][+-]?[0-9]+)?[fFhHuUlL]*/)),
    string: _ => token(/"([^"\\\n]|\\.)*"/),
    char_literal: _ => token(/'([^'\\\n]|\\.)+'/),
    concatenated_string: $ => prec.right(seq($.string, repeat1($.string))),
    comment: _ => token(choice(/\/\/([^\n\\]|\\[^\n]|\\\n)*/, /\/\*[^*]*\*+([^/*][^*]*\*+)*\//)),
    // Directives are single (possibly continued) lines. #bind and #include/#import get
    // their own nodes so the editor can highlight and navigate them.
    bind_directive: _ => token(prec(2, seq('#', /[ \t]*/, 'bind', /[ \t]([^\n\\]|\\[^\n]|\\\n)*/))),
    include_directive: _ => token(prec(2, seq('#', /[ \t]*/, choice('include', 'import'), /[ \t]*[<"][^\n]*/))),
    preprocessor: _ => token(prec(1, seq('#', /([^\n\\]|\\[^\n]|\\\n)*/))),
  }
});
