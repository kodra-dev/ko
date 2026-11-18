(function_definition body: (block) @function.inside) @function.around
(kernel_block body: (block) @function.inside) @function.around
(struct_definition body: (field_list) @class.inside) @class.around
(comment)+ @comment.around
