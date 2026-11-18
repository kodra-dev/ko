"""APEX Script names that are not in the generated function catalogue.

Taken from the APEX Script documentation (character/kinefx/apexscript*.txt) and
Houdini's transpiler (apex/graphscriptutils.py CONSTANT_DEFINITIONS and FUNC_* names).
"""

KEYWORDS = ['def', 'return', 'if', 'elif', 'else', 'for', 'in', 'and', 'or', 'not', 'is',
            'pass', 'raise', 'True', 'False']

# name: (signature, description, return type or '' when it depends on arguments)
SPECIAL_FUNCTIONS = {
    'BindInput': ('BindInput(*defaults, **named_defaults)',
                  'Defines graph inputs. Each positional argument is the default value (and type) of one input, '
                  'for example `f = BindInput(3.1)` or `x, y = BindInput(2.6, "test")`. Annotate to declare a type '
                  'without a default: `s: String = BindInput()`. Keyword arguments name the input ports; '
                  '`__dot__` in a name becomes `.` (`Base__dot__skel` is `Base.skel`).', ''),
    'BindOutput': ('BindOutput(*values, **named_values)',
                   'Defines graph outputs. Keyword arguments name the output ports, for example '
                   '`BindOutput(y, int_test=i)`.', ''),
    'HoudiniVersion': ('HoudiniVersion(version: String)',
                       'Sets the version of the APEX functions to use, for example `HoudiniVersion("20.5.250")`. '
                       '`"newest"` uses the newest versions; `"ignore"` (the default when absent) uses the exact '
                       'version called, so `IntBitMaskToBool.v2_0()` selects version 2.', ''),
    'BeginMetadata': ('BeginMetadata(__name=, __color=, __pos=, __properties=, __tags=)',
                      'Sets metadata on the begin node of the enclosing for loop or if block.', ''),
    'EndMetadata': ('EndMetadata(__name=, __color=, __pos=, __properties=, __tags=)',
                    'Sets metadata on the end node of the enclosing for loop or if block.', ''),
    'InputMetadata': ('InputMetadata(__name=, __color=, __pos=, __properties=, __tags=)',
                      'Sets metadata on the input (parms) node inside the enclosing function subnet.', ''),
    'OutputMetadata': ('OutputMetadata(__name=, __color=, __pos=, __properties=, __tags=)',
                       'Sets metadata on the output node inside the enclosing function subnet.', ''),
    'InvokeOutputs': ('InvokeOutputs(function, parms: Dict, outputs: StringArray)',
                      'Executes another graph or function. `parms` passes parameter values; each `outputs` entry is '
                      '`"<output_node>:<output_port>"` (or `"output:result<N>"` for the Nth returned expression). '
                      'Annotate the receiving variables, since the result types are not known.', ''),
    'SetGraph': ('SetGraph(nodeid: ApexNodeID, graph: ApexGraphHandle)',
                 'Associates an APEX node ID with a graph. Also callable as `nodeid.setGraph(graph)`.', ''),
    'StickyNote': ('StickyNote(contents: String, dimensions: Vector2, is_background_hidden: Bool, '
                   'is_collapsed: Bool, text_color: Vector3, text_size: Float, __name=, __color=, __pos=)',
                   'Creates a sticky note in the graph.', ''),
    '_portalias': ('_portalias(value)',
                   'Creates another identifier for a variable without making a copy. Can introduce graph sorting '
                   'issues; use with caution.', ''),
    'print': ('print(*values)', 'Logs a message (a Log node). Supports f-strings. Messages appear in the log '
              'viewer with APEX enabled, and repeat only when the graph re-runs.', ''),
    'warning': ('warning(message: String)', 'Logs a warning (a Log node).', ''),
    'error': ('error(message: String)', 'Logs an error (a Log node). Use as `raise error("...")`.', ''),
    'len': ('len(value: String | Array | Dict) -> Int',
            'Number of characters in a string, items in an array, or entries in a dictionary.', 'Int'),
    'range': ('range(count: Int)', 'Iterates a for loop `count` times: `for i in range(3):`.', ''),
    'enumerate': ('enumerate(array)', 'Iterates an array with its index: `for i, value in enumerate(array):`.', ''),
    'addToFolder': ('addToFolder(folder_name: String, inputs, foldertype=, parent=)',
                    'Organizes bound inputs (variables declared with `BindInput()`) into a parameter folder.', ''),
    'bindMultiparm': ('bindMultiparm()', 'Creates a new multiparm for the graph interface.', ''),
    'addToMultiparm': ('addToMultiparm(multiparm, name: String, default, preset=, preset_kwargs=)',
                       'Adds an element to a multiparm created with `bindMultiparm()`.', ''),
}

DECORATORS = {
    'subgraph': ('@subgraph([file_path], name=, hidden=, ...)',
                 'Saves the function as a packed subgraph (with **Save Subgraphs** on the APEX Script SOP). '
                 'Saved subgraphs can then be called like global functions. Extra keyword arguments are stored '
                 'in the graph `properties`.'),
    'namespace': ("@namespace('my_namespace')",
                  'Places a saved subgraph function in a namespace: call it as `my_namespace.test(...)`.'),
    'safeguard_inputs': ('@safeguard_inputs(True|False)',
                         'When True (the default), copies function inputs that feed in-place ports, to prevent '
                         'graph sorting issues.'),
}

# name: (type, value) from graphscriptutils.CONSTANT_DEFINITIONS
CONSTANTS = {
    'M_E': ('Float', '2.718281828459045'),
    'M_PI': ('Float', '3.141592653589793'),
    'M_SQRT2': ('Float', '1.4142135623730951'),
    'RED': ('Vector3', '(1, 0, 0)'),
    'GREEN': ('Vector3', '(0, 1, 0)'),
    'BLUE': ('Vector3', '(0, 0, 1)'),
    'WHITE': ('Vector3', '(1, 1, 1)'),
    'BLACK': ('Vector3', '(0, 0, 0)'),
}

# Object-oriented built-ins that Houdini's generated apexscript.py omits, because the
# mapped callback's first input is the graph, not the node/port (apexscript/builtins.py
# and "Function mappings" in apexscriptfunctions.txt). type: {name: (params, returns, maps_to)}
OBJECT_METHODS = {
    'ApexNodeID': {
        'ancestors': ('', '', 'graphutils::NodeAncestors'),
        'callbackName': ('', 'String', 'graphutils::NodeCallbackName'),
        'data': ('', '', 'graph::NodeData; returns (name, callback, pos, color, parms, tags, properties, output)'),
        'inPorts': ('', 'ApexPortIDArray', 'graph::NodeInputs'),
        'name': ('', 'String', 'graphutils::NodeName'),
        'outPorts': ('', 'ApexPortIDArray', 'graph::NodeOutputs'),
        'parent': ('', '', 'graphutils::NodeParent'),
        'parms': ('', 'Dict', 'graphutils::NodeParms'),
        'path': ('', 'String', 'graphutils::NodePath'),
        'port': ('portname: String', 'ApexPortID', 'graph::FindPort'),
        'properties': ('', 'Dict', 'graphutils::NodeProperties'),
        'setColor': ('color: Vector3', '', 'graph::UpdateNode'),
        'setName': ('name: String', '', 'graph::UpdateNode'),
        'setParms': ('parms: Dict', '', 'graph::UpdateNodeParms'),
        'setPos': ('pos: Vector3', '', 'graph::UpdateNode'),
        'setProperties': ('properties: Dict', '', 'graph::UpdateNodeProperties'),
        'setTags': ('tags: StringArray', '', 'graph::UpdateNodeTags'),
        'tags': ('', 'StringArray', 'graphutils::NodeTags'),
        'setGraph': ('graph: ApexGraphHandle', '', 'SetGraph (special function)'),
    },
    'ApexPortID': {
        'connect': ('dstport: ApexPortID', '', 'graph::ConnectInput'),
        'connectPath': ('dstport: ApexPortID, name: String', '', 'graph::AddWirePath'),
        'connected': ('', 'ApexPortIDArray', 'graph::GetConnectedPorts'),
        'data': ('', '', 'graph::PortData; returns (name, alias, datatype, porttype, isconnected, ispromoted)'),
        'disconnect': ('', '', 'graph::DisconnectPort'),
        'node': ('', 'ApexNodeID', 'graph::PortNode'),
        'promote': ('name: String', '', 'graph::PromotePort'),
        'promoted': ('', '', 'graph::GetPromotedPort'),
        'promoteInput': ('parmname: String, parmnodeid: ApexNodeID', '', 'graph::PromoteInput'),
        'promoteOutput': ('outputname: String, outputnodeid: ApexNodeID', '', 'graph::PromoteOutput'),
        'subport': ('portname: String', 'ApexPortID', 'graph::GetSubPort'),
    },
}

# Keyword arguments accepted by every function call (node metadata), plus templated types.
SPECIAL_ARGUMENTS = {
    '__name': 'String: the name of the node.',
    '__color': 'Vector3: the color of the node.',
    '__pos': 'Vector3: the position of the node in the graph.',
    '__properties': 'Dict: the properties stored on the node.',
    '__tags': 'StringArray: the tags stored on the node.',
    'valuetype': 'The type of a templated function, e.g. `valuetype=String`, or a tuple/list of types.',
}
