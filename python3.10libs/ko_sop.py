import hou
import ko_soputils as sopu
import ko_ui

def nodeParmsToDict(node, flatten_ramp=False):
    verb = hou.sopNodeTypeCategory().nodeVerb("attribfromparm")
    verb.setParms({"nodepath": node.path(), "flattenramp": flatten_ramp})
    temp_geo = hou.Geometry()
    temp_geo.execute(verb, [])
    return temp_geo.attribValue("parms")


def isPythonBased(node):
    definition = node.type().definition()
    return definition and 'PythonCook' in definition.sections().keys()


def groupSelectMenu(node, input_index = 0, type = None):
    # Get the currently selected group type
    if not type:
        parm = node.parm("grouptype")
        type = parm.evalAsString()

    geo = node.inputGeometry(input_index)

    # Select the groups to work with based
    # on the selected group type
    if node:
        if type == "points":
            groups = geo.pointGroups()
        elif type == "edges":
            groups = geo.edgeGroups()
        elif type == "prims":
            groups = geo.primGroups()
        elif type == "vertices":
            groups = geo.vertexGroups()
        else:
            return []
    return ko_ui.menuize([group.name() for group in groups])

def allDownstreamSops(node: hou.Node):
    visited = set()
    nodes = []
    def recurse(node: hou.Node, nodes: list[hou.Node]):
        if node in visited:
            return
        visited.add(node)
        for child in node.outputs():
            nodes.append(child)
            recurse(child, nodes)

    recurse(node, nodes)
    return nodes

def findDownstreamSops(node: hou.Node, type: hou.NodeType):
    return [n for n in allDownstreamSops(node) if n.type() == type]
    



## Sop/xform (Transform SOP)

def xform_getPreviousTransformPivot(pwd: hou.SopNode) -> hou.Vector3:
    attrib_name = pwd.parm("outputattrib").evalAsString()
    xord = pwd.evalParm("xOrd")
    # rord = pwd.evalParm("rOrd")
    xform = pwd.inputGeometry(0).attribValue(attrib_name)
    xform = hou.Matrix4(xform)
    return xform.extractTranslates(sopu.xordToString(xord))

def xform_getPreviousTransformPivotRot(pwd: hou.SopNode) -> hou.Vector3:
    attrib_name = pwd.parm("outputattrib").evalAsString()
    xord = pwd.evalParm("xOrd")
    rord = pwd.evalParm("rOrd")
    xform = pwd.inputGeometry(0).attribValue(attrib_name)
    xform = hou.Matrix4(xform)
    return xform.extractRotates(sopu.xordToString(xord), sopu.rordToString(rord))

