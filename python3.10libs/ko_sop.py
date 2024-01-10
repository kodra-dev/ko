import hou
import ko_soputils as sopu

def nodeParmsToDict(node, flatten_ramp=True):
    verb = hou.sopNodeTypeCategory().nodeVerb("attribfromparm")
    verb.setParms({"nodepath": node.path(), "flattenramp": flatten_ramp})
    temp_geo = hou.Geometry()
    temp_geo.execute(verb, [])
    return temp_geo.attribValue("parms")

def isPythonBased(node):
    definition = node.type().definition()
    return definition and 'PythonCook' in definition.sections().keys()


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

