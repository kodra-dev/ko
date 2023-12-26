import hou

def nodeParmsToDict(node):
    verb = hou.sopNodeTypeCategory().nodeVerb("attribfromparm")
    verb.setParms({"nodepath": node.path()})
    temp_geo = hou.Geometry()
    temp_geo.execute(verb, [])
    return temp_geo.attribValue("parms")

def isPythonBased(node):
    definition = node.type().definition()
    return definition and 'PythonCook' in definition.sections().keys()