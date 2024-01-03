import hou
import apex
import ko_math as kmath

def getNode(rig: apex.Graph, pattern: str, must_exist: bool = True) -> int | None:
    nodes = rig.matchNodes(pattern)
    if not nodes:
        if must_exist:
            raise Exception(f"Pattern {pattern} doesn't match any nodes.")
        return None
    if len(nodes) > 1:
        raise Exception(f"Pattern {pattern} matches multiple nodes.")
    return nodes[0]

def tfo(rig: apex.Graph, name: str) -> int:
    return getNode(rig, f"%callback(TransformObject) & {name}")

def ac(rig: apex.Graph, name: str) -> int:
    return getNode(rig, f"%callback(AbstractControl) & {name}")

def tfoRestLocal(rig: apex.Graph, tfo: int) -> hou.Matrix4:
    return rig.getNodeParms(tfo)["restlocal"] or hou.Matrix4(1)

def tfoRestTransform(rig: apex.Graph, tfo: int) -> hou.Matrix4:
    xform = tfoRestLocal(rig, tfo)
    parent = getParentTfo(rig, tfo, must_exist=False)
    while parent:
        parent_xform = rig.getNodeParms(parent)["restlocal"] or hou.Matrix4(1)
        xform = xform * parent_xform
        parent = getParentTfo(rig, parent, must_exist=False)
    return xform

def axesToRord(primary_axis: int, secondary_axis: int) -> int:
    """
    primary, tertiary, secondary
    xyz = 0
    xzy = 1
    yxz = 2
    yzx = 3
    zxy = 4
    zyx = 5
    """
    if primary_axis == 0:
        if secondary_axis == 2:
            return 0
        if secondary_axis == 1:
            return 1
    elif primary_axis == 1:
        if secondary_axis == 2:
            return 2
        if secondary_axis == 0:
            return 3
    elif primary_axis == 2:
        if secondary_axis == 0:
            return 4
        if secondary_axis == 1:
            return 5
    raise Exception(f"Invalid axes {primary_axis}, {secondary_axis}")


def setParentTfo(rig: apex.Graph, child: int, parent: int, compensate_xform: bool = False):
    if compensate_xform:
        child_xform = tfoRestTransform(rig, child)
        parent_xform = tfoRestTransform(rig, parent)
        child_xform = child_xform * parent_xform.inverted()
        updateParms(rig, child, { "restlocal": child_xform })
    connect(rig, parent, "xform", child, "parent")
    connect(rig, parent, "localxform", child, "parentlocal")


def getParentTfo(rig: apex.Graph, child: int, must_exist: bool = True) -> int | None:
    return getSourceNode(rig, child, "parent", must_exist=must_exist)

def insertBetweenParentTfo(rig: apex.Graph, child: int, parent: int,
                           biases: (float, float, float) = (1, 1, 1)):
    """
    Insert parent between child and its old parent
    biases: 0 if the new parent is at the old parent, 1 if at child (traslation, rotation, scale)
    """
    old_parent = getParentTfo(rig, child)
    setParentTfo(rig, parent, old_parent)

    restlocal = rig.getNodeParms(child)["restlocal"] or hou.Matrix4(1)
    updateParms(rig, parent, { "restlocal": kmath.lerpMatrix4(hou.Matrix4(1), restlocal, biases) })
    setParentTfo(rig, child, parent, compensate_xform=True)
    

def promoteTfo(rig: apex.Graph, tfo: int, t: bool = True, r: bool = True, s: bool = False, demote: bool = False):
    parms_node = getNode(rig, "%callback(__parms__)")

    if demote:
        tp = getInPort(rig, tfo, "t")
        wires = rig.portWires(tp)
        rig.removeWires(wires, True)
        rp = getInPort(rig, tfo, "r")
        wires = rig.portWires(rp)
        rig.removeWires(wires, True)
        sp = getInPort(rig, tfo, "s")
        wires = rig.portWires(sp)
        rig.removeWires(wires, True)

    name = rig.nodeName(tfo)
    if t:
        rig.promoteInput(getInPort(rig, tfo, "t"), parms_node, f"{name}_t")
    if r:
        rig.promoteInput(getInPort(rig, tfo, "r"), parms_node, f"{name}_r")
    if s:
        rig.promoteInput(getInPort(rig, tfo, "s"), parms_node, f"{name}_s")




def inPortName(name: str) -> str:
    return name if name.endswith("[in]") else name + "[in]"

def outPortName(name: str) -> str:
    return name if name.endswith("[out]") else name + "[out]"
    
def getInPort(rig: apex.Graph, node: int, name: str) -> int:
    return rig.getPort(node, inPortName(name))

def getOutPort(rig: apex.Graph, node: int, name: str) -> int:
    return rig.getPort(node, outPortName(name))

def addNode(rig: apex.Graph, name: str, apex_cb: str, node_storage: set[int] | None = None) -> int:
    n = rig.addNode(name, apex_cb)
    if node_storage != None:
        node_storage.add(n)
    return n

def safeAdd(rig: apex.Graph, name: str, apex_cb: str, node_storage: set[int] | None = None) -> int:
    if(rig.matchNodes(name)):
        raise Exception(f"Node with name {name} already exists.")
    n = rig.addNode(name, apex_cb)
    if node_storage != None:
        node_storage.add(n)
    return n

def connect(rig: apex.Graph, src_node: int, src_port_name: str, dst_node: int, dst_port_name: str):
    sp = getOutPort(rig, src_node, outPortName(src_port_name))
    dp = getInPort(rig, dst_node, inPortName(dst_port_name))
    rig.addWire(sp, dp)

def getSourceNode(rig: apex.Graph, dst_node: int, dst_port_name: str, must_exist: bool = True) -> int | None:
    port = getInPort(rig, dst_node, dst_port_name)
    srcPorts = rig.connectedPorts(port)
    if not srcPorts:
        if must_exist:
            raise Exception(f"Node {rig.nodeName(dst_node)}'s port {dst_port_name} has no source connected.")
        return None
    return rig.portNode(srcPorts[0])


def updateParms(rig: apex.Graph, node: int, parms: dict):
    old_parms = rig.getNodeParms(node)
    for k in parms:
        old_parms[k] = parms[k]
    rig.setNodeParms(node, old_parms)

def xformmask(t: bool = True, r: bool = True, s: bool = True) -> int:
    return (1 if t else 0) | (2 if r else 0) | (4 if s else 0)

def setNodesColor(rig: apex.Graph, nodes: set[int], color: hou.Color):
    for n in nodes:
        rig.setNodeColor(n, color)


JOINT_PREFIXES = ["CTL", "MCH", "DEF", "TGT"]
JOINT_SUFFIXES = ["L", "R"] 

def splitJointName(name: str) -> (str, str, str):
    prefix = main = suffix = None
    parts = name.split('_')
    if parts[0] in JOINT_PREFIXES:
        prefix = parts[0]
        parts.pop(0)
    if parts[-1] in JOINT_SUFFIXES:
        suffix = parts[-1]
        parts.pop(-1)
    main = "_".join(parts)

    return (prefix, main, suffix)

        
def joinJointName(prefix: str, main: str, suffix: str) -> str:
    parts = list(p for p in [prefix, main, suffix] if p)
    return "_".join(parts)


def jointSetPrefix(old_name: str, prefix: str) -> str:
    parts = splitJointName(old_name)
    return joinJointName(prefix, parts[1], parts[2])


def jointSetSuffix(old_name: str, suffix: str) -> str:
    parts = splitJointName(old_name)
    return joinJointName(parts[0], parts[1], suffix)


def mchJointName(comp_name: str, sub_prefix: str) -> str:
    parts = splitJointName(comp_name)
    return joinJointName("MCH", sub_prefix + '_' + parts[1], parts[2])

def ctlJoinName(comp_name: str, sub_prefix: str) -> str:
    parts = splitJointName(comp_name)
    return joinJointName("CTL", sub_prefix + '_' + parts[1], parts[2])