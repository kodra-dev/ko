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


def setParentTfo(rig: apex.Graph, child: int, parent: int):
    connect(rig, parent, "xform", child, "parent")
    connect(rig, parent, "localxform", child, "parentlocal")

def getParentTfo(rig: apex.Graph, child: int) -> int:
    return getSourceNode(rig, child, "parent")

def insertBetweenParentTfo(rig: apex.Graph, child: int, parent: int,
                           biases: (float, float, float) = (1, 1, 1)):
    """
    Insert parent between child and its old parent
    biases: 0 if the new parent is at the old parent, 1 if at child (traslation, rotation, scale)
    """
    old_parent = getParentTfo(rig, child)
    setParentTfo(rig, child, parent)
    setParentTfo(rig, parent, old_parent)

    restlocal = rig.getNodeParms(child)["restlocal"] or hou.Matrix4(1)
    updateParms(rig, parent, { "restlocal": kmath.lerpMatrix4(hou.Matrix4(1), restlocal, biases) })

def promoteTfo(rig: apex.Graph, tfo: int, t: bool = True, r: bool = True, s: bool = False):
    parms_node = getNode(rig, "%callback(__parms__)")
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


def safeAdd(rig: apex.Graph, name: str, apex_cb: str) -> int:
    if(rig.matchNodes(name)):
        raise Exception(f"Node with name {name} already exists.")
    return rig.addNode(name, apex_cb)

def connect(rig: apex.Graph, src_node: int, src_port_name: str, dst_node: int, dst_port_name: str):
    sp = getOutPort(rig, src_node, outPortName(src_port_name))
    dp = getInPort(rig, dst_node, inPortName(dst_port_name))
    rig.addWire(sp, dp)

def getSourceNode(rig: apex.Graph, dst_node: int, dst_port_name: str) -> int:
    port = getInPort(rig, dst_node, dst_port_name)
    srcPorts = rig.connectedPorts(port)
    if not srcPorts:
        raise Exception(f"Node {rig.nodeName(dst_node)}'s port {dst_port_name} has no source connected.")
    return rig.portNode(srcPorts[0])


def updateParms(rig: apex.Graph, node: int, parms: dict):
    old_parms = rig.getNodeParms(node)
    for k in parms:
        old_parms[k] = parms[k]
    rig.setNodeParms(node, old_parms)



JOINT_PREFIXES = ["CTL", "MCH", "DEF", "TGT"]
JOINT_SUFFIXES = ["L", "R"] 

def split_joint_name(name: str) -> (str, str, str):
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
    parts = split_joint_name(old_name)
    return joinJointName(prefix, parts[1], parts[2])


def jointSetSuffix(old_name: str, suffix: str) -> str:
    parts = split_joint_name(old_name)
    return joinJointName(parts[0], parts[1], suffix)


def mchJointName(comp_name: str, sub_prefix: str) -> str:
    parts = split_joint_name(comp_name)
    return joinJointName("MCH", sub_prefix + '_' + parts[1], parts[2])

def ctlJoinName(comp_name: str, sub_prefix: str) -> str:
    parts = split_joint_name(comp_name)
    return joinJointName("CTL", sub_prefix + '_' + parts[1], parts[2])