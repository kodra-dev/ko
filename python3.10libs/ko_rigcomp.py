import hou
import apex
from kinefx import utils as ku
import ko_rigutils as ru

def setJointsRord(rig, skel, **kwargs):
    name_attrib = skel.findPointAttrib("name")
    rord_attrib = skel.findPointAttrib("rord")

    if not name_attrib:
        raise hou.NodeError("No name attribute")
    if not rord_attrib:
        raise hou.NodeError("No rord attribute")

    for pt in skel.points():
        name = pt.stringAttribValue(name_attrib)
        if name:
            joint = tfo(rig, name)
            parms = rig.getNodeParms(joint)
            parms['rord'] = pt.intAttribValue(rord_attrib)
            rig.setNodeParms(joint, parms)


def twoBoneIK(rig, skel, **kwargs):
    comp_name = kwargs['compname']
    root_name = kwargs['root']
    mid_name = kwargs['mid']

    tip_name = kwargs['tip']
    color = hou.Vector3(kwargs['nodecolor'])
    falloff = kwargs['falloff']
    stretch_axis = kwargs['stretchaxis']

    promote_target_r = kwargs['promotargetr']
    promote_target_s = kwargs['promotargets']

    root = ru.tfo(rig, root_name)
    mid = ru.tfo(rig, mid_name)
    tip = ru.tfo(rig, tip_name)
    root_joint = ku.findPointName(skel, root_name)
    mid_joint = ku.findPointName(skel, mid_name)
    tip_joint = ku.findPointName(skel, tip_name)
    root_joint_xform = ku.getPointTransform(root_joint)
    mid_joint_xform = ku.getPointTransform(mid_joint)
    tip_joint_xform = ku.getPointTransform(tip_joint)

    ctl_target_name = ru.ctlJoinName(comp_name, "IKTarget")
    ctl_target = ru.safeAdd(rig, ctl_target_name, "TransformObject")
    ru.updateParms(rig, ctl_target, { "restlocal": tip_joint_xform })
    rig.setNodeColor(ctl_target, color)
    ru.promoteTfo(rig, ctl_target, t=True, r=promote_target_r, s=promote_target_s)

    mch_root_name = ru.mchJointName(comp_name, "IKRoot")
    mch_root = ru.safeAdd(rig, mch_root_name, "TransformObject")
    ru.insertBetweenParentTfo(rig, root, mch_root) 
    rig.setNodeColor(mch_root, color)

    smooth_ik_name = f"SmoothIK_{comp_name}"
    smooth_ik = ru.safeAdd(rig, smooth_ik_name, "SmoothIk")
    rig.setNodeColor(smooth_ik, color)

    ru.connect(rig, mch_root, "xform", smooth_ik, "driver_root")
    ru.connect(rig, ctl_target, "xform", smooth_ik, "driver_target")

    ru.connect(rig, smooth_ik, "rootout", root, "xform")
    ru.connect(rig, smooth_ik, "midout", mid, "xform")
    ru.connect(rig, smooth_ik, "tipout", tip, "xform")

    ru.updateParms(rig, smooth_ik, {
        "falloff": falloff,
        "axis": stretch_axis,
        "rest_root": root_joint_xform,
        "rest_mid": mid_joint_xform,
        "rest_tip": tip_joint_xform,
    })