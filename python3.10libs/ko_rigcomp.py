import hou
import apex
from kinefx import utils as ku
import ko_rigutils as ru
import ko_math as kmath
from hou import hmath as hm
import math

def setJointsRord(rig, skel, **kwargs):
    joint_group = kwargs['jointgroup']

    name_attrib = skel.findPointAttrib("name")
    rord_attrib = skel.findPointAttrib("rord")

    if not name_attrib:
        raise hou.NodeError("No name attribute")
    if not rord_attrib:
        raise hou.NodeError("No rord attribute")

    points = skel.globPoints(joint_group) if joint_group else skel.points()
    for pt in points:
        name = pt.stringAttribValue(name_attrib)
        if name:
            joint = ru.tfo(rig, name, must_exist=False)
            if joint != -1:
                parms = rig.getNodeParms(joint)
                parms['rord'] = pt.intAttribValue(rord_attrib)
                rig.setNodeParms(joint, parms)


def twoBoneIK(rig, skel, **kwargs):
    comp_name = kwargs['compname']
    color = kwargs['nodecolor']

    ctl_parent_name = kwargs['ctlparent']
    root_name = kwargs['root']
    mid_name = kwargs['mid']
    tip_name = kwargs['tip']
    ctl_target_name = kwargs['ctltargetname']
    tip_follow_control = kwargs['tipfollowcontrol']
    # falloff = kwargs['falloff']
    # stretch_axis = kwargs['stretchaxis']

    promote_target_r = kwargs['promotargetr']
    promote_target_s = kwargs['promotargets']

    pole_length = kwargs['polelength']
    pole_style = kwargs['polestyle']
    
    new_nodes = set()

    root = ru.tfo(rig, root_name)
    mid = ru.tfo(rig, mid_name)
    tip = ru.tfo(rig, tip_name)
    root_joint = ku.findPointName(skel, root_name)
    mid_joint = ku.findPointName(skel, mid_name)
    tip_joint = ku.findPointName(skel, tip_name)
    root_joint_xform = ku.getPointTransform(root_joint)
    mid_joint_xform = ku.getPointTransform(mid_joint)
    tip_joint_xform = ku.getPointTransform(tip_joint)

    # mch_root: the root of the IK chain. e.g. hip
    mch_root_name = ru.mchJointName(comp_name, "IKRoot")
    mch_root = ru.safeAdd(rig, mch_root_name, "TransformObject", new_nodes)
    ru.insertBetweenParentTfo(rig, root, mch_root) 
    
    # ctl_target: the control of target joint. e.g. foot
    ctl_target = ru.tfo(rig, ctl_target_name)
    # ctl_target = ru.safeAdd(rig, ctl_target_name, "TransformObject")
    # ru.updateParms(rig, ctl_target, { "restlocal": tip_joint_xform })
    ru.promoteTfo(rig, ctl_target, t=True, r=promote_target_r, s=promote_target_s, demote=True)
    if ctl_parent_name:
        ctl_parent = ru.tfo(rig, ctl_parent_name)
        ru.setParentTfo(rig, ctl_target, ctl_parent, compensate_xform=True)


    root_joint_xlate = root_joint_xform.extractTranslates()
    tip_joint_xlate = tip_joint_xform.extractTranslates()
    mid_joint_xlate = mid_joint_xform.extractTranslates()
    projected_mid = mid_joint_xlate.pointOnSegment(root_joint_xlate, tip_joint_xlate)

    ctl_ik_pole = None
    to_twist_xform = None

    if pole_style == "free":
        # Calculate ctl_ik_pole's position: project mid to the line root->tip,
        # then extend the line from projected_mid to mid by the length pole_length

        # ctl_ik_pole: the control of IK pole. e.g. knee's direction
        ctl_ik_pole_name = ru.ctlJointName(comp_name, "IKPole")
        ctl_ik_pole = ru.safeAdd(rig, ctl_ik_pole_name, "TransformObject", new_nodes)

        pole_position = mid_joint_xlate + (mid_joint_xlate - projected_mid).normalized() * pole_length
        pole_xform = hou.hmath.buildTranslate(pole_position)

        ru.updateParms(rig, ctl_ik_pole, { "restlocal": pole_xform })
        ru.promoteTfo(rig, ctl_ik_pole, t=True, r=False, s=False)

        to_twist_xform = ru.getOutPort(rig, ctl_ik_pole, "xform")

        if ctl_parent_name:
            ctl_parent = ru.tfo(rig, ctl_parent_name)
            ru.setParentTfo(rig, ctl_ik_pole, ctl_parent, compensate_xform=True)
    elif pole_style == "revolve":
        # In this case, pole is initially at the projected_mid, and will stay on the line root->tip dynamically
        # Its restlocal's orientation matters, but not its translation
        # ctl_ik_pole: the control of IK pole. e.g. knee's direction

        ctl_ik_pole_name = ru.ctlJointName(comp_name, "IKPolePivot")
        ctl_ik_pole = ru.safeAdd(rig, ctl_ik_pole_name, "TransformObject", new_nodes)

        ru.setParentTfo(rig, ctl_ik_pole, mch_root)

        pole_lookat_pos = mid_joint_xlate + (mid_joint_xlate - projected_mid).normalized() * 1 # pole_length doesn't matter here
        pole_up_pos = root_joint_xlate
        pole_xform = kmath.lookat(pole_lookat_pos-projected_mid, pole_up_pos-projected_mid, projected_mid)
        pole_xform_local = pole_xform * root_joint_xform.inverted()
        ru.updateParms(rig, ctl_ik_pole, { "restlocal": pole_xform_local })
        ru.promoteTfo(rig, ctl_ik_pole, t=False, r=True, s=False)

        op_mul = ru.addNode(rig, "mul", "Multiply<Vector3,Matrix4>", new_nodes)
        ru.updateParms(rig, op_mul, { "a": hou.Vector3(0, 0, 1) })
        ru.connect(rig, ctl_ik_pole, "xform", op_mul, "b")
        op_build = ru.addNode(rig, "build", "transform::Build", new_nodes)
        ru.connect(rig, op_mul, "result", op_build, "t")
        to_twist_xform = ru.getOutPort(rig, op_build, "m")

        mid_ratio = (projected_mid - root_joint_xlate).length() / (tip_joint_xlate - root_joint_xlate).length()
        op_slerp = ru.addNode(rig, "slerp", "transform::Slerp<Matrix4>", new_nodes)
        ru.connect(rig, mch_root, "xform", op_slerp, "a")
        ru.connect(rig, ctl_target, "xform", op_slerp, "b")
        ru.updateParms(rig, op_slerp, { "bias": mid_ratio })
        ru.connect(rig, op_slerp, "result", ctl_ik_pole, "xform")
        ru.updateParms(rig, ctl_ik_pole, { "xformmask": ru.xformmask(t=True, r=False, s=False) })


    smooth_ik_name = f"SmoothIK_{comp_name}"
    smooth_ik = ru.safeAdd(rig, smooth_ik_name, "rig::TwoBoneIK")
    ru.connect(rig, mch_root, "xform", smooth_ik, "rootdriver")
    ru.connect(rig, ctl_target, "xform", smooth_ik, "goal")
    rig.addWire(to_twist_xform, ru.getInPort(rig, smooth_ik, "twist"))

    if tip_follow_control:
        # ru.setParentTfo(rig, tip, ctl_target, compensate_xform=True)
        ru.connect(rig, ctl_target, "xform", tip, "xform")

    ru.connect(rig, smooth_ik, "rootout", root, "xform")
    ru.connect(rig, smooth_ik, "midout", mid, "xform")

    ru.updateParms(rig, smooth_ik, {
        "root": root_joint_xform,
        "mid": mid_joint_xform,
        "tip": tip_joint_xform,
        "blend": 1.0,
        "stretch": 1,
    })

    ru.setNodesColor(rig, new_nodes, color)


def ikFKSwitch(rig: apex.Graph, skel: hou.Geometry, **kwargs):
    compname = kwargs['compname']
    color = kwargs['nodecolor']

    switch_control_name = kwargs['switchcontrol']
    root_name = kwargs['root']
    mid_name = kwargs['mid']
    tip_name = kwargs['tip']
    root_fk_name = kwargs['rootfkcontrol']
    mid_fk_name = kwargs['midfkcontrol']
    tip_fk_name = kwargs['tipfkcontrol']

    switch_control = ru.ac(rig, switch_control_name)
    root = ru.tfo(rig, root_name)
    mid = ru.tfo(rig, mid_name)
    tip = ru.tfo(rig, tip_name)
    root_fk = ru.tfo(rig, root_fk_name)
    mid_fk = ru.tfo(rig, mid_fk_name)
    tip_fk = ru.tfo(rig, tip_fk_name)

    new_nodes = set()

    root_blend = ru.safeAdd(rig, f"{compname}_RootBlend", "transform::Slerp<Matrix4>", new_nodes)
    root_ik_xform_port = ru.getSourcePort(rig, root, "xform")
    rig.addWire(root_ik_xform_port, ru.getInPort(rig, root_blend, "a"))
    ru.connect(rig, root_fk, "xform", root_blend, "b")
    ru.connect(rig, switch_control, "x", root_blend, "bias")
    ru.connect(rig, root_blend, "result", root, "xform")

    mid_blend = ru.safeAdd(rig, f"{compname}_MidBlend", "transform::Slerp<Matrix4>", new_nodes)
    mid_ik_xform_port = ru.getSourcePort(rig, mid, "xform")
    rig.addWire(mid_ik_xform_port, ru.getInPort(rig, mid_blend, "a"))
    ru.connect(rig, mid_fk, "xform", mid_blend, "b")
    ru.connect(rig, switch_control, "x", mid_blend, "bias")
    ru.connect(rig, mid_blend, "result", mid, "xform")

    tip_blend = ru.safeAdd(rig, f"{compname}_TipBlend", "transform::Slerp<Matrix4>")
    tip_ik_xform_port = ru.getSourcePort(rig, tip, "xform")
    rig.addWire(tip_ik_xform_port, ru.getInPort(rig, tip_blend, "a"))
    ru.connect(rig, tip_fk, "xform", tip_blend, "b")
    ru.connect(rig, switch_control, "x", tip_blend, "bias")
    ru.connect(rig, tip_blend, "result", tip, "xform")

    ru.setNodesColor(rig, new_nodes, color)


def reverseFoot(rig: apex.Graph, skel: hou.Geometry, **kwargs):
    compname = kwargs['compname']
    color = kwargs['nodecolor']

    ctl_target_name = kwargs['existingiktarget']
    ctl_heel_name = kwargs['heel']
    ctl_foot_tip_name = kwargs['foottip']
    ctl_ball_name = kwargs['ball']

    use_ik_fk_switch = kwargs['useikfkswitch']
    switch_control_name = kwargs['switchcontrol']

    new_nodes = set()

    ctl_target = ru.tfo(rig, ctl_target_name)
    ctl_heel = ru.tfo(rig, ctl_heel_name)
    ctl_foot_tip = ru.tfo(rig, ctl_foot_tip_name)
    ctl_ball = ru.tfo(rig, ctl_ball_name)
    new_nodes.update([ctl_target, ctl_heel, ctl_foot_tip, ctl_ball])

    (prefix, main, suffix) = ru.splitJointName(ctl_ball_name)
    ctl_ball_rev_name = ru.joinJointName(prefix, f"{main}Rev", suffix)
    ctl_ball_rev = ru.duplicateNode(rig, ctl_ball, ctl_ball_rev_name, node_storage=new_nodes)

    # ctl_target is now controlled by ctl, so we rename it to a new MCH name
    (prefix, main, suffix) = ru.splitJointName(ctl_target_name)
    mch_target_name = ru.joinJointName("MCH", f"IKTarget_{main}", suffix)
    mch_target = ctl_target
    rig.setNodeName(mch_target, mch_target_name)
    ctl = ru.safeAdd(rig, ctl_target_name, "TransformObject", new_nodes)
    ru.insertBetweenParentTfo(rig, mch_target, ctl)

    if use_ik_fk_switch:
        switch_control = ru.ac(rig, switch_control_name)
        (_, main, suffix) = ru.splitJointName(ctl_ball_name)
        mch_blend_name = ru.joinJointName("MCH", f"IKFKBlend_{main}", suffix)
        mch_blend = ru.safeAdd(rig, mch_blend_name, "TransformObject", new_nodes)
        fk_ball = ru.addNode(rig, f"FK_{main}", "rig::FkTransform", new_nodes)
        ru.insertBetweenParentTfo(rig, ctl_ball, mch_blend)
        ru.insertBetweenParentTfo(rig, mch_blend, fk_ball)
        ball_blend = ru.safeAdd(rig, f"{compname}_BallBlend", "Lerp<Matrix4>", new_nodes)
        ru.connect(rig, ctl_ball_rev, "xform", ball_blend, "a")
        ru.connect(rig, fk_ball, "xform", ball_blend, "b")
        ru.connect(rig, ball_blend, "result", mch_blend, "xform")
        ru.connect(rig, switch_control, "x", ball_blend, "bias")

    ru.setParentTfo(rig, ctl_heel, ctl, compensate_xform=True)
    ru.setParentTfo(rig, ctl_foot_tip, ctl_heel, compensate_xform=True)
    ru.setParentTfo(rig, ctl_ball_rev, ctl_foot_tip, compensate_xform=True)
    if use_ik_fk_switch:
        ru.setParentTfo(rig, ctl_ball, mch_blend, compensate_xform=True)
    else:
        ru.setParentTfo(rig, ctl_ball, ctl_ball_rev, compensate_xform=True)
    ru.setParentTfo(rig, mch_target, ctl_ball_rev, compensate_xform=True)

    ru.promoteTfo(rig, ctl, t=True, r=True, s=False, demote=True)
    ru.promoteTfo(rig, mch_target, t=False, r=False, s=False, demote=True)
    ru.promoteTfo(rig, ctl_heel, t=False, r=True, s=False, demote=True)
    ru.promoteTfo(rig, ctl_foot_tip, t=False, r=True, s=False, demote=True)
    ru.promoteTfo(rig, ctl_ball_rev, t=False, r=True, s=False, demote=True)
    ru.promoteTfo(rig, ctl_ball, t=False, r=True, s=False, demote=True)

    # The "reverse foot" behavior: ctl_ball's restlocal multiplies the inverse of ctl_ball_rev_inherited -> ctl_ball_rev_current
    # new_ctl_ball_restlocal = ctl_ball_restlocal * ctl_ball_rev_inherited * inverted(ctl_ball_rev_current)
    # So ctl_ball doesn't move when ctl_ball_rev moves
    ctl_ball_rev_parent = ctl_foot_tip
    op_restlocal = ru.addNode(rig, f"{ctl_ball_rev_name}_restlocal", "Value<Matrix4>", new_nodes)
    ru.updateParms(rig, op_restlocal, { "parm": ru.tfoRestLocal(rig, ctl_ball_rev) })
    op_inherited = ru.addNode(rig, f"{ctl_ball_rev_name}_inherited", "Multiply<Matrix4>", new_nodes)
    ru.connect(rig, op_restlocal, "value", op_inherited, "a")
    ru.connect(rig, ctl_ball_rev_parent, "xform", op_inherited, "b")
    op_inv = ru.addNode(rig, f"{ctl_ball_rev_name}_inv", "Invert<Matrix4>", new_nodes)
    ru.connect(rig, ctl_ball_rev, "xform", op_inv, "a")
    op_mul = ru.addNode(rig, f"mul", "Multiply<Matrix4>", new_nodes)
    ru.connect(rig, op_inherited, "result", op_mul, "a")
    ru.connect(rig, op_inv, "result", op_mul, "b")
    op_restlocal2 = ru.addNode(rig, f"{ctl_ball_name}_restlocal", "Value<Matrix4>", new_nodes)
    ru.updateParms(rig, op_restlocal2, { "parm": ru.tfoRestLocal(rig, ctl_ball) })
    op_mul2 = ru.addNode(rig, f"mul2", "Multiply<Matrix4>", new_nodes)
    ru.connect(rig, op_restlocal2, "value", op_mul2, "a")
    if use_ik_fk_switch:
        switch_control = ru.ac(rig, switch_control_name)
        op_lerp = ru.addNode(rig, f"lerp", "Lerp<Matrix4>", new_nodes)
        ru.updateParms(rig, op_lerp, { "b": hou.Matrix4() })
        ru.connect(rig, op_mul, "result", op_lerp, "a")
        ru.connect(rig, switch_control, "x", op_lerp, "bias")
        ru.connect(rig, op_lerp, "result", op_mul2, "b")
    else:
        ru.connect(rig, op_mul, "result", op_mul2, "b")
    ru.connect(rig, op_mul2, "result", ctl_ball, "restlocal")

    ru.setNodesColor(rig, new_nodes, color)


def trackTo(rig, skel, **kwargs):
    comp_name = kwargs['compname']
    color = kwargs['nodecolor']

    driven_name = kwargs['driven']
    target_name = kwargs['target']
    mirroed = kwargs['mirrored']

    driven = ru.tfo(rig, driven_name)
    target = ru.tfo(rig, target_name)

    new_nodes = set()

    mch_ref_name = ru.mchJointName(comp_name, "TrackFrom")
    mch_ref = ru.safeAdd(rig, mch_ref_name, "TransformObject", new_nodes)
    ru.insertBetweenParentTfo(rig, driven, mch_ref)

    op_lookat = ru.addNode(rig, "lookat", "transform::LookAt", new_nodes)
    ru.connect(rig, mch_ref, "xform", op_lookat, "inxform")

    op_up = ru.addNode(rig, "up", "Value<Vector3>", new_nodes)
    ru.updateParms(rig, op_up, { "parm": hou.Vector3(0, 1, 0) })
    op_mul = ru.addNode(rig, "mul", "Multiply<Vector3,Matrix4>", new_nodes) 
    ru.connect(rig, op_up, "value", op_mul, "a")
    ru.connect(rig, mch_ref, "xform", op_mul, "b")
    op_build = ru.addNode(rig, "build", "transform::Build", new_nodes)
    ru.connect(rig, op_mul, "result", op_build, "t")

    ru.connect(rig, op_build, "m", op_lookat, "lookup")

    if mirroed:
        op_explode1 = ru.addNode(rig, "explode1", "transform::Explode", new_nodes)
        ru.connect(rig, target, "xform", op_explode1, "m")

        op_explode2 = ru.addNode(rig, "explode2", "transform::Explode", new_nodes)
        ru.connect(rig, mch_ref, "xform", op_explode2, "m")
        op_val = ru.addNode(rig, "val", "Value<Float>", new_nodes)
        ru.updateParms(rig, op_val, { "parm": 2.0 })
        op_mul2 = ru.addNode(rig, "mul2", "Multiply<Vector3,Float>", new_nodes)
        ru.connect(rig, op_explode2, "t", op_mul2, "a")
        ru.connect(rig, op_val, "value", op_mul2, "b")

        op_sub = ru.addNode(rig, "sub", "Subtract<Vector3>", new_nodes)
        ru.connect(rig, op_mul2, "result", op_sub, "a")
        ru.connect(rig, op_explode1, "t", op_sub, "b")

        op_build2 = ru.addNode(rig, "build2", "transform::Build", new_nodes)
        ru.connect(rig, op_sub, "result", op_build2, "t")

        ru.connect(rig, op_build2, "m", op_lookat, "lookat")

    else:
        ru.connect(rig, target, "xform", op_lookat, "lookat")

    ru.connect(rig, op_lookat, "xform", driven, "xform")

    ru.setNodesColor(rig, new_nodes, color)
    ru.promoteTfo(rig, target, t=True, r=False, s=False, demote=True)


def eyesTarget(rig, skel, **kwargs):
    trackTo(rig, skel,
            compname=kwargs['compname'] + "_L",
            nodecolor=kwargs['nodecolor'],
            driven=kwargs['driven'] + "_L",
            target=kwargs['target'] + "_L",
            mirrored=False)
    trackTo(rig, skel,
            compname=kwargs['compname'] + "_R",
            nodecolor=kwargs['nodecolor'],
            driven=kwargs['driven'] + "_R",
            target=kwargs['target'] + "_R",
            mirrored=True)
    target_center = ru.tfo(rig, kwargs['targetcenter'])
    ru.promoteTfo(rig, target_center, t=True, r=False, s=False, demote=True)


def rotationChain(rig, skel, **kwargs):
    compname = kwargs['compname']
    color = kwargs['nodecolor']

    ctl_name = kwargs['ctlname']
    tfo_pattern = kwargs['tfopattern']

    primary_axis = kwargs['primaxis']
    secondary_axis = kwargs['scndaxis']
    lock_secondary_axis = kwargs['lockscndaxis']

    joints = rig.matchNodes(tfo_pattern)

    new_nodes = set()

    ctl_main = ru.tfo(rig, ctl_name)
    rord = ru.axesToRord(primary_axis, secondary_axis)

    ancestors = ru.tfoAncestors(rig, ctl_main)
    for j in joints:
        if j in ancestors:
            raise Exception(f"""Driven joint {rig.nodeName(j)} is an ancestor of control {rig.nodeName(ctl_main)};
                            it's not allowed for now.""")

    ru.updateParms(rig, ctl_main, {
        "restlocal": ru.tfoRestLocal(rig, ctl_main),
        "rord": rord
    })
    ru.promoteTfo(rig, ctl_main, t=False, r=True, s=False, demote=True)
    rig.setNodeTag(ctl_main, "rord_set")

    op_val = ru.addNode(rig, "val", "Value<Vector3>", new_nodes)
    value = hou.Vector3(0, 0, 0)
    for i in range(3):
        if i == primary_axis or (i == secondary_axis and not lock_secondary_axis):
            value[i] = 1.0 / len(joints)
        else:
            value[i] = 0
    ru.updateParms(rig, op_val, { "parm": value } )

    op_mul = ru.addNode(rig, "mul", "Multiply<Vector3>", new_nodes)
    ru.connect(rig, ctl_main, "r", op_mul, "a")
    ru.connect(rig, op_val, "value", op_mul, "b")

    joints.sort(key=lambda j: rig.nodeName(j))

    for i in range(len(joints)):
        joint = joints[i]
        mch_joint_name = ru.mchJointName(f"{compname}_{i}", "Int")
        mch_joint = ru.addNode(rig, mch_joint_name, "TransformObject", new_nodes)
        ru.insertBetweenParentTfo(rig, joint, mch_joint)
        # HACK: in case that the joint has its xform controlled by something else... like arm IK
        ip = ru.getInPort(rig, joint, "xform")
        op = ru.getSourcePort(rig, joint, "xform", must_exist=False)
        actual_mch = mch_joint
        if op != -1:
            rig.addWire(op, ru.getInPort(rig, mch_joint, "xform"))
            mch_joint_name2 = ru.mchJointName(f"{compname}_{i}", "Int2")
            mch_joint2 = ru.addNode(rig, mch_joint_name2, "TransformObject", new_nodes)
            ru.insertBetweenParentTfo(rig, joint, mch_joint2)
            rig.removeWires(rig.portWires(ip), True)
            actual_mch = mch_joint2
        ru.updateParms(rig, actual_mch, { "rord": rord })
        ru.connect(rig, op_mul, "result", actual_mch, "r")

    ru.setNodesColor(rig, new_nodes, color)


def twistChain(rig, skel, **kwargs):
    kwargs['primaxis'] = kwargs['twistaxis']
    kwargs['scndaxis'] = (kwargs['twistaxis'] + 1) % 3
    kwargs['lockscndaxis'] = True
    rotationChain(rig, skel, **kwargs)


def fbikChain(rig, skel, **kwargs):
    compname = kwargs['compname']
    color = kwargs['nodecolor']
    basename = kwargs['basename']

    ctl_parent_name = kwargs['ctlparent']
    tfo_pattern = kwargs['tfopattern']
    config_from_skel = kwargs['configfromskel']
    iterations = kwargs['iterations']
    damping = kwargs['damping']
    tolerance = kwargs['tolerance']

    target_specs = kwargs['targets']

    new_nodes = set()

    joints = rig.matchNodes(tfo_pattern)
    chain = ru.extractTfoChain(rig, joints)

    # Build up a temporary skeleton geo for FBIK
    op_geo = ru.safeAdd(rig, f"FBIKSkel_{compname}", "Value<Geometry>", new_nodes)
    last_add_joint = None
    last_geo_op = op_geo
    for i in range(len(chain)):
        n = chain[i]
        op_add_joint = ru.safeAdd(rig, f"AddJoint_{compname}_{i}", "skel::AddJoint", new_nodes)
        ru.updateParms(rig, op_add_joint, {
            "name": n["name"],
            "xform": n["xform"],
            "color": hou.Vector3(1, 1, 1),
        })
        if i == 0:
            ru.connect(rig, op_geo, "value", op_add_joint, "geo")
            last_geo_op = op_add_joint
        else:
            ru.connect(rig, last_geo_op, "geo", op_add_joint, "geo")
            op_set_parent = ru.addNode(rig, f"SetParent_{compname}_{i}", "skel::SetParent", new_nodes)
            ru.connect(rig, op_add_joint, "geo", op_set_parent, "geo")
            ru.connect(rig, op_add_joint, "ptnum", op_set_parent, "joint")
            ru.connect(rig, last_add_joint, "ptnum", op_set_parent, "parent")
            last_geo_op = op_set_parent
        last_add_joint = op_add_joint

        if config_from_skel:
            pass # Not supported at this point, waiting for SideFX response
            # parms = ru.getParmsNode(rig)
            # skel_port = ru.getOutPort(rig, parms, f"{basename}.skel")
            # source_joint = ru.findSourceJoint(rig, skel, n["node"], True)
            # op_get = ru.addNode(rig, "", "geo::PointAttribValue<Dict>", new_nodes)
            # rig.addWire(skel_port, ru.getInPort(rig, op_get, "geo"))
            # ru.updateParms(rig, op_get, { "attribname": "fbik_jointconfig", "elemnum": source_joint })
            # op_set = ru.addNode(rig, "", "geo::SetPointAttribValue<Dict>", new_nodes)
            # ru.connect(rig, op_get, "value", op_set, "value")
            # ru.connect(rig, last_geo_op, "geo", op_set, "geo")
            # ru.connect(rig, op_add_joint, "ptnum", op_set, "elemnum")
            # ru.updateParms(rig, op_set, { "attribname": "fbik_jointconfig" })
            # last_geo_op = op_set


    output = ru.getNode(rig, "output")
    outp = ru.getOutPort(rig, last_geo_op, "geo")
    inp = ru.getInPort(rig, output, "next")
    rig.setPortName(inp, "FBIKSkel")
    rig.addWire(outp, inp)

    # Actual FBIK logic
    op_skel = ru.safeAdd(rig, f"SkelFromGeo_{compname}", "fbik::SkeletonFromGeo", new_nodes)
    ru.connect(rig, last_geo_op, "geo", op_skel, "geo")
    op_solver = ru.safeAdd(rig, f"Solver_{compname}", "fbik::PhysIKSolver", new_nodes)
    ru.connect(rig, op_skel, "skel", op_solver, "skel")
    ru.updateParms(rig, op_solver, {
        "iterations": iterations,
        "damping": damping,
        "tolerance": tolerance,
    })

    # Targets and their controls
    ctl_parent = None
    if ctl_parent_name:
        ctl_parent = ru.tfo(rig, ctl_parent_name)
    last_solver_op = op_solver
    for ts in target_specs:
        name = ts['target#']
        weight = ts['weight#']
        priority = ts['priority#']
        ctl_name = ts['ctlname#']
        type = ts['targettype#']
        if not ctl_name:
            ctl_name = ru.ctlJointName(f"{name}", "FBIKTarget")

        target = ru.tfo(rig, name)
        xform = ru.tfoRestTransform(rig, target)

        ctl = ru.safeAdd(rig, ctl_name, "TransformObject", new_nodes)
        ru.updateParms(rig, ctl, {
            "restlocal": xform,
        })
        ru.promoteTfo(rig, ctl, t=True, r=True, s=False)

        op_target = ru.safeAdd(rig, f"FBIKTarget_{compname}_{name}", "fbik::Target", new_nodes)
        ru.updateParms(rig, op_target, {
            "type": type,
            "weight": weight,
            "priority": priority,
        });
        ru.connect(rig, ctl, "xform", op_target, "xform")

        op_set_target = ru.addNode(rig, f"SetTarget_{compname}_{name}", "fbik::SetTarget", new_nodes)
        ru.updateParms(rig, op_set_target, { "bone": name, })
        ru.connect(rig, last_solver_op, "solver", op_set_target, "solver")
        ru.connect(rig, op_target, "target", op_set_target, "target")
        last_solver_op = op_set_target

        if ctl_parent:
            ru.setParentTfo(rig, ctl, ctl_parent, compensate_xform=True)


    op_solve = ru.addNode(rig, f"Solve_{compname}", "fbik::SolvePhysIK", new_nodes)
    ru.connect(rig, last_solver_op, "solver", op_solve, "solver")
    op_skel_geo = ru.addNode(rig, f"SkelToGeo_{compname}", "fbik::SkeletonUpdateGeo", new_nodes)
    ru.connect(rig, op_solve, "skel", op_skel_geo, "skel")
    ru.connect(rig, last_geo_op, "geo", op_skel_geo, "geo")

    # Update the Tfos from the updated skeleton geo
    for i in range(len(chain)):
        n = chain[i]
        op_get_xform = ru.addNode(rig, f"GetXform_{compname}_{i}", "skel::GetPointTransform", new_nodes)
        ru.updateParms(rig, op_get_xform, { "name": n["name"], })
        ru.connect(rig, op_skel_geo, "geo", op_get_xform, "geo")
        driven = ru.tfo(rig, n["name"])
        ru.connect(rig, op_get_xform, "xform", driven, "xform")

        # These tfos can't be manually controlled anymore, so we demote them
        ru.promoteTfo(rig, driven, t=False, r=False, s=False, demote=True)


    ru.setNodesColor(rig, new_nodes, color)


def torsoTwist(rig, skel, **kwargs):
    compname = kwargs['compname']
    color = kwargs['nodecolor']

    ctl_parent_name = kwargs['ctlparent']

    target_btm_name = kwargs['targetbtm']
    target_mid_name = kwargs['targetmid']
    target_top_name = kwargs['targettop']
    btm_weight = kwargs['btmweight']
    mid_weight = kwargs['midweight']
    top_weight = kwargs['topweight']

    mid_twist_ratio = kwargs['midtwistratio']
    btm_twist_ratio = kwargs['btmtwistratio']


    twist_axis = kwargs['twistaxis']

    fbik_kwargs = kwargs.copy()
    clt_btm_name = ru.ctlJointName(f"{compname}_Btm")
    ctl_mid_name = ru.ctlJointName(f"{compname}_Mid")
    clt_top_name = ru.ctlJointName(f"{compname}_Top")
    fbik_kwargs['targets'] = [
        { 'target#': target_btm_name, 'ctlname#': clt_btm_name,
          'weight#': btm_weight, 'priority#': 1, 'targettype#': 2 },
        { 'target#': target_mid_name, 'ctlname#': ctl_mid_name,
          'weight#': mid_weight, 'priority#': 1, 'targettype#': 2 },
        { 'target#': target_top_name, 'ctlname#': clt_top_name,
          'weight#': top_weight, 'priority#': 1, 'targettype#': 2 },
    ]

    fbikChain(rig, skel, **fbik_kwargs)

    new_nodes = set()

    ctl_name = ru.ctlJointName(compname)
    ctl_parent = None
    if ctl_parent_name:
        ctl_parent = ru.tfo(rig, ctl_parent_name)

    ctl = ru.safeAdd(rig, ctl_name, "TransformObject", new_nodes)
    ctl_btm = ru.tfo(rig, clt_btm_name)
    ctl_btm_xform = ru.tfoRestTransform(rig, ctl_btm)
    ctl_xlate = ctl_btm_xform.extractTranslates()
    ctl_xform = kmath.lookatAnyY(twist_axis, ctl_xlate)
    ru.setParentTfo(rig, ctl, ctl_parent)
    ru.setTfoRestTransform(rig, ctl, ctl_xform)

    ctl_rest_local = ru.tfoRestLocal(rig, ctl)
    op_val = ru.addNode(rig, f"{ctl_name}_restlocal", "Value<Matrix4>", new_nodes)
    ru.updateParms(rig, op_val, { "parm": ctl_rest_local })
    op_mul0 = ru.addNode(rig, f"{ctl_name}_inherited", "Multiply<Matrix4>", new_nodes)
    ru.connect(rig, op_val, "value", op_mul0, "a")
    ru.connect(rig, ctl_parent, "xform", op_mul0, "b")
    op_invert = ru.addNode(rig, "invert", "Invert<Matrix4>", new_nodes)
    ru.connect(rig, op_mul0, "result", op_invert, "a")
    op_mul = ru.addNode(rig, "mul", "Multiply<Matrix4>", new_nodes)
    ru.connect(rig, op_invert, "result", op_mul, "a")
    ru.connect(rig, ctl, "xform", op_mul, "b")

    def mch_twist_logic(driven_ctl, ratio):
        mch_name = ru.mchJointNameFromCtl(rig.nodeName(driven_ctl))
        mch = ru.safeAdd(rig, mch_name, "TransformObject", new_nodes)
        ru.insertBetweenParentTfo(rig, driven_ctl, mch)
        mch_rest_local = ru.tfoRestLocal(rig, mch)
        op_val2 = ru.addNode(rig, f"{mch_name}_restlocal", "Value<Matrix4>", new_nodes)
        ru.updateParms(rig, op_val2, { "parm": mch_rest_local })
        op_mul2 = ru.addNode(rig, f"{mch_name}_inherited", "Multiply<Matrix4>", new_nodes)
        mch_parent = ru.getParentTfo(rig, mch)
        ru.connect(rig, op_val2, "value", op_mul2, "a")
        ru.connect(rig, mch_parent, "xform", op_mul2, "b")
        op_mul3 = ru.addNode(rig, "mul", "Multiply<Matrix4>", new_nodes)
        ru.connect(rig, op_mul2, "result", op_mul3, "a")
        op_lerp1 = ru.addNode(rig, f"{compname}_ratio", "transform::Slerp<Matrix4>", new_nodes)
        ru.updateParms(rig, op_lerp1, {
            "a": hou.Matrix4(1),
            "bias": ratio,
        })
        ru.connect(rig, op_mul, "result", op_lerp1, "b")
        ru.connect(rig, op_lerp1, "result", op_mul3, "b")
        ru.connect(rig, op_mul3, "result", mch, "xform")

    # mch_btm: similar to mch_mid, but for ctl_btm
    ctl_mid = ru.tfo(rig, ctl_mid_name)
    mch_twist_logic(ctl_mid, mid_twist_ratio)
    mch_twist_logic(ctl_btm, btm_twist_ratio)

    # ru.setParentTfo(rig, ctl_btm, ctl, compensate_xform=True)

    # fbik_target = ru.getDestNode(rig, ctl_mid, "xform", lambda n: rig.callbackName(n) == "fbik::Target")

    ctl_top = ru.tfo(rig, clt_top_name)
    ru.setParentTfo(rig, ctl_top, ctl, compensate_xform=True)

    ru.promoteTfo(rig, ctl, t=False, r=True, s=False)

    ru.setNodesColor(rig, new_nodes, color)



### Not so "component-like" functions below

def tagsFromJointGroups(rig: apex.Graph, skel: hou.Geometry, **kwargs):
    groups = skel.pointGroups()
    for g in groups:
        points = g.points()
        for p in points:
            joint = ru.getNode(rig, p.attribValue("name"), must_exist=False)
            if joint != -1:
                rig.setNodeTag(joint, g.name())


def initializeAbstractControls(rig: apex.Graph, skel: hou.Geometry, **kwargs):
    color = kwargs['nodecolor']

    pattern = kwargs['tfopattern']
    
    new_nodes = set()

    tfos = rig.matchNodes(pattern)
    for tfo in tfos:
        name = rig.nodeName(tfo)
        (_, main, suffix) = ru.splitJointName(name)
        mch_name = ru.joinJointName("MCH", f"ForAC_{main}", suffix)
        mch = ru.safeAdd(rig, mch_name, "rig::FkTransform", new_nodes) 
        ru.insertBetweenParentTfo(rig, tfo, mch)
        rig.setNodeName(tfo, f"__")
        ac = ru.safeAdd(rig, name, "AbstractControl", new_nodes)
        ru.connect(rig, mch, "xform", ac, "xform") 
        ru.promoteAc(rig, ac, x=True, y=True)

    rig.removeNodes(tfos)

    ru.setNodesColor(rig, new_nodes, color)


# Used by abstract_controls_config, which is a SOP unlike other Python-based components
def abstractControlConfig(rig: apex.Graph, skel: hou.Geometry, **kwargs):
    color = kwargs['nodecolor'] if 'nodecolor' in kwargs else kmath.randomColor()

    ac = kwargs['ac']
    x_min_value = kwargs['xrange'][0]
    x_max_value = kwargs['xrange'][1]
    y_min_value = kwargs['yrange'][0]
    y_max_value = kwargs['yrange'][1]
    override_properties = kwargs['overrideproperties']
    lock_range = kwargs['lockrange']
    nub_shape_name = kwargs['nubshapename']
    slider_shape_name = kwargs['slidershapename']
    ac_color = kwargs['color']
    ac_size = kwargs['size']
    slider_length = kwargs['sliderlength']
    style = kwargs['style']
    range_as_labels = kwargs['rangeaslabels']

    # TODO
    if style != 'xslider':
        raise Exception("Only xslider style is supported at this point")
    if range_as_labels and not override_properties:
        raise Exception("rangeaslabels is only supported when overrideproperties is True")


    properties = rig.getNodeProperties(ac)
    control_properties = properties['control']
    if control_properties == None:
        control_properties = apex.Dict()
    
    if override_properties:
        parms = apex.Dict({
            "lock_range": lock_range,
            "x_min": x_min_value,
            "x_max": x_max_value,
            "y_min": y_min_value,
            "y_max": y_max_value,
        })
        control_properties['parms'] = parms
    else:
        parms = control_properties['parms']
        if parms:
            x_min_value = parms['x_min']
            x_max_value = parms['x_max']
            y_min_value = parms['y_min']
            y_max_value = parms['y_max']
        else:
            raise Exception("No parms found in the control properties, but overrideproperties is False")
    
    control_properties['shapeoverride'] = nub_shape_name
    control_properties['color'] = ac_color
    control_properties['shapeoffset'] = hm.buildScale(ac_size, ac_size, ac_size)

    properties['control'] = control_properties

    rig.setNodeProperties(ac, properties)

    parent = ru.getSourceNode(rig, ac, "xform")
    op_comb = ru.addNode(rig, "combine", "rig::CombineParmTransform")
    ru.connect(rig, parent, "xform", op_comb, "inputlocalxform")
    ru.connect(rig, op_comb, "localxform", ac, "xform")
    ru.updateParms(rig, op_comb, {
        "parentlocalscale": hou.Vector3(1, 1, 1),
    })

    zero_point_five = ru.addNode(rig, "zero_point_five", "Value<Float>")
    ru.updateParms(rig, zero_point_five, {
        "parm": 0.5,
    })

    def offsetLogic(src_port, min_value, max_value):
        min_v = ru.addNode(rig, "min", "Value<Float>")
        ru.updateParms(rig, min_v, {
            "parm": min_value,
        })
        max_v = ru.addNode(rig, "max", "Value<Float>")
        ru.updateParms(rig, max_v, {
            "parm": max_value + (0.0001 if max_value == min_value else 0), # Avoid divide by zero
        })

        delta = ru.addNode(rig, "delta", "Subtract<Float>")
        rig.addWire(src_port, ru.getInPort(rig, delta, "a"))
        ru.connect(rig, min_v, "value", delta, "b")
        range = ru.addNode(rig, "range", "Subtract<Float>")
        ru.connect(rig, max_v, "value", range, "a")
        ru.connect(rig, min_v, "value", range, "b")
        ratio = ru.addNode(rig, "ratio", "Divide<Float>")
        ru.connect(rig, delta, "result", ratio, "a")
        ru.connect(rig, range, "result", ratio, "b")
        normalized = ru.addNode(rig, "normalized", "Subtract<Float>")
        ru.connect(rig, ratio, "result", normalized, "a")
        ru.connect(rig, zero_point_five, "value", normalized, "b")
        return normalized

    offset = ru.addNode(rig, "offset", "FloatToVector3")
    x_moveable = x_max_value > x_min_value
    y_moveable = y_max_value > y_min_value

    ru.promoteAc(rig, ac, x=x_moveable, y=y_moveable, demote=True)

    x_src_port = ru.getSourcePort(rig, ac, "x", must_exist=False)
    y_src_port = ru.getSourcePort(rig, ac, "y", must_exist=False)

    if x_src_port != -1 and (style == 'xslider' or style == 'xypanel'):
        normalized = offsetLogic(x_src_port, x_min_value, x_max_value)
        ru.connect(rig, normalized, "result", offset, "x")
    if y_src_port != -1 and (style == 'yslider' or style == 'xypanel'):
        normalized = offsetLogic(y_src_port, y_min_value, y_max_value)
        ru.connect(rig, normalized, "result", offset, "y")

    scaled_offset = ru.addNode(rig, "scaled_offset", "Multiply<Vector3,Float>")
    scale = ru.addNode(rig, "scale", "Value<Float>")
    ru.updateParms(rig, scale, {
        "parm": ac_size * slider_length,
    })
    ru.connect(rig, offset, "vector", scaled_offset, "a")
    ru.connect(rig, scale, "value", scaled_offset, "b")
    ru.connect(rig, scaled_offset, "result", op_comb, "t")

    ac_name = rig.nodeName(ac)
    (_, main, suffix) = ru.splitJointName(ac_name)
    slider_name = ru.joinJointName("UI", f"{main}", suffix)
    slider = ru.addNode(rig, slider_name, "AbstractControl")
    ru.connect(rig, parent, "xform", slider, "xform")
    slider_properties = rig.getNodeProperties(slider)
    container_control_properties = control_properties.freeze()
    container_control_properties['shapeoverride'] = slider_shape_name
    slider_properties['control'] = container_control_properties
    rig.setNodeProperties(slider, slider_properties)

REST_SKEL_NODE_NAME = "rest"
POSED_SKEL_TRANSFORMS_NODE_NAME = "pointtransform"
BLENDSHAPE_CORE_NODE_NAME = "ApplyBlendshapes"

def preparePosedTransforms(rig: apex.Graph, skel: hou.Geometry, **kwargs):
    color = kwargs['nodecolor']

    new_nodes = set()

    root_points = [p for p in skel.points() if ku.getPointParent(p)]
    visited = set()

    rest_skel = ru.getNode(rig, REST_SKEL_NODE_NAME)
    posed_skel = ru.getNode(rig, POSED_SKEL_TRANSFORMS_NODE_NAME)

    def recurse(point, parent=None):
        if point in visited:
            return
        visited.add(point)

        name = point.attribValue("name")
        rest = ru.safeAdd(rig, f"{name}_rest", "skel::GetPointTransform", new_nodes)
        ru.updateParms(rig, rest, { "name": name })
        ru.connect(rig, rest_skel, "value", rest, "geo")
        posed = ru.safeAdd(rig, f"{name}_posed", "skel::GetPointTransform", new_nodes)
        ru.updateParms(rig, posed, { "name": name })
        ru.connect(rig, posed_skel, "geo", posed, "geo")

        if parent:
            parent_name = parent.attribValue("name")
            rest_parent = ru.getNode(rig, f"{parent_name}_rest")
            posed_parent = ru.getNode(rig, f"{parent_name}_posed")
            rest_parent_space = ru.safeAdd(rig, f"{name}_rest_parentspace", "rig::ExtractLocalTransform", new_nodes)
            posed_parent_space = ru.safeAdd(rig, f"{name}_posed_parentspace", "rig::ExtractLocalTransform", new_nodes)
            ru.connect(rig, rest, "xform", rest_parent_space, "xform")
            ru.connect(rig, rest_parent, "xform", rest_parent_space, "parent")
            ru.connect(rig, posed, "xform", posed_parent_space, "xform")
            ru.connect(rig, posed_parent, "xform", posed_parent_space, "parent")

            op_invert = ru.addNode(rig, f"invert", "Invert<Matrix4>", new_nodes)
            ru.connect(rig, rest_parent_space, "localxform", op_invert, "a")
            local_pose = ru.addNode(rig, f"{name}_localpose", "Multiply<Matrix4>", new_nodes)
            ru.connect(rig, posed_parent_space, "localxform", local_pose, "a")
            ru.connect(rig, op_invert, "result", local_pose, "b")
        else:
            op_invert = ru.addNode(rig, f"invert", "Invert<Matrix4>", new_nodes)
            ru.connect(rig, rest, "xform", op_invert, "a")
            local_pose = ru.addNode(rig, f"{name}_localpose", "Multiply<Matrix4>", new_nodes)
            ru.connect(rig, posed, "xform", local_pose, "a")
            ru.connect(rig, op_invert, "result", local_pose, "b")


        for child in ku.getPointChildren(point):
            recurse(child, point)
    
    for point in root_points:
        recurse(point)

    ru.setNodesColor(rig, new_nodes, color)


def driveBlendshapes(rig: apex.Graph, skel: hou.Geometry, **kwargs):
    specs = kwargs['specs']
    color = kwargs['nodecolor']
    basename = kwargs['basename']

    new_nodes = set()

    op_core = ru.safeAdd(rig, BLENDSHAPE_CORE_NODE_NAME, "sop::kinefx::characterblendshapescore",
                         node_storage=new_nodes, get_existing=True)
    ru.updateParms(rig, op_core, {
        "attribs": "P N tangentu",
    })

    parms_node = ru.getParmsNode(rig)
    shp_port = ru.getOutPort(rig, parms_node, f"{basename}.shp")
    rig.addWire(shp_port, ru.getInPort(rig, op_core, "geoinput0"))
    bone_deform = ru.getNode(rig, "Bonedeformation")
    ru.connect(rig, op_core, "geo", bone_deform, "geoinput0")

    def driverLogic(blendshape, last_skel_geo, driver_type, parms):
        point_transforms = ru.getNode(rig, POSED_SKEL_TRANSFORMS_NODE_NAME)
        set_blendshape = ru.addNode(rig, f"set_blendshape_{blendshape}", "geo::SetDetailAttribValue<Float>", new_nodes)
        ru.updateParms(rig, set_blendshape, {
            "attribname": blendshape,
        })
        driver = ru.addNode(rig, f"_driver_{blendshape}", "Value<Float>", new_nodes)
        p = ru.getOutPort(rig, last_skel_geo, "geo", must_exist=False)
        if p == -1:
            p = ru.getOutPort(rig, last_skel_geo, "value")
        rig.addWire(p, ru.getInPort(rig, set_blendshape, "geo"))

        ru.connect(rig, driver, "value", set_blendshape, "value")

        if driver_type == 'single_channel':
            control_name = parms['singlecontrol']
            channle_name = parms['singlechannel']
            port_name = f"{control_name}_{channle_name}" if channle_name != 'none' else control_name
            ru.connect(rig, parms_node, port_name, driver, "parm")
        elif driver_type == 'angle_between':
            first_joint_name = parms['firstjoint']
            center_joint_name = parms['centerjoint']
            last_joint_name = parms['lastjoint']
            sign_normal_index = parms['signnormal']
            first_joint = ru.safeAdd(rig, f"{first_joint_name}_posed", "skel::GetPointTransform", new_nodes, get_existing=True)
            ru.updateParms(rig, first_joint, { "name": first_joint_name })
            ru.connect(rig, point_transforms, "geo", first_joint, "geo")
            center_joint = ru.safeAdd(rig, f"{center_joint_name}_posed", "skel::GetPointTransform", new_nodes, get_existing=True)
            ru.updateParms(rig, center_joint, { "name": center_joint_name })
            ru.connect(rig, point_transforms, "geo", center_joint, "geo")
            last_joint = ru.safeAdd(rig, f"{last_joint_name}_posed", "skel::GetPointTransform", new_nodes, get_existing=True)
            ru.updateParms(rig, last_joint, { "name": last_joint_name })
            ru.connect(rig, point_transforms, "geo", last_joint, "geo")

            op_explode_first = ru.addNode(rig, f"{first_joint_name}_xlt", "transform::Explode", new_nodes)
            ru.connect(rig, first_joint, "xform", op_explode_first, "m")
            op_explode_center = ru.addNode(rig, f"{center_joint_name}_xlt", "transform::Explode", new_nodes)
            ru.connect(rig, center_joint, "xform", op_explode_center, "m")
            op_explode_last = ru.addNode(rig, f"{last_joint_name}_xlt", "transform::Explode", new_nodes)
            ru.connect(rig, last_joint, "xform", op_explode_last, "m")

            op_sub1 = ru.addNode(rig, "sub1", "Subtract<Vector3>", new_nodes)
            ru.connect(rig, op_explode_center, "t", op_sub1, "a")
            ru.connect(rig, op_explode_first, "t", op_sub1, "b")
            op_sub2 = ru.addNode(rig, "sub2", "Subtract<Vector3>", new_nodes)
            ru.connect(rig, op_explode_last, "t", op_sub2, "a")
            ru.connect(rig, op_explode_center, "t", op_sub2, "b")

            op_degrees = ru.addNode(rig, "degrees_between", "ko::DegreesBetween", new_nodes)
            ru.updateParms(rig, op_degrees, {
                "signed": 1
            })
            ru.connect(rig, op_sub1, "result", op_degrees, "a")
            ru.connect(rig, op_sub2, "result", op_degrees, "b")

            op_bases = ru.addNode(rig, "bases", "ko::GetBases", new_nodes)
            ru.connect(rig, first_joint, "xform", op_bases, "xform")
            op_switch = ru.addNode(rig, "switch_signnormal", "Switch<Vector3>", new_nodes)
            ru.updateParms(rig, op_switch, {
                "index": sign_normal_index,
            })
            ru.connect(rig, op_bases, "x", op_switch, "input")
            ru.connect(rig, op_bases, "y", op_switch, "input")
            ru.connect(rig, op_bases, "z", op_switch, "input")
            ru.connect(rig, op_switch, "out", op_degrees, "signnormal")

            ru.connect(rig, op_degrees, "result", driver, "parm")
        elif driver_type == 'swing_twist':
            twist_joint_name = parms['twistjoint']
            twist_axis = parms['twistaxis']
            local_pose = ru.getNode(rig, f"{twist_joint_name}_localpose")
            op_convert = ru.addNode(rig, "convert", "Convert<Matrix4,Vector4>", new_nodes)
            ru.connect(rig, local_pose, "result", op_convert, "a")
            op_bases = ru.addNode(rig, "bases", "ko::GetBases", new_nodes)
            basis_port_name = None
            if twist_axis == 'localx':
                basis_port_name = "x" 
            elif twist_axis == 'localy':
                basis_port_name = "y"
            elif twist_axis == 'localz':
                basis_port_name = "z"
            else:
                raise Exception(f"Unknown twist axis: {twist_axis}")
            op_decomp = ru.addNode(rig, "decomp", "ko::SwingTwistDecomp2", new_nodes)
            ru.connect(rig, op_convert, "b", op_decomp, "q")
            ru.connect(rig, op_bases, basis_port_name, op_decomp, "twistaxis")
            op_degrees = ru.addNode(rig, "degrees", "RadiansToDegrees<Float>", new_nodes)
            ru.connect(rig, op_decomp, "twistradians", op_degrees, "radians")

            ru.connect(rig, op_degrees, "degrees", driver, "parm")
        else:
            raise Exception(f"Unknown driver type: {driver_type}")

        
        return (driver, set_blendshape)
    
    def mappingLogic(blendshape, driver, set_blendshape, driver_range, driven_range, use_ramp, ramp):
        driver_min = driver_range[0]
        driver_max = driver_range[1]
        driven_min = driven_range[0]
        driven_max = driven_range[1]

        if use_ramp:
            op_remap1 = ru.addNode(rig, f"remap1", "ko::Remap<Float>", new_nodes)
            ru.connect(rig, driver, "value", op_remap1, "value")
            ru.updateParms(rig, op_remap1, {
                'old_min': driver_min,
                'old_max': driver_max,
                'new_min': 0.0,
                'new_max': 1.0,
                'clamp': True,
            })
            
            ramp_interps = ["constant", "linear", "catmullrom", "monotonecubic", "bezier", "bspline", "hermite"]
            bases_str = ", ".join([
                f"\"{ramp_interps[ramp[i]['ramp#_#interp']]}\"" for i in range(len(ramp))
                ])
            values_str = ", ".join([str(ramp[i]['ramp#_#value']) for i in range(len(ramp))])
            poses_str = ", ".join([str(ramp[i]['ramp#_#pos']) for i in range(len(ramp))])
            vex_snippet =f"""
                string basis[] = array({bases_str});
                float value[] = array({values_str});
                float pos[] = array({poses_str});
                result = spline(basis, t, value, pos);
            """

            op_spline = ru.addNode(rig, f"spline", "RunVex", new_nodes)
            ru.updateParms(rig, op_spline, {
                "snippet": vex_snippet,
            })
            op_spline_inp = ru.getInPort(rig, op_spline, "inputs")
            op_spline_outp = ru.getOutPort(rig, op_spline, "outputs")
            t_p = rig.addSubPort(op_spline_inp, "t")
            remap1_result = ru.getOutPort(rig, op_remap1, "result")
            rig.addWire(remap1_result, t_p)

            op_remap2 = ru.addNode(rig, f"remap2", "ko::Remap<Float>", new_nodes)
            ru.updateParms(rig, op_remap2, {
                'old_min': 0.0,
                'old_max': 1.0,
                'new_min': driven_min,
                'new_max': driven_max,
                'clamp': True,
            })
            result_p = rig.addSubPort(op_spline_outp, "result")
            remap2_value = ru.getInPort(rig, op_remap2, "value")
            rig.addWire(result_p, remap2_value)

            ru.connect(rig, op_remap2, "result", set_blendshape, "value")
        else:
            op_remap = ru.addNode(rig, f"remap", "ko::Remap<Float>", new_nodes)
            ru.connect(rig, driver, "value", op_remap, "value")
            ru.updateParms(rig, op_remap, {
                'old_min': driver_min,
                'old_max': driver_max,
                'new_min': driven_min,
                'new_max': driven_max,
                'clamp': True,
            })
            ru.connect(rig, op_remap, "result", set_blendshape, "value")

        save_driver_value = ru.addNode(rig, f"save_driver_value_{blendshape}", "geo::SetDetailAttribValue<Float>", new_nodes)
        ru.updateParms(rig, save_driver_value, {
            "attribname": f"_driver_{blendshape}",
        })
        ru.connect(rig, set_blendshape, "geo", save_driver_value, "geo")
        ru.connect(rig, driver, "value", save_driver_value, "value")
        
        return save_driver_value


    i = 0
    last_skel_geo = ru.getNode(rig, POSED_SKEL_TRANSFORMS_NODE_NAME)
    for i in range(len(specs)):
        spec = specs[i]

        mirror = spec["mirror#"]

        (driver, set_blendshape) = driverLogic(spec['blendshape#'], last_skel_geo, spec['drivertype#'], {
            # single channel
            'singlecontrol': spec['singlecontrol#'],
            'singlechannel': spec['singlechannel#'],
            # angle between
            'centerjoint': spec['centerjoint#'],
            'firstjoint': spec['firstjoint#'],
            'lastjoint': spec['lastjoint#'],
            'signnormal': spec['signnormal#'],
            # swing twist
            'twistjoint': spec['twistjoint#'],
            'twistaxis': spec['twistaxis#'],
        })
        last_skel_geo = mappingLogic(spec['blendshape#'], driver, set_blendshape,
                    spec['driverrange#'],
                    spec['drivenrange#'],
                    spec['useramp#'],
                    spec['ramp#'])
        if mirror:
            (driver, set_blendshape) = driverLogic(spec['mirrorblendshape#'], last_skel_geo, spec['drivertype#'], {
                # single channel
                'singlecontrol': spec['mirrorsinglecontrol#'],
                'singlechannel': spec['mirrorsinglechannel#'],
                # angle between
                'centerjoint': spec['mirrorcenterjoint#'],
                'firstjoint': spec['mirrorfirstjoint#'],
                'lastjoint': spec['mirrorlastjoint#'],
                'signnormal': spec['signnormal#'], # always the same as the non-mirror one
                # swing twist
                'twistjoint': spec['mirrortwistjoint#'],
                'twistaxis': spec['twistaxis#'], # always the same as the non-mirror one
            })
            last_skel_geo = mappingLogic(spec['mirrorblendshape#'], driver, set_blendshape,
                        spec['mirrordriverrange#'] if spec['usemirrordriverrange#'] else spec['driverrange#'],
                        spec['drivenrange#'],
                        spec['useramp#'],
                        spec['ramp#'])

    ru.connect(rig, last_skel_geo, "geo", op_core, "geoinput1")
    output_node = ru.getOutputNode(rig)
    ru.connect(rig, last_skel_geo, "geo", output_node, f"{basename}.skel")

    ru.setNodesColor(rig, new_nodes, color)

        