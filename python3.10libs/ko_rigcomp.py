import hou
import apex
from kinefx import utils as ku
import ko_rigutils as ru
import ko_rigsubcomp as rsc
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
    smooth_ik = ru.safeAdd(rig, smooth_ik_name, "rig::TwoBoneIK", new_nodes)
    ru.connect(rig, mch_root, "xform", smooth_ik, "rootdriver")
    ru.connect(rig, ctl_target, "xform", smooth_ik, "goal")
    rig.addWire(to_twist_xform, ru.getInPort(rig, smooth_ik, "twist"))

    if tip_follow_control:
        # ru.setParentTfo(rig, tip, ctl_target, compensate_xform=True)
        ru.connect(rig, ctl_target, "xform", tip, "xform")
    else:
        pass
        # This causes the tip to "kinda" follow the target, but not exactly... accidentially found this behavior
        
        # mch_tip_name = ru.joinJointName("MCH", f"IKTipOut", "{comp_name}")
        # mch_tip = ru.safeAdd(rig, mch_tip_name, "TransformObject", new_nodes)
        # ru.insertBetweenParentTfo(rig, tip, mch_tip)

        # tip_rest_local_xform = rig.getNodeParms(tip)["restlocal"]
        # tip_rest_local = ru.addNode(rig, f"{comp_name}_tip_rest_local", "Value<Matrix4>", new_nodes)
        # ru.updateParms(rig, tip_rest_local, { "parm": tip_rest_local_xform })
        # tip_parent = ru.getParentTfo(rig, mch_tip)
        # tip_parent_ori_xform = ru.getOriginalTransform(rig, skel, tip_parent)
        # mid_ori_xform = ru.getOriginalTransform(rig, skel, mid)
        # tip_parent_ori_xform_mid_space = tip_parent_ori_xform * mid_ori_xform.inverted()
        # tip_parent_ori_mid_space = ru.addNode(rig, f"{comp_name}_tip_parent_ori_mid_space", "Value<Matrix4>", new_nodes)
        # ru.updateParms(rig, tip_parent_ori_mid_space, { "parm": tip_parent_ori_xform_mid_space })
        # op_mul = ru.addNode(rig, "mul", "Multiply<Matrix4>", new_nodes)
        # ru.connect(rig, tip_parent_ori_mid_space, "value", op_mul, "a")
        # ru.connect(rig, mid, "xform", op_mul, "b")
        # target_in_tip_parent_space = ru.addNode(rig, f"{comp_name}_target_in_tip_parent_space", "rig::ExtractLocalTransform", new_nodes)
        # ru.connect(rig, ctl_target, "xform", target_in_tip_parent_space, "xform")
        # ru.connect(rig, op_mul, "result", target_in_tip_parent_space, "parent")
        # ru.connect(rig, target_in_tip_parent_space, "localxform", mch_tip, "restlocal")
        # ru.connect(rig, mch_tip, "xform", tip, "xform")


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
    tip_following_ik = kwargs['tipfollowingik']

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

    # if tip_following_ik:
    #     ru.connect(rig, tip_fk, "xform", tip, "xform")
    # else:
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

    (_, _, suffix) = ru.splitJointName(ctl_ball_name)
    ctl_ball_rev_name = f"CTL_RevBall_{compname}"
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
        ru.updateParms(rig, op_lerp, { "b": hou.Matrix4(1) })
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


def driveBlendshapes(rig: apex.Graph, skel: hou.Geometry, **kwargs):
    specs = kwargs['specs']
    color = kwargs['nodecolor']
    basename = kwargs['basename']

    new_nodes = set()

    op_core = ru.setUpBlendshapeCoreNode(rig, basename, new_nodes)
    parms_node = ru.getParmsNode(rig)

    def driverLogic(blendshape, last_skel_geo, driver_type, parms):
        point_transforms = ru.getNode(rig, POSED_SKEL_TRANSFORMS_NODE_NAME)
        set_blendshape = ru.safeAdd(rig, f"set_blendshape_{blendshape}", "geo::SetDetailAttribValue<Float>", new_nodes)
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
            channel_name = parms['singlechannel']
            port_name = f"{control_name}_{channel_name}" if channel_name != 'none' else control_name
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

    i = 0
    last_skel_geo = ru.getLatestPosedSkelNode(rig)
    for i in range(len(specs)):
        spec = specs[i]
        if not spec["enable#"]:
            continue

        blendshape_name = spec['blendshape#']

        mirror = spec["mirror#"]

        (driver, set_blendshape) = driverLogic(blendshape_name, last_skel_geo, spec['drivertype#'], {
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

        rsc.mappingLogic(rig, ru.getOutPort(rig, driver, "value"), ru.getInPort(rig, set_blendshape, "value"),
                    spec['driverrange#'],
                    spec['drivenrange#'],
                    spec['useramp#'],
                    spec['ramp#'],
                    new_nodes)

        save_driver_value = ru.addNode(rig, f"save_driver_value_{blendshape_name}", "geo::SetDetailAttribValue<Float>", new_nodes)
        ru.updateParms(rig, save_driver_value, {
            "attribname": f"_driver_{blendshape_name}",
        })
        ru.connect(rig, set_blendshape, "geo", save_driver_value, "geo")
        ru.connect(rig, driver, "value", save_driver_value, "value")
        last_skel_geo = save_driver_value
        ru.updateLatestPosedSkelNode(rig, last_skel_geo)

        if mirror:
            mirror_blendshape_name = spec['mirrorblendshape#']
            (driver, set_blendshape) = driverLogic(mirror_blendshape_name, last_skel_geo, spec['drivertype#'], {
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

            rsc.mappingLogic(rig, ru.getOutPort(rig, driver, "value"), ru.getInPort(rig, set_blendshape, "value"),
                        spec['mirrordriverrange#'] if spec['usemirrordriverrange#'] else spec['driverrange#'],
                        spec['drivenrange#'],
                        spec['useramp#'],
                        spec['ramp#'],
                        new_nodes)
            save_driver_value = ru.addNode(rig, f"save_driver_value_{mirror_blendshape_name}",
                                           "geo::SetDetailAttribValue<Float>", new_nodes)
            ru.updateParms(rig, save_driver_value, {
                "attribname": f"_driver_{mirror_blendshape_name}",
            })
            ru.connect(rig, set_blendshape, "geo", save_driver_value, "geo")
            ru.connect(rig, driver, "value", save_driver_value, "value")
            last_skel_geo = save_driver_value
            ru.updateLatestPosedSkelNode(rig, last_skel_geo)

    ru.connect(rig, last_skel_geo, "geo", op_core, "geoinput1")
    output_node = ru.getOutputNode(rig)
    ru.connect(rig, last_skel_geo, "geo", output_node, f"{basename}.skel")

    ru.setNodesColor(rig, new_nodes, color)



def rbfBlendshapes(rig: apex.Graph, skel: hou.Geometry, **kwargs):
    base_name = kwargs['basename']
    comp_name = kwargs['compname']
    color = kwargs['nodecolor']
    joint_specs = kwargs['joints']
    blendshapes = kwargs['blendshapes']
    examples = kwargs['examples']

    new_nodes = set()

    example_array = ru.addNode(rig, f"{comp_name}_examples", "array::Build<Float>", new_nodes)
    example_array_port = ru.getInPort(rig, example_array, "values")  
    index = 0

    def tripleDecomp(q_node, arryr_port, order, mask_0, mask_1, mask_2):
        nonlocal index
        op_bases = ru.addNode(rig, f"bases", "ko::GetBases", new_nodes)
        op_decomp1 = ru.addNode(rig, f"decomp1", "ko::SwingTwistDecomp2", new_nodes)
        op_decomp2 = ru.addNode(rig, f"decomp2", "ko::SwingTwistDecomp2", new_nodes)
        if order == 0 or order == 1:
            axis_0 = "x"
        elif order == 2 or order == 3:
            axis_0 = "y"
        elif order == 4 or order == 5:
            axis_0 = "z"
        if order == 2 or order == 4:
            axis_1 = "x"
        elif order == 0 or order == 5:
            axis_1 = "y"
        elif order == 1 or order == 3:
            axis_1 = "z"
        ru.connect(rig, op_bases, axis_0, op_decomp1, "twistaxis")
        ru.connect(rig, op_bases, axis_1, op_decomp2, "twistaxis")
        ru.connect(rig, q_node, "b", op_decomp1, "q")
        ru.connect(rig, op_decomp1, "swing", op_decomp2, "q")
        op_angle_axis = ru.addNode(rig, "angle_axis", "ko::QToAngleAxis", new_nodes)
        ru.connect(rig, op_decomp2, "swing", op_angle_axis, "q")
        if mask_0:
            port = rig.addSubPort(arryr_port, str(index))
            index += 1
            rig.addWire(ru.getOutPort(rig, op_decomp1, "twistradians"), port)
        if mask_1:
            port = rig.addSubPort(arryr_port, str(index))
            index += 1
            rig.addWire(ru.getOutPort(rig, op_decomp2, "twistradians"), port)
        if mask_2:
            port = rig.addSubPort(arryr_port, str(index))
            index += 1
            rig.addWire(ru.getOutPort(rig, op_angle_axis, "radians"), port)

    def twistSwing(q_node, arryr_port, twist_axis, use_twist, use_swing):
        nonlocal index
        op_bases = ru.addNode(rig, f"bases", "ko::GetBases", new_nodes)
        op_decomp = ru.addNode(rig, f"decomp", "ko::SwingTwistDecomp2", new_nodes)
        ru.connect(rig, q_node, "b", op_decomp, "q")
        ru.connect(rig, op_bases, twist_axis, op_decomp, "twistaxis")
        if use_twist:
            port = rig.addSubPort(arryr_port, str(index))
            index += 1
            rig.addWire(ru.getOutPort(rig, op_decomp, "twistradians"), port)
        if use_swing:
            op_floats = ru.addNode(rig, f"floats", "Vector4ToFloat", new_nodes)
            ru.connect(rig, op_decomp, "swing", op_floats, "vector")
            port = rig.addSubPort(arryr_port, str(index))
            rig.addWire(ru.getOutPort(rig, op_floats, "x"), port)
            port = rig.addSubPort(arryr_port, str(index + 1))
            rig.addWire(ru.getOutPort(rig, op_floats, "y"), port)
            port = rig.addSubPort(arryr_port, str(index + 2))
            rig.addWire(ru.getOutPort(rig, op_floats, "z"), port)
            port = rig.addSubPort(arryr_port, str(index + 3))
            rig.addWire(ru.getOutPort(rig, op_floats, "w"), port)
            index += 4
            

    for i in range(len(examples)):
        example = examples[i]
        example_name = example['name']
        joint_xforms = example['joint_xforms']
        for js in joint_specs:
            joint_name = js['joint#']
            joint_type = js['type#']
            joint_xform = ru.addNode(rig, f"{comp_name}_{example_name}_{joint_name}_xform", "Value<Matrix4>", new_nodes)
            ru.updateParms(rig, joint_xform, { "parm": joint_xforms[joint_name] })
            joint_xform_q = ru.addNode(rig, f"{comp_name}_{example_name}_{joint_name}_xform_q", "Convert<Matrix4,Vector4>", new_nodes)
            ru.connect(rig, joint_xform, "value", joint_xform_q, "a")
            if joint_type == 'tripledecomp':
                tripleDecomp(joint_xform_q, example_array_port, js['order#'], js['mask0_#'], js['mask1_#'], js['mask2_#'])
            elif joint_type == 'twistswing':
                twistSwing(joint_xform_q, example_array_port, js['twistaxis#'], js['usetwist#'], js['useswing#'])
            else:
                raise Exception(f"Unknown joint type: {joint_type}")

    target_array = ru.addNode(rig, f"{comp_name}_target", "array::Build<Float>", new_nodes)
    target_array_port = ru.getInPort(rig, target_array, "values")
    index = 0
    for js in joint_specs:
        joint_name = js['joint#']
        joint_type = js['type#']
        local_pose = ru.getNode(rig, f"{joint_name}_localpose")
        local_pose_q = ru.addNode(rig, f"{comp_name}_{joint_name}_localpose_q", "Convert<Matrix4,Vector4>", new_nodes)
        ru.connect(rig, local_pose, "result", local_pose_q, "a")
        if joint_type == 'tripledecomp':
            tripleDecomp(local_pose_q, target_array_port, js['order#'], js['mask0_#'], js['mask1_#'], js['mask2_#'])
        elif joint_type == 'twistswing':
            twistSwing(local_pose_q, target_array_port, js['twistaxis#'], js['usetwist#'], js['useswing#'])
        else:
            raise Exception(f"Unknown joint type: {joint_type}")

    op_rbf = ru.safeAdd(rig, f"{comp_name}_RBF", "rig::RBFInterpolation", new_nodes)
    ru.updateParms(rig, op_rbf, {
        "clamp": 1,
        "hyperplane": 0,
        "exponent": 1.0,
        "falloff": 1.0,
        "damping": 0.0,
        "iterations": 10,
        "reload": 1,
    })
    ru.connect(rig, example_array, "result", op_rbf, "examples")
    ru.connect(rig, target_array, "result", op_rbf, "target")

    op_core = ru.setUpBlendshapeCoreNode(rig, base_name, new_nodes)

    for i in range(len(blendshapes)):
        bs = blendshapes[i]
        blendshape = bs['blendshape#']
        value_range = bs['range#']

        set_blendshape = ru.safeAdd(rig, f"set_blendshape_{blendshape}", "geo::SetDetailAttribValue<Float>", new_nodes)
        ru.updateParms(rig, set_blendshape, {
            "attribname": blendshape,
        })
        ru.connect(rig, ru.getLatestPosedSkelNode(rig), "geo", set_blendshape, "geo")
        ru.updateLatestPosedSkelNode(rig, set_blendshape)
        op_clamp = ru.addNode(rig, f"clamp_{blendshape}", "Clamp<Float>", new_nodes)
        ru.connect(rig, op_clamp, "result", set_blendshape, "value")
        ru.updateParms(rig, op_clamp, {
            "min": value_range[0],
            "max": value_range[1],
        })

        op_add_all = ru.addNode(rig, f"{comp_name}_{blendshape}_unclamped", "Add<Float>", new_nodes)
        ru.connect(rig, op_add_all, "result", op_clamp, "val")

        for j in range(len(examples)):
            example = examples[j]
            example_name = example['name']
            blendshape_weights = example['blendshape_weights']

            example_weight = blendshape_weights[blendshape]
            op_get = ru.addNode(rig, f"{comp_name}_{example_name}_{blendshape}_weight", "array::Get<Float>", new_nodes)
            ru.connect(rig, op_rbf, "weights", op_get, "array")
            ru.updateParms(rig, op_get, { "index": i })
            op_mul = ru.addNode(rig, f"mul", "Multiply<Float>", new_nodes)
            ru.updateParms(rig, op_mul, { "a": example_weight })
            ru.connect(rig, op_get, "value", op_mul, "b")
            port = rig.addSubPort(ru.getInPort(rig, op_add_all, "b"), example_name)
            rig.addWire(ru.getOutPort(rig, op_mul, "result"), port)

    ru.connect(rig, ru.getLatestPosedSkelNode(rig), "geo", op_core, "geoinput1")

    ru.setNodesColor(rig, new_nodes, color)



# Used by RBF Blendshapes SOP internally
def extractLocalPosesFromRBFExample(rig: apex.Graph, skel: hou.Geometry, **kwargs):
    color = kwargs['nodecolor']

    joint_specs = kwargs['joints']
    example = kwargs['other_geos']['/Example.skel']

    new_nodes = set()
    parms_node = ru.getParmsNode(rig)
    posed_skel = ru.getNode(rig, POSED_SKEL_TRANSFORMS_NODE_NAME)
    ru.unconnectAllInputs(rig, posed_skel)
    ru.connect(rig, parms_node, "next", posed_skel, "geo")
    rig.setPortName(rig.getOutputPorts(parms_node)[-1], "Example.skel")

    for js in joint_specs:
        joint_name = js['joint#']
        local_pose = ru.getNode(rig, f"{joint_name}_localpose")
        output_node = ru.getOutputNode(rig)
        ru.connect(rig, local_pose, "result", output_node, "next")
        rig.setPortName(rig.getInputPorts(output_node)[-1], f"{joint_name}")

    ru.setNodesColor(rig, new_nodes, color)


        
def keyposeControls(rig: apex.Graph, skel: hou.Geometry, **kwargs):
    comp_name = kwargs['compname']
    color = kwargs['nodecolor']
    specs = kwargs['keyposes']
    geo_path = kwargs['geopath']

    new_nodes = set()

    keyposes_geo = kwargs['other_geos'][geo_path]
    packed_keyposes = keyposes_geo.prims()

    skel = ru.computeLocalTransforms(skel)
    joint_dict = {}
    for p in skel.points():
        joint_dict[p.attribValue("name")] = p.number()

    def keyposeLogic(pose_name, control_name, channel_name, driver_range, driven_range, use_ramp, ramp):
        posed_skel = [p for p in packed_keyposes if p.attribValue("name") == pose_name][0].getEmbeddedGeometry()
        # keypose = ru.computeLocalTransforms(keypose)

        # posed_skel = hou.Geometry()
        # posed_skel.copy(skel)
        # for p in keypose.points():
        #     joint_name = p.attribValue("name")
        #     pid = joint_dict[joint_name]
        #     psp = posed_skel.point(pid)
        #     sp = skel.point(pid)
        #     psp.setAttribValue("P", p.attribValue("P"))
        #     psp.setAttribValue("transform", p.attribValue("transform"))
        # posed_skel = ru.computeLocalTransforms(posed_skel)

        op_weight = ru.safeAdd(rig, f"weight_{comp_name}_{pose_name}", "Value<Float>", new_nodes)
        for psp in posed_skel.points():
            joint_name = psp.attribValue("name")
            pid = joint_dict[joint_name]
            sp = skel.point(pid)
            # local_xform is in parent's space, as KineFx's convention
            # local_pose is in rest child's space
            local_xform_posed = hou.Matrix4(psp.attribValue("localtransform"))
            local_xform_rest = hou.Matrix4(sp.attribValue("localtransform"))
            if kmath.matrixEqualTo(local_xform_posed, local_xform_rest):
                continue
            local_pose_xform = local_xform_posed * local_xform_rest.inverted()
            local_pose_name = f"{pose_name}_{joint_name}_localpose"
            local_pose = ru.addNode(rig, local_pose_name, "Value<Matrix4>", new_nodes)
            ru.updateParms(rig, local_pose, { "parm": local_pose_xform })

            joint = ru.tfo(rig, joint_name)
            mch_keypose_name = f"MCH_{comp_name}_{joint_name}"
            mch_keypose = ru.getNode(rig, mch_keypose_name, must_exist=False)
            if mch_keypose == -1:
                mch_keypose = ru.safeAdd(rig, mch_keypose_name, "TransformObject", new_nodes, get_existing=True)
                ru.insertBetweenParentTfo(rig, joint, mch_keypose)
            op_srt_combine = ru.safeAdd(rig, f"combine_{comp_name}_{joint_name}",
                                        "ko::TransformsSRTCombine", new_nodes, get_existing=True)
            joint_parms = rig.getNodeParms(joint)
            ru.updateParms(rig, op_srt_combine, {
                "xord": joint_parms['xord'],
                "rord": joint_parms['rord'],
            })
            op_mul1 = ru.safeAdd(rig, f"mul1_{comp_name}_{joint_name}", "Multiply<Matrix4>", new_nodes, get_existing=True)
            op_mul2 = ru.safeAdd(rig, f"mul2_{comp_name}_{joint_name}", "Multiply<Matrix4>", new_nodes, get_existing=True)
            ru.connect(rig, op_srt_combine, "result", op_mul1, "a")
            ru.connect(rig, mch_keypose, "xform", op_mul1, "b")
            ru.connect(rig, op_srt_combine, "result", op_mul2, "a")
            ru.connect(rig, mch_keypose, "localxform", op_mul2, "b")
            ru.connect(rig, op_mul1, "result", joint, "parent")
            ru.connect(rig, op_mul2, "result", joint, "parentlocal")

            op_xforms = ru.safeAdd(rig, f"array_xforms_{comp_name}_{joint_name}",
                                         "array::Build<Matrix4>", new_nodes, get_existing=True)
            op_weights = ru.safeAdd(rig, f"array_weights_{comp_name}_{joint_name}",
                                          "array::Build<Float>", new_nodes, get_existing=True)
            ru.connect(rig, op_xforms, "result", op_srt_combine, "transforms")
            ru.connect(rig, op_weights, "result", op_srt_combine, "weights")
            op_xform_port = rig.addSubPort(ru.getInPort(rig, op_xforms, "values"), pose_name)
            rig.addWire(ru.getOutPort(rig, local_pose, "value"), op_xform_port)
            op_weight_port = rig.addSubPort(ru.getInPort(rig, op_weights, "values"), pose_name)
            rig.addWire(ru.getOutPort(rig, op_weight, "value"), op_weight_port)

        parms_node = ru.getParmsNode(rig)
        driver = ru.addNode(rig, f"_driver_{pose_name}", "Value<Float>", new_nodes)
        port_name = f"{control_name}_{channel_name}"
        ru.connect(rig, parms_node, port_name, driver, "parm")

        rsc.mappingLogic(rig, ru.getOutPort(rig, driver, "value"), ru.getInPort(rig, op_weight, "parm"),
                    driver_range,
                    driven_range,
                    use_ramp,
                    ramp, 
                    new_nodes)

    for spec in specs:
        if not spec["enable#"]:
            continue
        if not spec['posename#']:
            continue

        keyposeLogic(
            pose_name=spec['posename#'],
            control_name=spec['driverac#'],
            channel_name=spec['driveracchannel#'],
            driver_range=spec['driverrange#'],
            driven_range=spec['drivenrange#'],
            use_ramp=spec['useramp#'],
            ramp=spec['ramp#'],
        )
        if spec['mirror#']:
            keyposeLogic(
                pose_name=spec['mirrorposename#'],
                control_name=spec['mirrordriverac#'],
                channel_name=spec['mirrordriveracchannel#'],
                driver_range=spec['mirrordriverrange#'] if spec['usemirrordriverrange#'] else spec['driverrange#'],
                driven_range=spec['drivenrange#'],
                use_ramp=spec['useramp#'],
                ramp=spec['ramp#'],
            )

    ru.setNodesColor(rig, new_nodes, color)


def volumeHolder(rig: apex.Graph, skel: hou.Geometry, **kwargs):
    color = kwargs['nodecolor']
    comp_name = kwargs['compname']

    followed_name = kwargs['followed']
    holder_name = kwargs['holder']
    bias = kwargs['bias']
    def_skel_name = kwargs['packeddefskeleton']
    if def_skel_name == '':
        def_skel = skel
    else:
        def_skel = kwargs['other_geos'][def_skel_name]

    new_nodes = set()

    def_skel = ru.computeLocalTransforms(def_skel)

    followed = ru.tfo(rig, followed_name)
    followed_ori_parent = ru.getOriginalParentTfo(rig, def_skel, followed)
    followed_posed_parent_space = ru.addNode(rig, f"{comp_name}_{followed_name}_posed_parentspace",
                                             "rig::ExtractLocalTransform", new_nodes)
    ru.connect(rig, followed_ori_parent, "xform", followed_posed_parent_space, "parent")
    ru.connect(rig, followed, "xform", followed_posed_parent_space, "xform")
    followed_ori_rest_local = ru.getOriginalRestLocal(rig, def_skel, followed)
    followed_rest_parent_space = ru.addNode(rig, f"{comp_name}_{followed_name}_rest_parentspace",
                                            "Value<Matrix4>", new_nodes)
    ru.updateParms(rig, followed_rest_parent_space, { "parm": followed_ori_rest_local })
    op_invert = ru.addNode(rig, f"invert", "Invert<Matrix4>", new_nodes)
    ru.connect(rig, followed_rest_parent_space, "value", op_invert, "a")
    followed_local_pose = ru.addNode(rig, f"{comp_name}_{followed_name}_localpose", "Multiply<Matrix4>", new_nodes)
    ru.connect(rig, followed_posed_parent_space, "localxform", followed_local_pose, "a")
    ru.connect(rig, op_invert, "result", followed_local_pose, "b")

    holder = ru.tfo(rig, holder_name)
    holder_old_rest_local = ru.addNode(rig, f"{comp_name}_{holder_name}_old_rest_local", "Value<Matrix4>", new_nodes)
    ru.updateParms(rig, holder_old_rest_local, { "parm": ru.tfoRestLocal(rig, holder) })
    op_invert2 = ru.addNode(rig, f"invert", "Invert<Matrix4>", new_nodes)
    ru.connect(rig, followed_local_pose, "result", op_invert2, "a")
    op_ident = ru.addNode(rig, f"ident", "Value<Matrix4>", new_nodes)
    ru.updateParms(rig, op_ident, { "parm": hou.Matrix4(1) })
    op_slerp = ru.addNode(rig, f"slerp", "transform::Slerp<Matrix4>", new_nodes)
    ru.updateParms(rig, op_slerp, { "bias": bias })
    ru.connect(rig, op_invert2, "result", op_slerp, "a")
    ru.connect(rig, op_ident, "value", op_slerp, "b")
    holder_new_rest_local = ru.addNode(rig, f"{comp_name}_{holder_name}_new_rest_local", "Multiply<Matrix4>", new_nodes)
    ru.connect(rig, holder_old_rest_local, "value", holder_new_rest_local, "a")
    ru.connect(rig, op_slerp, "result", holder_new_rest_local, "b")

    ru.connect(rig, holder_new_rest_local, "result", holder, "restlocal")

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

def preparePosedTransforms(rig: apex.Graph, skel: hou.Geometry, **kwargs):
    color = kwargs['nodecolor']

    def_skel_name = kwargs['packeddefskeleton']
    if def_skel_name == '':
        def_skel = skel
    else:
        def_skel = kwargs['other_geos'][def_skel_name]

    new_nodes = set()

    root_points = [p for p in def_skel.points() if not ku.getPointParent(p)]
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


def promoteTransformObjects(rig: apex.Graph, skel: hou.Geometry, **kwargs):
    specs = kwargs['specs']

    for spec in specs:
        pattern = spec['pattern#']
        action = spec['action#']
        t = spec['t#']
        r = spec['r#']
        s = spec['s#']
        tfos = rig.matchNodes(pattern)

        for tfo in tfos:
            if action == 'promote':
                ru.promoteTfo(rig, tfo, t, r, s)
            elif action == 'set':
                ru.promoteTfo(rig, tfo, t, r, s, demote=True)
            else:
                raise Exception(f"Unknown action: {action}")
    