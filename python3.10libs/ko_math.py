import hou

def lerp(a, b, bias):
    if type(a) != type(b):
        raise ValueError(f"{a} and {b} are not the same type!")
    if isinstance(a, int) or isinstance(a, float):
        return a + (b - a) * bias;
    if isinstance(a, hou.Vector2):
        return hou.Vector2(a[0] + (b[0] - a[0]) * bias,
                           a[1] + (b[1] - a[1]) * bias)
    if isinstance(a, hou.Vector3):
        return hou.Vector3(a[0] + (b[0] - a[0]) * bias,
                           a[1] + (b[1] - a[1]) * bias,
                           a[2] + (b[2] - a[2]) * bias)
    if isinstance(a, hou.Vector4):
        return hou.Vector4(a[0] + (b[0] - a[0]) * bias,
                           a[1] + (b[1] - a[1]) * bias,
                           a[2] + (b[2] - a[2]) * bias,
                           a[3] + (b[3] - a[3]) * bias)
    raise ValueError(f"Can't lerp {a} and {b}")


def lerpMatrix4(a, b, biases):
    t_bias, r_bias, s_bias = biases
    a_srt = a.explode()
    a_xlt = a_srt['translate']
    a_rot = a_srt['rotate']
    a_rot_quat = hou.Quaternion()
    a_rot_quat.setToEulerRotates(a_rot)
    a_scl = a_srt['scale']
    b_srt = b.explode()
    b_xlt = b_srt['translate']
    b_rot = b_srt['rotate']
    b_rot_quat = hou.Quaternion()
    b_rot_quat.setToEulerRotates(b_rot)
    b_scl = b_srt['scale']

    new_xlt = lerp(a_xlt, b_xlt, t_bias)
    new_rot_quat = a_rot_quat.slerp(b_rot_quat, r_bias)
    new_scl = lerp(a_scl, b_scl, s_bias)
    new_rot = new_rot_quat.extractEulerRotates()

    return hou.hmath.buildTransform({
        'translate': new_xlt,
        'rotate': new_rot,
        'scale': new_scl
    })