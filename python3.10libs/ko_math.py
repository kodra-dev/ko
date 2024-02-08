import hou
import math

def fit01(value, min, max):
    return (value - min) / (max - min)

def remap(value, old_min, old_max, new_min, new_max):
    return new_min + (value - old_min) * (new_max - new_min) / (old_max - old_min)

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

# Note that z is not negative z. It's more like maketransform than lookat in Vex
def lookat(z: hou.Vector3, y: hou.Vector3 = hou.Vector3(0, 1, 0), translate: hou.Vector3 = hou.Vector3(0, 0, 0)) -> hou.Matrix4:
    z = z.normalized()
    x = y.cross(z).normalized()
    y = z.cross(x).normalized()
    return hou.Matrix4((
        x[0], x[1], x[2], 0,
        y[0], y[1], y[2], 0,
        z[0], z[1], z[2], 0,
        translate[0], translate[1], translate[2], 1
    ))

def lookatAnyY(z: hou.Vector3, translate: hou.Vector3 = hou.Vector3(0, 0, 0)) -> hou.Matrix4:
    z = z.normalized()
    y = hou.Vector3(0, 1, 0)
    x = y.cross(z).normalized()
    if x.length() < 0.001:
        y = hou.Vector3(1, 0, 0)
    return lookat(z, y, translate)

def extendedLookat(forward: hou.Vector3, up: hou.Vector3,
                   local_forward: hou.Vector3 = hou.Vector3(0, 0, 1), local_up: hou.Vector3 = hou.Vector3(0, 1, 0),
                   translate: hou.Vector3 = hou.Vector3(0, 0, 0)) -> hou.Matrix4:
    forward = forward.normalized()
    up = up.normalized()
    local_forward = local_forward.normalized()
    local_up = local_up.normalized()
    lookat1 = lookat(forward, up, translate)
    lookat2 = lookat(local_forward, local_up)
    return lookat2.inverted() * lookat1

def randomColor():
    import random
    return hou.Color((random.random(), random.random(), random.random()))

def matrix3ToMatrix4(m: hou.Matrix3, translate: hou.Vector3) -> hou.Matrix4:
    return hou.Matrix4((
        m.at(0, 0), m.at(0, 1), m.at(0, 2), 0,
        m.at(1, 0), m.at(1, 1), m.at(1, 2), 0,
        m.at(2, 0), m.at(2, 1), m.at(2, 2), 0,
        translate[0], translate[1], translate[2], 1
    ))

def matrixIsClose(a: hou.Matrix4 | hou.Matrix3, b: hou.Matrix4 | hou.Matrix3) -> bool:
    if type(a) != type(b):
        return False
    if isinstance(a, hou.Matrix4):
        for i in range(4):
            for j in range(4):
                if not math.isclose(a.at(i, j), b.at(i, j), abs_tol=1e-6):
                    return False
        return True
    elif isinstance(a, hou.Matrix3):
        for i in range(3):
            for j in range(3):
                if not math.isclose(a.at(i, j), b.at(i, j), abs_tol=1e-6):
                    return False
        return True
    else:
        raise ValueError(f"Expected hou.Matrix3 or hou.Matrix4, got {type(a)}")