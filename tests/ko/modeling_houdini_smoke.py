"""Offline test: run with Houdini's hython, not system Python."""
from pathlib import Path
import sys
import tempfile

import hou

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'python3.13libs'))
import ko_model_workbench as m


def mesh(node, rows, *, height=0., sleeve=None, scope='arm'):
    code = f'''
import hou
geo = hou.pwd().geometry()
g = geo.createPrimGroup({scope!r})
n = {rows}
for z in {([height] if sleeve is None else [height, sleeve])!r}:
    points = geo.createPoints(tuple((-1 + 2*i/n, -1 + 2*j/n, z) for j in range(n+1) for i in range(n+1)))
    for j in range(n):
        for i in range(n):
            k = j*(n+1)+i
            f = geo.createPolygon()
            for a in (k,k+1,k+n+2,k+n+1):
                f.addVertex(points[a])
            g.add(f)
'''
    node.parm('python').set(code)


def checked(node):
    geo = node.geometry()
    assert not node.errors() and not node.warnings(), (node.path(), node.errors(), node.warnings())
    return geo


def failed(node, source, message):
    geo = node.geometry()
    assert not node.errors() and any(message in w for w in node.warnings()), (node.errors(), node.warnings())
    assert m.exact_signature(geo) == m.exact_signature(source.geometry())


def run():
    obj = hou.node('/obj').createNode('geo')
    src = obj.createNode('python')
    mesh(src, 8)
    geo = checked(src)
    p = hou.Vector3(.07, .09, 0.)
    hit, pos = m._ray(geo, p + hou.Vector3(0, 0, 1), hou.Vector3(0, 0, -1), 'arm', 0., 2., 1e-6)
    spec = m.make_anchor(geo, hit, pos, .8)
    grab = m.create_operation(src, 'grab', spec, group='arm', radius=.8, delta=(0, 0, .2))
    assert checked(grab).boundingBox().maxvec().z() > .15
    mesh(src, 13, height=.02)
    assert checked(grab).pointCount() == 196
    assert checked(grab).boundingBox().maxvec().z() > .17
    mesh(src, 13, height=.02, sleeve=.03)
    failed(grab, src, 'multiple surfaces')
    mesh(src, 13, sleeve=0.)
    failed(grab, src, 'multiple surfaces')
    mesh(src, 13, scope='not_arm')
    failed(grab, src, 'scope')
    mesh(src, 13)
    point = m.create_operation(src, 'point', spec, group='arm', amount=.1)
    point_geo = checked(point)
    assert point_geo.pointCount() == 1 and abs(point_geo.point(0).position().z() - spec['normal'][2]*.1) < 1e-5
    assert point_geo.findPointAttrib('orient') is not None
    point.parm('ko_group').set('nonexistent')
    assert point.geometry().pointCount() == 0 and point.warnings()
    uid, delta = grab.evalParm('ko_uid'), grab.evalParmTuple('ko_delta')
    geo = src.geometry()
    hit, pos = m._ray(geo, hou.Vector3(-.37, .24, 1), hou.Vector3(0, 0, -1), 'arm', 0., 2., 1e-6)
    m.rebind_grab(grab, src, m.make_anchor(geo, hit, pos, .8), 'arm')
    assert grab.evalParm('ko_uid') == uid and grab.evalParmTuple('ko_delta') == delta
    assert checked(grab).boundingBox().maxvec().z() > .15

    # Generated rim is an edge role, not a component number snapshot.
    region = {'kind': 'group', 'group': 'arm'}
    ext = m.create_operation(src, 'extrude', region, amount=.2)
    cap, side, rim = m.feature_groups(ext.evalParm('ko_uid'))
    g = checked(ext)
    assert g.findPrimGroup(cap) and g.findPrimGroup(side) and g.findEdgeGroup(rim)
    bevel = m.create_operation(ext, 'bevel', {'kind': 'group', 'group': rim}, amount=.04, divisions=2)
    assert checked(bevel).primCount() > g.primCount()
    mesh(src, 5)
    assert checked(ext).findPrimGroup(cap).primCount() == 25
    checked(bevel)
    mesh(src, 5, scope='not_arm')
    failed(ext, src, 'missing or empty')
    failed(bevel, ext, 'missing or empty')
    mesh(src, 5)
    geo = checked(src)
    exact = m.capture_selection(geo, 'prim', hou.Selection(geo, hou.geometryType.Primitives, '0'))
    assert exact['kind'] == 'exact'
    inset = m.create_operation(src, 'inset', exact, amount=.04)
    checked(inset)
    mesh(src, 6)
    failed(inset, src, 'Exact selection input changed')
    checked(m.create_operation(src, 'smooth', region, amount=.1))

    ring = obj.createNode('python')
    for n in (8, 7):
        ring.parm('python').set(f'''
import math, hou
geo = hou.pwd().geometry()
n = {n}
ps = geo.createPoints(tuple((r*math.cos(i*math.tau/n), r*math.sin(i*math.tau/n), 0) for r in (1,2) for i in range(n)))
for i in range(n):
    j = (i+1)%n
    f = geo.createPolygon()
    for k in (j,i,i+n,j+n):
        f.addVertex(ps[k])
g = geo.createEdgeGroup('hole')
for i in range(n):
    g.add(geo.findEdge(ps[i],ps[(i+1)%n]))
''')
        if n == 8:
            fill = m.create_operation(ring, 'fill', {'kind': 'group', 'group': 'hole'})
            assert checked(fill).primCount() == 12
        else:
            failed(fill, ring, 'even, closed boundary')
    mesh(src, 5)
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / 'modeling.hip')
        grab_path, bevel_path = grab.path(), bevel.path()
        hou.hipFile.save(path)
        hou.hipFile.load(path, suppress_save_prompt=True)
        assert checked(hou.node(grab_path)).boundingBox().maxvec().z() > .1
        checked(hou.node(bevel_path))
    print('PASS modeling smoke:', hou.applicationVersionString())


if __name__ == '__main__':
    run()
