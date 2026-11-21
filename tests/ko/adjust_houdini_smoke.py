"""22.0.466 native KoAdjust replay and part isolation test."""
import json
from pathlib import Path
import sys
import tempfile

import hou

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'python3.13libs'))
import ko_adjust as a


def graph():
    obj = hou.node('/obj').createNode('geo',run_init_scripts=False)
    grid = obj.createNode('grid')
    grid.setParms(dict(orient=0,rows=29,cols=29,sizex=2,sizey=2))
    labels = obj.createNode('attribwrangle','labels')
    labels.setInput(0,grid)
    labels.setParms({'class':1,'snippet': '''
vector p=primuv(0,"P",@primnum,{0.5,0.5,0});
s@part = p.x<0 ? "left" : "right";
i@part_id = p.x<0 ? 7 : 11;
'''})
    move = obj.createNode('xform','other_shell')
    move.setInput(0,labels)
    move.parm('tz').set(.16)
    offset = obj.createNode('attribwrangle','other_name')
    offset.setInput(0,move)
    offset.setParms({'class':1,'snippet':'s@part="other"; i@part_id=23;'})
    merged = obj.createNode('merge')
    merged.setInput(0,labels)
    merged.setInput(1,offset)
    return grid,labels,merged


def points_z(geo):
    return list(geo.pointFloatAttribValues('P')[2::3])


def topology(geo):
    assert geo is not None
    return geo.pointCount(), tuple(tuple(p.number() for p in f.points()) for f in geo.prims())


def grab_fixture(a_node, source, mode, tool='grab'):
    if tool == 'grab':
        pos, normal = (-.115,.121,0), source.geometry().prim(0).normal()
    else:
        geo = a_node.node('stack/OUT').geometry()
        pos, uv = hou.Vector3(), hou.Vector3()
        hit = geo.intersect(hou.Vector3(-.115,.121,-1),hou.Vector3(0,0,1),pos,hou.Vector3(),uv,pattern='@part=left')
        assert hit >= 0
        normal = geo.prim(hit).normal()
    stage=a.append(a_node,tool,mode=mode,anchor_part='left',definition=a.part_key(a_node.node('choose_parts').geometry()),
                   center=pos,normal=normal,radius=.46)
    deform=stage.node('deform')
    deform.parmTuple('delta').set((0,0,.28))
    deform.parm('strength').set(.06 if tool in ('inflate','clay') else .4)
    return stage


def run():
    hou.hda.installFile(str(ROOT/'otls/sop_ko.koadjust.1.0.hda'))
    grid, labels, source=graph()
    info=source.geometry()
    original=points_z(info)
    topo=topology(info)
    right={p.number() for p in labels.geometry().points() if p.position().x()>.001}
    seam={p.number() for p in labels.geometry().points() if abs(p.position().x())<.001}
    back={p.number() for p in info.points() if p.position().z()>.1}
    nodes={}
    for mode in ('part','nearby','connected'):
        node=a.make_adjust(source)
        node.setParms({'part_source':'attribute','part_attribute':'part'})
        assert a.parts(node)==['left','other','right']
        stage=grab_fixture(node,source,mode)
        geo=node.geometry()
        assert not node.errors() and not a.status(stage),(node.errors(),a.status(stage))
        assert topology(geo)==topo
        assert geo.findPrimAttrib(a.PART) is None
        zs=points_z(geo)
        changed={i for i,z in enumerate(zs) if abs(z-original[i])>1e-6}
        assert changed
        assert bool(changed & right)==(mode!='part')
        assert bool(changed & seam)==(mode!='part')
        assert bool(changed & back)==(mode=='nearby')
        nodes[mode]=(node,stage)

    node, grab=nodes['part']
    authored = grab.node('bind').evalParmTuple('center'), grab.node('deform').evalParmTuple('delta')
    grab.destroyUserData('ko_adjust_kernel')
    grab.node('bind').parm('snippet').set('')
    assert a.update_saved_strokes(node)==1 and a.update_saved_strokes(node)==0
    assert authored==(grab.node('bind').evalParmTuple('center'),grab.node('deform').evalParmTuple('delta'))
    result_points=node.geometry().pointFloatAttribValues('P')
    node.parm('view_hidden').set(json.dumps({'left':1},ensure_ascii=False))
    visible=node.geometry()
    assert topology(visible)==topo and visible.pointFloatAttribValues('P')==result_points
    hidden=node.node('viewport_mask').geometry().findPrimGroup(a.HIDDEN)
    assert hidden and hidden.primCount()==labels.geometry().primCount()/2
    rendered_mask=visible.findPrimGroup('_3d_hidden_primitives')
    assert rendered_mask and rendered_mask.primCount()==hidden.primCount()
    node.parm('view_hidden').set('')
    grid.setParms(dict(rows=39,cols=37))
    assert topology(node.geometry())==topology(source.geometry()) and not a.status(grab)
    assert max(points_z(node.geometry()))>.2
    node.parm('part_attribute').set('part_id')
    assert a.parts(node)==['11','23','7']
    assert 'definition changed' in a.status(grab)
    node.geometry()
    assert any('definition changed' in w for w in node.warnings())
    node.parm('part_attribute').set('missing')
    assert not a.parts(node) and 'primitive attribute' in a.status(grab)
    node.parm('part_attribute').set('part')
    grab.node('bind').parm('anchor_part').set('not_a_part')
    assert 'rebind' in a.status(grab) and points_z(node.geometry())==points_z(source.geometry())
    grab.node('bind').parm('anchor_part').set('left')
    assert not a.status(grab)

    connectivity=a.make_adjust(source)
    assert len(a.parts(connectivity))==2
    # UV seam along the center of a connected grid, created on vertices.
    uv=grid.parent().createNode('attribwrangle','uv_seam')
    uv.setInput(0,labels)
    uv.setParms({'class':3,'snippet': 'vector p=point(0,"P",@ptnum); vector shift=prim(0,"part_id",@primnum)==7 ? {0,0,0} : {2,0,0}; v@uv=p+shift;'})
    islands=a.make_adjust(uv)
    islands.parm('part_source').set('uv')
    assert len(a.parts(islands))==2,(a.parts(islands),islands.errors(),islands.warnings())
    assert topology(islands.geometry())==topology(grid.geometry())
    stacked=grid.parent().createNode('merge')
    stacked.setInput(0,uv)
    stacked.setInput(1,uv)
    islands.setInput(0,stacked)
    assert len(a.parts(islands))==4, a.parts(islands)
    islands.parm('uv_attribute').set('missing')
    islands.geometry()
    assert not a.parts(islands) and any('vertex UV' in x for x in islands.warnings())

    for tool in ('inflate','flatten','relax','clay','smooth'):
        before=node.geometry().pointFloatAttribValues('P')
        stage=grab_fixture(node,source,'part',tool)
        assert topology(node.geometry())==topology(source.geometry()) and not a.status(stage), (tool,a.status(stage),node.errors())
        assert node.geometry().pointFloatAttribValues('P')!=before,tool
    edits=a.history(node)
    assert len(edits)==6
    rewind_items=a.stage_menu({'node':node})
    assert rewind_items[::2]==['']+[stage.name() for stage in reversed(edits)]+[a.BASE]
    assert rewind_items[3]=='6. Smooth' and '4. Relax' in rewind_items and '3. Planar' in rewind_items
    assert edits[-1].node('deform').parm('direction') is None
    saved=node.geometry().pointFloatAttribValues('P')
    node.parm('active_stage').set(a.BASE)
    assert points_z(node.geometry())==points_z(source.geometry()) and len(a.history(node))==6
    node.parm('active_stage').set(grab.name())
    assert node.geometry().pointFloatAttribValues('P')==grab.geometry().pointFloatAttribValues('P')
    node.parm('active_stage').set('')
    assert node.geometry().pointFloatAttribValues('P')==saved

    # Even when both sheets are within the search band, the clicked sheet at
    # zero projection error wins. Nearby expands the brush, not the anchor.
    close_grid,_,close=graph()
    close.parent().node('other_shell').parm('tz').set(.025)
    for mode in ('part','connected','nearby'):
        near=a.make_adjust(close)
        edit=grab_fixture(near,close,mode)
        result=points_z(near.geometry())
        assert not a.status(edit),(mode,a.status(edit))
        assert max(result[:close_grid.geometry().pointCount()])>.22
        assert (max(result[close_grid.geometry().pointCount():])>.04)==(mode=='nearby')
    close.parent().node('other_shell').parm('tz').set(0)
    for mode in ('part','nearby'):
        near=a.make_adjust(close)
        edit=grab_fixture(near,close,mode)
        result=points_z(near.geometry())
        if mode=='nearby':
            assert not a.status(edit) and max(result)>.22
        else:
            assert 'equally plausible' in a.status(edit) and max(result)==0
    wide=grid.parent().createNode('attribwrangle','make_many_part_labels')
    wide.setInput(0,grid)
    wide.setParms({'class':1,'snippet':'i@part_id=@primnum;'})
    list_node=a.make_adjust(wide)
    list_node.setParms({'part_source':'attribute','part_attribute':'part_id'})
    assert len(a.parts(list_node))==grid.geometry().primCount()
    assert list_node.parm('part_rows') is None
    assert all(n.type().name()!='python' for n in node.allSubChildren())
    with tempfile.TemporaryDirectory() as tmp:
        path=str(Path(tmp)/'brushes.hip')
        name=node.path()
        expected=node.geometry().pointFloatAttribValues('P')
        hou.hipFile.save(path)
        hou.hipFile.load(path,suppress_save_prompt=True)
        del sys.modules['ko_adjust']
        reopened=hou.node(name)
        assert len(a.history(reopened))==6 and reopened.geometry().pointFloatAttribValues('P')==expected
    print('PASS KoAdjust brushes, part modes, topology and replay',hou.applicationVersionString())


if __name__=='__main__':
    run()
