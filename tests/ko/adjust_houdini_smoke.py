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
    assert node.geometry().findPrimGroup('_3d_hidden_primitives') is None
    node.parm('view_active').set(True)
    visible=node.geometry()
    assert topology(visible)==topo and visible.pointFloatAttribValues('P')==result_points
    hidden=node.node('viewport_mask').geometry().findPrimGroup(a.HIDDEN)
    assert hidden and hidden.primCount()==labels.geometry().primCount()/2
    rendered_mask=visible.findPrimGroup('_3d_hidden_primitives')
    assert rendered_mask and rendered_mask.primCount()==hidden.primCount()
    node.setParms({'view_active':False,'visibility_output':True})
    assert node.geometry().findPrimGroup('_3d_hidden_primitives').primCount()==hidden.primCount()
    node.setParms({'view_hidden':'','visibility_output':False})
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

    tiny=a.make_adjust(source)
    tiny.setParms({'part_source':'attribute','part_attribute':'part'})
    for _ in range(4):
        stroke=grab_fixture(tiny,source,'part')
        stroke.node('bind').parm('radius').set(1e-5)
        assert 'smaller than local spacing' in a.status(stroke), a.status(stroke)
    assert points_z(tiny.geometry())==points_z(source.geometry())
    warnings=tiny.warnings()
    assert len(warnings)==1 and '4 edits need attention' in warnings[0], warnings
    assert 'smaller than local spacing' in warnings[0], warnings

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
    assert rewind_items[1]=='Latest · 6. Smooth'
    assert rewind_items[3]=='6. Smooth' and '4. Relax' in rewind_items and '3. Planar' in rewind_items
    assert edits[-1].node('deform').parm('direction') is None
    saved=node.geometry().pointFloatAttribValues('P')
    rewind=node.parm('rewind_to')
    assert rewind.evalAsString()=='__latest' and rewind.menuLabels()[0]=='Latest · 6. Smooth'
    assert not rewind.keyframes()
    for name in ('tool','influence','part_source','move_direction'):
        assert isinstance(node.parm(name).parmTemplate(),hou.MenuParmTemplate)
    rewind.set(a.BASE)
    rewind.pressButton()
    assert node.evalParm('active_stage')==a.BASE and rewind.evalAsString()==a.BASE
    assert points_z(node.geometry())==points_z(source.geometry()) and len(a.history(node))==6
    a.set_rewind(node,grab.name())
    assert rewind.evalAsString()==grab.name()
    assert node.geometry().pointFloatAttribValues('P')==grab.geometry().pointFloatAttribValues('P')
    a.set_rewind(node,'')
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
    path_grid=grid.parent().createNode('grid','stroke_fixture')
    path_grid.setParms({'orient':0,'rows':35,'cols':35,'sizex':2,'sizey':2})
    path_node=a.make_adjust(path_grid)
    n=path_grid.geometry().prim(0).normal()
    path=a.append(path_node,'inflate',center=(-.6,.07,0),normal=n,radius=.25)
    path.node('deform').parm('strength').set(.15)
    path.node('bind').parm('samples').set(a.encode_samples([(-.6+.10*i,.07,0,*n) for i in range(1,13)]))
    for rows in (35,44):
        path_grid.parm('rows').set(rows)
        mesh=path_node.geometry()
        assert not a.status(path),a.status(path)
        assert any(p.position().x()>.55 and abs(p.position().z())>.06 for p in mesh.points())
        assert any(p.position().x()<-.55 and abs(p.position().z())>.06 for p in mesh.points())
        assert topology(mesh)==topology(path_grid.geometry())
    wave=grid.parent().createNode('attribwrangle','smooth_fixture')
    wave.setInput(0,path_grid)
    wave.parm('snippet').set('@P.z=.3*exp(-8*dot(@P,@P));')
    wg=wave.geometry()
    center=hou.Vector3()
    prim=wg.intersect(hou.Vector3(.02,.03,1),hou.Vector3(0,0,-1),center,hou.Vector3(),hou.Vector3())
    assert prim>=0
    smooth_node=a.make_adjust(wave)
    smooth=a.append(smooth_node,'smooth',center=center,normal=wg.prim(prim).normal(),radius=.6)
    params=smooth.node('deform')
    params.setParms({'strength':1,'smooth_span':1})
    narrow=smooth_node.geometry().boundingBox().maxvec().z()
    params.parm('smooth_span').set(3)
    assert narrow-smooth_node.geometry().boundingBox().maxvec().z()>.004

    def paths(geo):
        samples=[]
        for i in range(9):
            pos=hou.Vector3()
            hit=geo.intersect(hou.Vector3(-.36+.09*i,.03,1),hou.Vector3(0,0,-1),pos,hou.Vector3(),hou.Vector3())
            assert hit>=0
            samples.append((*pos,*geo.prim(hit).normal()))
        return samples,samples+samples[-2::-1]+samples[1:]+samples[-2::-1]

    uneven=grid.parent().createNode('attribwrangle','relax_fixture')
    uneven.setInput(0,path_grid)
    uneven.parm('snippet').set('@P.x+=.04*sin(14*@P.x);')
    for tool,brush_input in (('smooth',wave),('relax',uneven),('inflate',path_grid)):
        brush_node=a.make_adjust(brush_input)
        source_geo=brush_input.geometry()
        once,repeated=paths(source_geo)
        stage=a.append(brush_node,tool,center=once[0][:3],normal=once[0][3:],radius=.31)
        stage.node('deform').parm('strength').set(1. if tool!='inflate' else .09)
        source_positions=source_geo.pointFloatAttribValues('P')
        changes=[]
        for stroke in (once,repeated):
            stage.node('bind').parm('samples').set(a.encode_samples(stroke[1:]))
            out=brush_node.geometry()
            assert not a.status(stage),a.status(stage)
            changes.append(sum(abs(x-y) for x,y in zip(source_positions,out.pointFloatAttribValues('P'))))
        assert changes[1] > changes[0]*1.1,(tool,changes)
    assert all(n.type().name()!='python' for n in node.allSubChildren())
    with tempfile.TemporaryDirectory() as tmp:
        path=str(Path(tmp)/'brushes.hip')
        name=node.path()
        expected=node.geometry().pointFloatAttribValues('P')
        node.setParms({'view_hidden':'{"left":1}','view_active':True})
        assert node.geometry().findPrimGroup('_3d_hidden_primitives') is not None
        hou.hipFile.save(path)
        hou.hipFile.load(path,suppress_save_prompt=True)
        del sys.modules['ko_adjust']
        reopened=hou.node(name)
        assert len(a.history(reopened))==6 and reopened.geometry().pointFloatAttribValues('P')==expected
        assert not reopened.evalParm('view_active') and a.hidden_parts(reopened)=={'left'}
        assert reopened.geometry().findPrimGroup('_3d_hidden_primitives') is None
    print('PASS KoAdjust brushes, part modes, topology and replay',hou.applicationVersionString())


if __name__=='__main__':
    run()
