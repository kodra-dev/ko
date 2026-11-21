"""Authoring for KoAdjust. All replay and geometry work happens in SOPs/VEX."""
import json
import uuid

import hou

TYPE = 'ko::koadjust::1.0'
ICON = 'opdef:/ko::Sop/koadjust::1.0?IconSVG'
STATE = 'ko_adjust'
PART = '__ko_adjust_part'
PART_ERROR = '__ko_adjust_part_error'
PART_KEY = '__ko_adjust_part_key'
HIDDEN = '__ko_adjust_hidden'
TOOLS = ('grab','clay','inflate','smooth','relax','flatten')
LABEL = dict(grab='Move', clay='Clay', inflate='Inflate', smooth='Smooth', relax='Relax', flatten='Planar')
STATE_CONTEXT = 'h.pane.gview.state.sop.' + STATE
MOVE_DIRECTION_ACTION = STATE_CONTEXT + '.toggle_move_direction'
MOVE_DIRECTION_KEY = 'Ctrl+Alt+M'
BASE = '__base'
KERNEL_VERSION = '4'
MAX_STAMPS = 96
LATEST = '__latest'
INFLUENCES = ('part','nearby','connected')
INFLUENCE_LABELS = ('One Part','Nearby in 3D','Topologically Connected')
DIRECTIONS = ('view','normal')
DIRECTION_LABELS = ('View Plane','Surface Normal')


def set_wrangle(node, code, detail=True):
    node.setParms({'class': 0 if detail else 2, 'snippet': code})


def make_adjust(source):
    if source.type().name() == TYPE:
        return source
    node = source.parent().createNode(TYPE, 'koadjust')
    node.setInput(0, source)
    node.moveToGoodPosition()
    return node


def history(node):
    tail=node.node('stack/OUT') if node is not None else None
    current=tail.input(0) if tail is not None else None
    result = []
    while current is not None and current.userData('ko_adjust_tool') in TOOLS:
        result.append(current)
        current = current.input(0)
    return list(reversed(result))


def status(stage):
    if stage.isBypassed():
        return ''
    geo = stage.geometry()
    if geo is None:
        return '; '.join(stage.errors()) or 'Stroke could not cook.'
    key = stage.userData('ko_adjust_status')
    return geo.attribValue(key) if key and geo.findGlobalAttrib(key) else ''


def active(node):
    edits = history(node)
    name = node.evalParm('active_stage')
    return next((s for s in edits if s.name() == name), edits[-1] if edits else None)


def stage_menu(kwargs):
    edits=history(kwargs['node'])
    latest=f'Latest · {len(edits)}. {LABEL[edits[-1].userData("ko_adjust_tool")]}' if edits else 'Latest'
    return ['', latest] + [
        s for i,stage in reversed(list(enumerate(edits,1)))
        for s in (stage.name(),f'{i}. {LABEL[stage.userData("ko_adjust_tool")]}{" (muted)" if stage.isBypassed() else ""}')
    ] + [BASE,'Before All Strokes']


def rewind_items(kwargs):
    items=stage_menu(kwargs)
    items[0]=LATEST
    return items


def set_rewind(node,stage):
    # Keep the menu a regular, editable token. The separate channel lets the
    # native graph cook without evaluating a Python-generated menu.
    parm=node.parm('rewind_to')
    token=stage or LATEST
    if parm is not None:
        if parm.keyframes():  # Remove the expression used by the old UI.
            parm.deleteAllKeyframes()
        if token not in parm.menuItems():
            token=LATEST
        parm.set(token)
    node.parm('active_stage').set('' if token==LATEST else token)


def sync_rewind(node):
    set_rewind(node,node.evalParm('active_stage'))


def rewind_menu_changed(kwargs):
    node=kwargs['node']
    token=kwargs['parm'].evalAsString()
    set_rewind(node,'' if token==LATEST else token)
    if hou.isUIAvailable():
        for pane in hou.ui.paneTabs():
            if (isinstance(pane,hou.SceneViewer) and pane.currentState()==STATE and pane.currentNode()==node):
                pane.runStateCommand('rewind_selection')


def choice(node,name):
    return node.parm(name).evalAsString()


def future(node):
    edits = history(node)
    name = node.evalParm('active_stage')
    if name == BASE:
        return edits
    for i, stage in enumerate(edits):
        if stage.name() == name:
            return edits[i+1:]
    return []


def brush_parts(stage):
    bind = stage.node('bind')
    geo = bind.geometry()
    weight = bind.evalParm('weight')
    if geo is None or not geo.findGlobalAttrib(weight+'_parts'):
        return set(), None
    return set(geo.attribValue(weight+'_parts')), geo.attribValue(weight+'_part')


def encode_samples(samples):
    return json.dumps({'samples':[float(f'{v:.9g}') for sample in samples for v in sample]}, separators=(',',':')) if samples else ''


def rebase_samples(bind,center,normal):
    parm=bind.parm('samples')
    if parm is None or not parm.eval():
        return
    old_center=hou.Vector3(bind.evalParmTuple('center'))
    old_normal=hou.Vector3(bind.evalParmTuple('normal'))
    rotation=old_normal.matrixToRotateTo(normal)
    data=json.loads(parm.eval())['samples']
    moved=[]
    for i in range(0,len(data),6):
        pos=(hou.Vector3(data[i:i+3])-old_center)*rotation+center
        n=hou.Vector3(data[i+3:i+6])*rotation
        moved.append((*pos,*n))
    parm.set(encode_samples(moved))


def hidden_parts(node):
    try:
        return set(json.loads(node.evalParm('view_hidden') or '{}'))
    except ValueError:
        return set()


def set_hidden_parts(node,names):
    with hou.undos.disabler():
        parm=node.parm('view_hidden')
        data=json.dumps(dict.fromkeys(sorted(names),1)) if names else ''
        if parm.eval()!=data:
            parm.set(data)


def editing_parts(node):
    parm = node.parm('editing_parts_data')
    if parm is None:
        return set()
    try:
        return set(json.loads(parm.eval() or '[]'))
    except ValueError:
        return set()


def parts(node):
    geo = node.node('choose_parts').geometry()
    attr = geo.findPrimAttrib(PART) if geo else None
    return sorted(set(attr.strings())) if attr is not None else []


def part_key(geo):
    return geo.attribValue(PART_KEY) if geo.findGlobalAttrib(PART_KEY) else ''


def add_controls(node, floats=(), strings=(), vectors=(), menus=()):
    ptg = node.parmTemplateGroup()
    for name, label, default in floats:
        ptg.append(hou.FloatParmTemplate(name, label, 1, default_value=(default,)))
    for name, label, default in strings:
        ptg.append(hou.StringParmTemplate(name, label, 1, default_value=(default,)))
    for name,label,tokens,labels,default in menus:
        ptg.append(hou.MenuParmTemplate(name,label,tokens,labels,default_value=tokens.index(default),
                                        store_default_value_as_string=True))
    for name, label, default in vectors:
        ptg.append(hou.FloatParmTemplate(name, label, 3, default_value=tuple(default), naming_scheme=hou.parmNamingScheme.XYZW))
    node.setParmTemplateGroup(ptg)


def update_saved_strokes(node):
    """Upgrade generated native kernels without changing their authored bindings."""
    edits = history(node)
    pending = [s for s in edits if s.userData('ko_adjust_kernel') in (None,'1','2')]
    if not pending:
        return 0
    sections = node.type().definition().sections()
    with hou.undos.group('Update KoAdjust Stroke Kernels'):
        if all(s.userData('ko_adjust_kernel') in (None,'1') for s in edits):
            # The old Review Edit picker never changed the cooked output.
            set_rewind(node,'')
        for stage in pending:
            bind, deform = stage.node('bind'), stage.node('deform')
            clean = stage.node('clean_weights')
            if bind is None or deform is None or clean is None:
                continue
            if stage.userData('ko_adjust_tool')=='grab' and deform.parm('direction') is None:
                add_controls(deform,strings=(('direction','Move Direction','view'),))
            if bind.parm('samples') is None:
                add_controls(bind,floats=(('falloff','Falloff',1.),),strings=(('samples','Stroke Samples',''),))
            if stage.userData('ko_adjust_tool')=='smooth' and deform.parm('smooth_span') is None:
                add_controls(deform,floats=(('smooth_span','Smooth Span',1.),))
            bind.parm('snippet').set(sections['BrushBindVEX'].contents())
            deform.parm('snippet').set(sections['BrushDeformVEX'].contents())
            weight = bind.evalParm('weight')
            _clean_brush_attrs(clean,weight)
            # Legacy scenes keep their authored field/blending behavior.
            stage.setUserData('ko_adjust_kernel','3')
    return len(pending)


def _clean_brush_attrs(node,weight):
    detail=' '.join(weight+'_'+suffix for suffix in ('center','normal','centers','normals','parts','part'))
    node.setParms({'ptdel':f'{weight} {weight}_stamp','dtldel':detail})


def append(node, tool, *, mode='part', anchor_part='', definition='', center=(0, 0, 0),
           normal=(0, 0, 1), radius=.25, branch_input=None, direction='view', falloff=1., brush_strength=1.):
    if tool not in TOOLS or mode not in ('part','nearby','connected'):
        raise ValueError('Unknown KoAdjust brush or influence mode.')
    stack = node.node('stack')
    tail = stack.node('OUT')
    source = tail.inputConnections()[0].inputItem()
    stage = stack.createNode('subnet', tool)
    try:
        token = uuid.uuid4().hex
        diagnostic, weight = 'ko_adjust_status_' + token, 'ko_adjust_weight_' + token
        stage.setUserData('ko_adjust_tool', tool)
        stage.setUserData('ko_adjust_kernel', KERNEL_VERSION)
        stage.setUserData('ko_adjust_status', diagnostic)
        stage.setInput(0, branch_input if branch_input is not None else source)
        bind = stage.createNode('attribwrangle', 'bind')
        bind.setInput(0, stage.indirectInputs()[0])
        add_controls(bind, floats=(('radius','Radius',radius), ('reach','Reprojection Distance',radius*.25),
                                   ('falloff','Falloff',falloff), ('brush_strength','Brush Strength',brush_strength)),
                     menus=(('mode','Affect',INFLUENCES,INFLUENCE_LABELS,mode),),
                     strings=(('anchor_part','Authored Named Part',anchor_part),
                              ('definition','Authored Part Definition',definition), ('samples','Stroke Samples',''),
                              ('weight','Internal Weight',weight), ('status','Internal Status',diagnostic)),
                     vectors=(('center','Anchor',center), ('normal','Anchor Normal',normal)))
        sections = node.type().definition().sections()
        set_wrangle(bind, sections['FastBrushBindVEX'].contents())
        deform = stage.createNode('attribwrangle', 'deform')
        deform.setInput(0, bind)
        add_controls(deform, floats=(('strength','Brush Amount',0.),)+
                                      ((('smooth_span','Smooth Span',3.),) if tool=='smooth' else ()),
                     vectors=(('delta','Displacement',(0,0,0)),),
                     strings=(('weight','Internal Weight',weight),('status','Internal Status',diagnostic)),
                     menus=(('tool','Brush',TOOLS,tuple(LABEL[t] for t in TOOLS),tool),)+
                           ((('direction','Move Direction',DIRECTIONS,DIRECTION_LABELS,direction),) if tool=='grab' else ()))
        set_wrangle(deform, sections['BrushDeformVEX'].contents(), False)
        if tool=='smooth':
            # The target is independent of cursor samples. Cook it once; the
            # fast point wrangle blends toward it only where the brush lands.
            target=stage.createNode('smooth::2.0','smooth_target')
            target.setInput(0,stage.indirectInputs()[0])
            target.setParms({'attributes':'P','method':2,'filterquality':1,'contrainedboundary':1,'updateaffectednmls':0})
            target.parm('strength').setExpression('max(0.1, ch("../deform/smooth_span") * 2)')
            deform.setInput(1,target)
        clean = stage.createNode('attribdelete', 'clean_weights')
        clean.setInput(0, deform)
        _clean_brush_attrs(clean,weight)
        out = stage.createNode('output', 'OUT')
        out.setInput(0, clean)
        out.setDisplayFlag(True)
        stage.layoutChildren()
        tail.setInput(0, stage)
        stage.moveToGoodPosition()
        tail.moveToGoodPosition()
        return stage
    except Exception:
        tail.setInput(0, source)
        stage.destroy()
        raise


def launch(kwargs, tool='grab'):
    viewer = kwargs.get('pane')
    if not isinstance(viewer, hou.SceneViewer):
        viewer = hou.ui.paneTabOfType(hou.paneTabType.SceneViewer)
    if viewer is None:
        return
    network = viewer.pwd()
    if network.childTypeCategory() != hou.sopNodeTypeCategory() or network.displayNode() is None:
        viewer.setPromptMessage('Display a SOP inside a Geometry object first.')
        return
    source = network.displayNode()
    try:
        with hou.undos.group('Enter KoAdjust'):
            node = make_adjust(source)
            node.parm('tool').set(tool)
            if node != source:
                size = source.geometry().boundingBox().sizevec().length()
                node.parm('radius').set(max(size * .09, .001))
            node.setDisplayFlag(True)
            if source.isRenderFlagSet():
                node.setRenderFlag(True)
            node.setCurrent(True, clear_all_selected=True)
            viewer.setCurrentNode(node, pick_node=False)
            if source == node and viewer.currentState() == STATE:
                viewer.runStateCommand('tool', tool)
            else:
                viewer.setCurrentState(STATE, generate=hou.stateGenerateMode.Enter)
    except (hou.Error, ValueError) as exc:
        viewer.setPromptMessage(str(exc))
