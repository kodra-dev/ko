"""Authoring for KoAdjust. All replay and geometry work happens in SOPs/VEX."""
import json
import uuid

import hou

TYPE = 'ko::koadjust::1.0'
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
KERNEL_VERSION = '2'


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
    current = node.node('stack/OUT').input(0)
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
    return ['', 'Latest'] + [
        s for i,stage in reversed(list(enumerate(edits,1)))
        for s in (stage.name(),f'{i}. {LABEL[stage.userData("ko_adjust_tool")]}{" (muted)" if stage.isBypassed() else ""}')
    ] + [BASE,'Before All Strokes']


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


def add_controls(node, floats=(), strings=(), vectors=()):
    ptg = node.parmTemplateGroup()
    for name, label, default in floats:
        ptg.append(hou.FloatParmTemplate(name, label, 1, default_value=(default,)))
    for name, label, default in strings:
        ptg.append(hou.StringParmTemplate(name, label, 1, default_value=(default,)))
    for name, label, default in vectors:
        ptg.append(hou.FloatParmTemplate(name, label, 3, default_value=tuple(default), naming_scheme=hou.parmNamingScheme.XYZW))
    node.setParmTemplateGroup(ptg)


def update_saved_strokes(node):
    """Upgrade generated native kernels without changing their authored bindings."""
    edits = history(node)
    pending = [s for s in edits if s.userData('ko_adjust_kernel') in (None,'1')]
    if not pending:
        return 0
    sections = node.type().definition().sections()
    with hou.undos.group('Update KoAdjust Stroke Kernels'):
        if len(pending)==len(edits):
            # The old Review Edit picker never changed the cooked output.
            node.parm('active_stage').set('')
        for stage in pending:
            bind, deform = stage.node('bind'), stage.node('deform')
            clean = stage.node('clean_weights')
            if bind is None or deform is None or clean is None:
                continue
            if stage.userData('ko_adjust_tool')=='grab' and deform.parm('direction') is None:
                add_controls(deform,strings=(('direction','Move Direction','view'),))
            bind.parm('snippet').set(sections['BrushBindVEX'].contents())
            deform.parm('snippet').set(sections['BrushDeformVEX'].contents())
            weight = bind.evalParm('weight')
            clean.parm('dtldel').set(f'{weight}_center {weight}_normal {weight}_parts {weight}_part')
            stage.setUserData('ko_adjust_kernel',KERNEL_VERSION)
    return len(pending)


def append(node, tool, *, mode='part', anchor_part='', definition='', center=(0, 0, 0),
           normal=(0, 0, 1), radius=.25, branch_input=None, direction='view'):
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
        add_controls(bind, floats=(('radius','Radius',radius), ('reach','Reprojection Distance',radius*.25)),
                     strings=(('mode','Influence',mode), ('anchor_part','Authored Named Part',anchor_part),
                              ('definition','Authored Part Definition',definition),
                              ('weight','Internal Weight',weight), ('status','Internal Status',diagnostic)),
                     vectors=(('center','Anchor',center), ('normal','Anchor Normal',normal)))
        sections = node.type().definition().sections()
        set_wrangle(bind, sections['BrushBindVEX'].contents())
        deform = stage.createNode('attribwrangle', 'deform')
        deform.setInput(0, bind)
        add_controls(deform, floats=(('strength','Brush Amount',0.),),
                     vectors=(('delta','Displacement',(0,0,0)),),
                     strings=(('tool','Brush',tool), ('weight','Internal Weight',weight),
                              ('status','Internal Status',diagnostic)) +
                             ((('direction','Move Direction',direction),) if tool=='grab' else ()))
        set_wrangle(deform, sections['BrushDeformVEX'].contents(), False)
        clean = stage.createNode('attribdelete', 'clean_weights')
        clean.setInput(0, deform)
        clean.setParms({'ptdel':weight, 'dtldel':f'{weight}_center {weight}_normal {weight}_parts {weight}_part'})
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
