"""Offline Adjust integration test; use scripts/modeling_gui_test.ps1."""
import ctypes
from ctypes import wintypes
import json
import os
from pathlib import Path
from types import SimpleNamespace
import traceback

import hou
from hutil.Qt import QtCore, QtTest, QtWidgets


def desktop_name():
    user = ctypes.WinDLL('user32', use_last_error=True)
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    user.GetThreadDesktop.argtypes = [wintypes.DWORD]
    user.GetThreadDesktop.restype = wintypes.HANDLE
    user.GetUserObjectInformationW.argtypes = [wintypes.HANDLE, ctypes.c_int, wintypes.LPVOID,
                                               wintypes.DWORD, ctypes.POINTER(wintypes.DWORD)]
    kernel.GetCurrentThreadId.restype = wintypes.DWORD
    buffer, needed = ctypes.create_unicode_buffer(256), wintypes.DWORD()
    if not user.GetUserObjectInformationW(user.GetThreadDesktop(kernel.GetCurrentThreadId()), 2,
                                          buffer, ctypes.sizeof(buffer), ctypes.byref(needed)):
        raise ctypes.WinError(ctypes.get_last_error())
    return buffer.value


def run():
    result = {'desktop': desktop_name(), 'houdini': hou.applicationVersionString(), 'checks': []}
    try:
        import ko_adjust as a
        import ko_adjust_state as s
        assert result['desktop'].startswith('ko-modeling-test-')
        repo = Path(os.environ['KO_MODEL_TEST_REPO'])
        assert hou.hotkeys.loadKeymap('Ko', str(repo / 'Ko.keymap2'))
        assert hou.ui.radialMenu('ko_adjust') is not None
        obj = hou.node('/obj').createNode('geo', run_init_scripts=False)
        grid = obj.createNode('grid')
        grid.setParms({'orient': 0, 'sizex': 2, 'sizey': 2, 'rows': 24, 'cols': 24})
        front = obj.createNode('xform')
        front.setInput(0,grid)
        front.parm('tz').set(.4)
        source = obj.createNode('merge')
        source.setInput(0,grid)
        source.setInput(1,front)
        source.setDisplayFlag(True)
        viewer = hou.ui.paneTabOfType(hou.paneTabType.SceneViewer)
        viewer.setPwd(obj)
        viewer.setCurrentNode(source, pick_node=False)
        hou.setUpdateMode(hou.updateMode.AutoUpdate)
        data = json.loads((repo / 'radialmenu/ko_adjust.radialmenu').read_text())
        exec(data['items']['0']['script'], {'kwargs': {'pane': viewer}})
        node = obj.displayNode()
        assert node.type().name() == a.TYPE
        assert hou.ui.isRegisteredViewerState(a.STATE)
        assert viewer.currentState() == a.STATE
        live = {}
        viewer.runStateCommand('inspect', live)
        assert live['ready'], live
        assert not hou.hotkeys.assignments('h.pane.gview.state.sop.ko_adjust.review')
        keys=hou.hotkeys.assignments(a.STATE_CONTEXT,a.MOVE_DIRECTION_ACTION)
        conflicts=hou.hotkeys.findConflicts(a.STATE_CONTEXT,a.MOVE_DIRECTION_ACTION,a.MOVE_DIRECTION_KEY)
        assert keys
        if any(set(key.lower().split('+'))=={'ctrl','alt','m'} for key in keys):
            assert all(x.endswith('?'+a.MOVE_DIRECTION_ACTION) for x in conflicts),(keys,conflicts)
        node.updateParmStates()
        assert node.parm('move_direction').isVisible()
        result['checks'].append('radial activates locked KoAdjust HDA and state-only actions')
        # Run callbacks with synthetic rays, never desktop/global mouse input.
        state = s.State(a.STATE, viewer)
        state.onEnter({'node': node})
        def event(kind, x, *, y=0, shift=False, ctrl=False, middle=False):
            return {'ui_event': SimpleNamespace(reason=lambda: kind,
                device=lambda: SimpleNamespace(isLeftButton=lambda: not middle,isMiddleButton=lambda:middle,
                                               mouseX=lambda: int(x*100),mouseY=lambda:y,mouseWheel=lambda: 1,
                                               isShiftKey=lambda:shift,isCtrlKey=lambda:ctrl),
                ray=lambda: (hou.Vector3(x, .13, 2), hou.Vector3(0, 0, -1)))}
        start, active, changed = (hou.uiEventReason.Start, hou.uiEventReason.Active, hou.uiEventReason.Changed)
        sizes = []
        original_ring = state._brush_ring
        def resize_ring(center, normal, radius):
            sizes.append(radius)
            original_ring(center, normal, radius)
        state._brush_ring = resize_ring
        radius_before = node.evalParm('radius')
        assert state.onMouseWheelEvent(event(active, .07))  # No preceding mouse movement.
        assert sizes == [node.evalParm('radius')] and sizes[0] > radius_before
        assert state.ring.visible() and state.inner_ring.visible()
        assert state.onMouseWheelEvent(event(active,99))  # Broken/irrelevant wheel ray must not hide the ring.
        assert len(sizes)==2 and sizes[1]>sizes[0] and state.ring.visible()
        state._brush_ring = original_ring
        result['checks'].append('stationary wheel event resizes the visible brush ring immediately')
        base=node.evalParm('radius')
        assert state.onMouseEvent(event(start,.07,ctrl=True,shift=True))
        assert state.onMouseEvent(event(changed,.47,ctrl=True,shift=True))
        assert node.evalParm('radius')>base and not a.history(node)
        previous_inner,previous_width=state._ring_feedback
        assert state.onMouseEvent(event(start,.47,ctrl=True,shift=True,middle=True))
        assert state.onMouseEvent(event(changed,.07,y=30,ctrl=True,shift=True,middle=True))
        assert node.evalParm('strength')<1 and node.evalParm('falloff')>1 and not a.history(node)
        _,strength,falloff=state._brush_settings()
        assert abs(strength-.8)<1e-6 and abs(falloff-1.25)<1e-6
        inner,width=state._ring_feedback
        assert inner<previous_inner and width<previous_width and state.inner_ring.visible()
        hud=state._hud_template('smooth','Smooth')
        assert hud['icon']==a.ICON and len(hud['rows'])==6
        assert node.type().definition().icon()==a.ICON
        assert not hou.qt.Icon(a.ICON,24,24).isNull()
        assert hud['rows'][0]['key']=='mousewheel' and hud['rows'][1]['key']=='Ctrl Shift LMB'
        node.setParms({'radius':base,'strength':1,'falloff':1})
        result['checks'].append('native-style HUD and two-ring cursor respond to radius, strength and falloff without adding strokes')
        assert state._begin(event(start,.07)['ui_event'])
        assert state.onMouseEvent(event(active,.31))
        assert node.geometry().boundingBox().sizevec().x() == 2  # Local patch, not a whole-object move.
        assert a.active(node).node('deform').evalParm('deltax') > .2
        p0 = source.geometry().pointFloatAttribValues('P')
        assert node.geometry().pointFloatAttribValues('P') != p0
        assert state.wire.visible() and state.ghost.visible()
        assert a.editing_parts(node) and a.editing_parts(node) <= set(a.parts(node))
        assert node.node('ghost_geometry').geometry().primCount() == grid.geometry().primCount()
        before_resize = node.geometry().pointFloatAttribValues('P')
        assert state.onMouseWheelEvent(event(active,.31))
        assert a.active(node).node('bind').evalParm('radius') > .25
        assert node.geometry().pointFloatAttribValues('P') != before_resize
        state.onInterrupt({})
        assert not a.history(node) and not state.wire.visible() and not state.ghost.visible()
        assert not node.evalParm('view_focus')
        assert state.onMouseEvent(event(start,.07))
        assert state.onMouseEvent(event(changed,.31))
        assert len(a.history(node)) == 1
        hou.undos.performUndo()
        assert not a.history(node) and obj.displayNode() == node
        result['checks'].append('shaded SOP result updates during drag, before wireframe, wheel, cancel and one-step undo')
        picked = state._pick(state.pick_geometry(),event(start,.07)['ui_event'])
        assert abs(picked[0].z()-.4)<1e-4
        state.onMenuAction({'menu_item':'hide'})
        assert state.onMouseEvent(event(hou.uiEventReason.Picked,.07))
        assert state.hidden == {picked[2]}
        assert node.geometry().pointFloatAttribValues('P') == p0
        assert abs(state._pick(state.pick_geometry(),event(start,.07)['ui_event'])[0].z())<1e-4
        state.onMenuAction({'menu_item':'show_all'})
        assert abs(state._pick(state.pick_geometry(),event(start,.07)['ui_event'])[0].z()-.4)<1e-4
        state.onMenuAction({'menu_item':'solo'})
        assert state.onMouseEvent(event(hou.uiEventReason.Picked,.07))
        assert state.hidden == set(a.parts(node))-{picked[2]}
        state.onMenuAction({'menu_item':'show_all'})
        result['checks'].append('hide / isolate / show parts changes picking without changing the mesh')
        for x in (.07,-.37):
            assert state.onMouseEvent(event(start,x)) and state.onMouseEvent(event(changed,x+.18))
        assert len(a.history(node)) == 2 and len(obj.children()) == 4
        state.onMenuAction({'menu_item':'prev'})
        viewer.runStateCommand('review')
        viewer.runStateCommand('inspect', live)
        assert live['reviewing'] and live['handle_visible'], live
        stage = a.active(node)
        handle = dict(parms={})
        state.onStateToHandle(handle)
        assert handle['parms']['tx'] > .2
        handle['parms']['ty'] += .14
        state.onHandleToState(dict(handle, mod_parms=['ty']))
        assert a.active(node).node('deform').evalParm('deltay') > .1
        state.onMenuAction({'menu_item':'rebind'})
        assert state.onMouseEvent(event(hou.uiEventReason.Picked,.43))
        assert abs(stage.node('bind').evalParm('centerx')-.43) < 1e-4
        assert len(a.history(node)) == 2
        result['checks'].append('two gestures in one SOP; native handle and in-place rebind of older edit')
        state.onMenuAction({'menu_item':'latest'})
        state.onMenuAction({'menu_item':'inflate'})
        node.updateParmStates()
        assert not node.parm('move_direction').isVisible()
        direction_before=a.choice(node,'move_direction')
        state.onMenuAction({'menu_item':'toggle_move_direction'})
        assert a.choice(node,'move_direction')==direction_before
        state.onMenuAction({'menu_item':'nearby'})
        assert state.onMouseEvent(event(hou.uiEventReason.Picked,.61))
        assert len(a.history(node))==3
        dab=a.active(node)
        assert dab.userData('ko_adjust_tool')=='inflate' and a.choice(dab.node('bind'),'mode')=='nearby'
        assert dab.node('deform').evalParm('strength')>0 and not a.status(dab)
        assert state.onMouseEvent(event(hou.uiEventReason.Picked,.65,shift=True))
        assert a.active(node).userData('ko_adjust_tool')=='smooth' and a.choice(node,'tool')=='inflate'
        state.onMenuAction({'menu_item':'grab'})
        node.updateParmStates()
        assert node.parm('move_direction').isVisible()
        state.onMenuAction({'menu_item':'toggle_move_direction'})
        assert a.choice(node,'move_direction')=='normal'
        assert state.onMouseEvent(event(start,.27,y=100))
        assert state.onMouseEvent(event(changed,.27,y=136))
        moved=a.active(node)
        motion=hou.Vector3(moved.node('deform').evalParmTuple('delta'))
        normal=hou.Vector3(moved.node('bind').evalParmTuple('normal'))
        assert motion.length()>.02 and motion.cross(normal).length()<1e-5,motion
        result['checks'].append('Shift records Smooth temporarily; Surface Normal Move uses the normal, not the view plane')

        strokes=a.history(node)
        rewind=node.parm('rewind_to')
        assert not rewind.keyframes() and strokes[3].userData('ko_adjust_tool')=='smooth'
        rewind.set(strokes[3].name())
        rewind.pressButton()
        assert node.geometry().pointFloatAttribValues('P')==strokes[3].geometry().pointFloatAttribValues('P')
        rewind.set(strokes[0].name())
        rewind.pressButton()
        assert node.geometry().pointFloatAttribValues('P')==strokes[0].geometry().pointFloatAttribValues('P')
        assert len(a.history(node))==5
        old_message,old_post=hou.ui.displayMessage,hou.ui.postEventCallback
        posted=[]
        questions=[]
        reply=1
        def dialog(*args,**kwargs):
            questions.append(args[0])
            state.onInterrupt({})  # A modal native dialog interrupts the viewer state.
            return reply
        try:
            hou.ui.displayMessage=dialog
            hou.ui.postEventCallback=posted.append
            assert state.onMouseEvent(event(start,.31)) and not state.drag
            assert state.onMouseEvent(event(start,.31)) and not state.drag
            assert len(posted)==1 and not questions
            posted.pop(0)()
            assert len(questions)==1 and len(a.history(node))==5
            reply=0
            state.onMenuAction({'menu_item':'smooth'})
            assert state.onMouseEvent(event(start,.31)) and not state.drag
            posted.pop(0)()
            assert len(questions)==2 and state.approved_rewind
            # The modal stole the original drag. Only the next press starts it.
            assert state.onMouseEvent(event(start,.31)) and state.drag
            state.onInterrupt({})
            assert a.history(node)==strokes and node.evalParm('active_stage')==strokes[0].name()
            radius=node.evalParm('radius')
            node.parm('radius').set(.00001)  # Too small to reach any vertices on this patch.
            assert state.onMouseEvent(event(hou.uiEventReason.Picked,.31))
            assert a.history(node)==strokes and len(questions)==2
            node.parm('radius').set(radius)
            assert state.onMouseEvent(event(start,.31))
            assert state.onMouseEvent(event(active,.47))
            assert state.onMouseEvent(event(changed,.64))
            assert len(a.history(node))==2 and a.history(node)[-1].userData('ko_adjust_tool')=='smooth'
            assert a.history(node)[-1].node('bind').evalParm('samples')
            assert not node.evalParm('active_stage') and len(questions)==2 and not state.approved_rewind
            hou.undos.performUndo()
            assert len(a.history(node))==5 and node.evalParm('active_stage')==strokes[0].name()
            assert rewind.evalAsString()==strokes[0].name()
        finally:
            hou.ui.displayMessage,hou.ui.postEventCallback=old_message,old_post
        result['checks'].append('rewind confirmation survives native-style interruption and is not repeated')
        assert node.parm('part_rows') is None
        node.parm('open_parts').pressButton()
        from ko_adjust_parts import INTERFACE, open_panel
        pane = open_panel(node)
        QtWidgets.QApplication.processEvents()
        panel = pane.activeInterfaceRootWidget()
        assert panel is not None, pane.activeInterfaceScriptErrors()
        assert pane.activeInterface().name() == INTERFACE and panel.node == node, pane.activeInterfaceScriptErrors()
        assert pane.pane() is None and not pane.isShowingNetworkControls() and not pane.isToolbarShown()
        assert panel.model.names == a.parts(node) and panel.proxy.rowCount() == 2
        assert not panel.model.icons[0].isNull() and not panel.model.icons[1].isNull()
        eye = panel.proxy.index(0,1)
        rect = panel.table.visualRect(eye)
        assert rect.isValid() and rect.height() == panel.table.visualRect(panel.proxy.index(1,1)).height()
        def click_eye():
            QtTest.QTest.mouseClick(panel.table.viewport(),QtCore.Qt.LeftButton,pos=rect.center())
        click_eye()
        assert a.hidden_parts(node)=={panel.model.names[0]}
        assert panel.table.visualRect(eye)==rect
        panel.grab().save(str(Path(os.environ['KO_MODEL_TEST_RUN']) / 'parts-panel.png'))
        click_eye()
        assert not a.hidden_parts(node) and panel.table.visualRect(eye)==rect
        panel.filter.setText(panel.model.names[-1])
        assert panel.proxy.rowCount()==1
        panel._toggle(panel.proxy.index(0,1))
        assert a.hidden_parts(node)=={panel.model.names[-1]}
        panel.show_all.click()
        assert not a.hidden_parts(node)
        cover=obj.createNode('grid','partial_occluder')
        cover.setParms({'orient':0,'sizex':.7,'sizey':2,'rows':12,'cols':12})
        lifted=obj.createNode('xform','lift_cover')
        lifted.setInput(0,cover)
        lifted.setParms({'tx':.52,'tz':.35})
        layers=obj.createNode('merge','stacked_surfaces')
        layers.setInput(0,grid)
        layers.setInput(1,lifted)
        other=a.make_adjust(layers)
        panel.bind(other)
        assert panel.model.names==a.parts(other)
        painter=s.State(a.STATE,viewer)
        painter.onEnter({'node':other})
        other.setParms({'tool':'inflate','radius':.26})
        assert abs(painter._pick(painter.pick_geometry(),event(active,.58)['ui_event'])[0].z()-.35)<1e-4
        assert painter.onMouseEvent(event(start,-.57))
        assert painter.onMouseEvent(event(active,-.17))
        assert painter.onMouseEvent(event(changed,.58))
        strokes_paint=a.history(other)
        assert len(strokes_paint)==1 and not a.status(strokes_paint[0]),a.status(strokes_paint[0])
        assert len(json.loads(strokes_paint[0].node('bind').evalParm('samples'))['samples'])>24
        count=grid.geometry().pointCount()
        positions=other.geometry().points()
        assert any(p.position().x()>.51 and abs(p.position().z())>.02 for p in positions[:count])
        assert all(abs(p.position().z()-.35)<1e-4 for p in positions[count:])
        painter.onExit({})
        result['checks'].append('continuous drag paints a replayable path as one stroke')
        panel.bind(grid)
        assert panel.node is None and panel.proxy.rowCount()==0
        assert node.parm('update_strokes').description()=='Migrate Old Brush Data'
        result['checks'].append('chrome-free floating Parts list: fixed eye alignment, click, filter, Show All and node binding')
        # Full Houdini pane, unlike the standalone launcher, has a native tab strip.
        full=hou.ui.curDesktop().createFloatingPanel(hou.paneTabType.PythonPanel,
            python_panel_interface=INTERFACE,immediate=True)
        dockable=full.paneTabOfType(hou.paneTabType.PythonPanel)
        QtWidgets.QApplication.processEvents()
        holder=dockable.pane()
        widget=dockable.activeInterfaceRootWidget()
        assert widget._tab_bar is not None and not holder.isShowingPaneTabs(), (
            widget._tab_bar, dockable.isCurrentTab(), holder.isShowingPaneTabs(), dockable.activeInterfaceScriptErrors())
        was_shown=widget._tab_bar[1]
        sibling=holder.createTab(hou.paneTabType.Parm)
        sibling.setIsCurrentTab()
        QtWidgets.QApplication.processEvents()
        assert holder.isShowingPaneTabs()==was_shown
        full.close()
        result['checks'].append('Parts hides a host pane’s tabs only while active')
        state.onExit({})
        viewer.setCurrentState('select')
    except Exception:
        result['error'] = traceback.format_exc()
    (Path(os.environ['KO_MODEL_TEST_RUN']) / 'result.json').write_text(json.dumps(result), encoding='utf-8')
    def quit_when_idle():
        hou.ui.removeEventLoopCallback(quit_when_idle)
        hou.exit(exit_code=int('error' in result), suppress_save_prompt=True)
    hou.ui.addEventLoopCallback(quit_when_idle)


QtCore.QTimer.singleShot(3500, run)
