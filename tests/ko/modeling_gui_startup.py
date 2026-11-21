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
        assert any(set(key.lower().split('+'))=={'ctrl','alt','m'} for key in keys),(keys,conflicts)
        assert all(x.endswith('?'+a.MOVE_DIRECTION_ACTION) for x in conflicts),(keys,conflicts)
        node.updateParmStates()
        assert node.parm('move_direction').isVisible()
        result['checks'].append('radial activates locked KoAdjust HDA and state-only actions')
        # Run callbacks with synthetic rays, never desktop/global mouse input.
        state = s.State(a.STATE, viewer)
        state.onEnter({'node': node})
        def event(kind, x, *, y=0, shift=False, ctrl=False):
            return {'ui_event': SimpleNamespace(reason=lambda: kind,
                device=lambda: SimpleNamespace(isLeftButton=lambda: True, mouseX=lambda: int(x*100),mouseY=lambda:y,
                                               mouseWheel=lambda: 1,isShiftKey=lambda:shift,isCtrlKey=lambda:ctrl),
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
        assert sizes == [node.evalParm('radius')] and sizes[0] > radius_before and state.ring.visible()
        state._brush_ring = original_ring
        result['checks'].append('stationary wheel event resizes the visible brush ring immediately')
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
        direction_before=node.evalParm('move_direction')
        state.onMenuAction({'menu_item':'toggle_move_direction'})
        assert node.evalParm('move_direction')==direction_before
        state.onMenuAction({'menu_item':'nearby'})
        assert state.onMouseEvent(event(hou.uiEventReason.Picked,.61))
        assert len(a.history(node))==3
        dab=a.active(node)
        assert dab.userData('ko_adjust_tool')=='inflate' and dab.node('bind').evalParm('mode')=='nearby'
        assert dab.node('deform').evalParm('strength')>0 and not a.status(dab)
        assert state.onMouseEvent(event(hou.uiEventReason.Picked,.65,shift=True))
        assert a.active(node).userData('ko_adjust_tool')=='smooth' and node.evalParm('tool')=='inflate'
        state.onMenuAction({'menu_item':'grab'})
        node.updateParmStates()
        assert node.parm('move_direction').isVisible()
        state.onMenuAction({'menu_item':'toggle_move_direction'})
        assert node.evalParm('move_direction')=='normal'
        assert state.onMouseEvent(event(start,.27,y=100))
        assert state.onMouseEvent(event(changed,.27,y=136))
        motion=a.active(node).node('deform').evalParmTuple('delta')
        assert abs(motion[2])>.02 and abs(motion[0])<.005, motion
        result['checks'].append('Shift records Smooth temporarily; Surface Normal Move uses the normal, not the view plane')

        strokes=a.history(node)
        node.parm('active_stage').set(strokes[0].name())
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
            state.onMenuAction({'menu_item':'clay'})
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
            assert state.onMouseEvent(event(hou.uiEventReason.Picked,.31))
            assert len(a.history(node))==2 and a.history(node)[-1].userData('ko_adjust_tool')=='clay'
            assert not node.evalParm('active_stage') and len(questions)==2 and not state.approved_rewind
            hou.undos.performUndo()
            assert len(a.history(node))==5 and node.evalParm('active_stage')==strokes[0].name()
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
        other=a.make_adjust(grid)
        panel.bind(other)
        assert panel.model.names==a.parts(other)
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
