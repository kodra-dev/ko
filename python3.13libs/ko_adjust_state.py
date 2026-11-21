"""KoAdjust viewer gestures and temporary part visibility."""
import json
import math
import time

import hou

import ko_adjust as a

MODES = {'part':'One Part','nearby':'Nearby in 3D','connected':'Topologically Connected'}


class State:
    def __init__(self, state_name, scene_viewer):
        self.viewer = scene_viewer
        self.node = self.handle = self.ring = self.wire = self.ghost = None
        self.drag = self.pick_action = None
        self.review = self.handle_undo = False
        self.hidden = set()
        self.last_hover = 0.
        self.hover = None
        self.pending_rewind = self.approved_rewind = None

    def onEnter(self, kwargs):
        self.node = kwargs['node']
        a.update_saved_strokes(self.node)
        self.handle = hou.Handle(self.viewer, 'grab_move')
        self.ring = hou.GeometryDrawable(self.viewer, hou.drawableGeometryType.Line, 'ko_adjust_radius')
        self.ring.setParams({'color1':(.99,.73,.25,1.),'line_width':2.})
        self.circle = hou.Geometry()
        circle = self.circle.createPolygon()
        for point in self.circle.createPoints(tuple((math.cos(i*math.tau/64),math.sin(i*math.tau/64),0.) for i in range(64))):
            circle.addVertex(point)
        self.ring.setGeometry(self.circle)
        self.wire = hou.GeometryDrawable(self.viewer,hou.drawableGeometryType.Line,'ko_adjust_before')
        self.wire.setParams({'color1':(.54,.75,.85,.22),'line_width':1.,'fade_factor':.2})
        self.ghost = hou.GeometryDrawable(self.viewer,hou.drawableGeometryType.Face,'ko_adjust_other_parts')
        self.ghost.setParams({'color1':(.54,.59,.65,.19),'fade_factor':.2})
        self._set_focus(set())
        self.set_hidden(a.hidden_parts(self.node))
        self.refresh()

    def onGenerate(self, kwargs):
        node = kwargs.get('node') or self.viewer.currentNode()
        if node is not None and node.type().name() == a.TYPE:
            self.onEnter(dict(kwargs, node=node))

    def set_hidden(self, names):
        if self.node is None:
            return
        self.hidden = set(names)
        a.set_hidden_parts(self.node,self.hidden)
        self.visibility_revision = self._part_revision()

    def _part_revision(self):
        geo = self.node.node('choose_parts').geometry()
        if geo is None:
            return None
        attr = geo.findPrimAttrib(a.PART)
        return (a.part_key(geo), geo.topologyDataId(), attr.dataId() if attr else None)

    def _expire_visibility(self):
        external = a.hidden_parts(self.node)
        if external!=self.hidden:
            self.hidden = external
            self.visibility_revision = self._part_revision()
        elif self.hidden and self._part_revision()!=self.visibility_revision:
            self.set_hidden(set())

    def _set_focus(self,parts):
        with hou.undos.disabler():
            self.node.parm('view_focus').set(json.dumps(dict.fromkeys(sorted(parts),1)) if parts else '')

    def _editing(self,names=()):
        with hou.undos.disabler():
            parm=self.node.parm('editing_parts_data')
            data=json.dumps(sorted(names)) if names else ''
            if parm is not None and parm.eval()!=data:
                parm.set(data)

    def _preview_parts(self,stage,update_wire=True):
        affected, picked = a.brush_parts(stage)
        all_parts = set(a.parts(self.node))
        others = all_parts-self.hidden-affected if affected else set()
        self._set_focus(affected if others else set())
        self._editing(affected or ({picked} if picked is not None else set()))
        self.ghost.show(False)
        if others:
            self.ghost_mesh=self.node.node('ghost_geometry').geometry().freeze(read_only=True)
            self.ghost.setGeometry(self.ghost_mesh)
            self.ghost.show(True)
        if update_wire and self.node.evalParm('wire'):
            source=self.node.node('visible_before') if others or self.hidden else self.node.node('view_source')
            self._original(source.geometry().freeze(read_only=True))

    def pick_geometry(self):
        self._expire_visibility()
        return self.node.node('viewport_mask' if self.hidden else 'view_source').geometry()

    def _grab(self):
        stage = a.active(self.node) if self.node and self.review and not self.pick_action else None
        return (stage.node('deform') if stage and stage.userData('ko_adjust_tool') == 'grab' and not stage.isBypassed()
                else None)

    def refresh(self, message=''):
        if self.node is None:
            return
        tool = self.node.evalParm('tool')
        stage = a.active(self.node)
        issue = a.status(stage) if stage and not self.drag else ''
        if self.pick_action:
            usage = dict(rebind='Click a new anchor', hide='Click a part to hide', solo='Click a part to isolate')[self.pick_action] + ' · Esc cancels'
        elif self.review:
            usage = 'Drag mesh: new brush from here · Wheel: reviewed radius · Move gizmo when available'
        else:
            usage = 'LMB brush · Shift: Smooth · Wheel: radius · RMB: controls'
        self.viewer.setPromptMessage(message or f'KoAdjust · {issue or usage}')
        stages = a.history(self.node)
        mode = stage.node('bind').evalParm('mode') if self.review and stage else self.node.evalParm('influence')
        label = f'{stages.index(stage)+1}/{len(stages)} · {a.LABEL[stage.userData("ko_adjust_tool")]}' if self.review and stage else a.LABEL[tool]
        rows = [{'label':'Review' if self.review else 'Brush','value':label},
                {'label':'Affect','value':MODES[mode]}, {'label':'Hidden parts','value':str(len(self.hidden))},
                {'label':'Brushes / Parts / History','key':'RMB'}]
        if not self.review and tool=='grab':
            direction='Surface Normal' if self.node.evalParm('move_direction')=='normal' else 'View Plane'
            row={'label':'Move Direction','value':direction}
            keys=hou.hotkeys.assignments(a.STATE_CONTEXT,a.MOVE_DIRECTION_ACTION)
            if keys:
                row['key']=', '.join(keys)
            rows.append(row)
        if issue:
            rows.append({'label':'Needs repair','value':issue})
        self.viewer.hudInfo(show=True, template={'title':'KoAdjust','icon':'SOP_edit','rows':rows}, panel=hou.hudPanel.ToolInfo)
        self.handle.show(self._grab() is not None and self.drag is None)
        if self._grab() is not None:
            self.handle.update()

    def onDraw(self, kwargs):
        for drawable in (self.ghost,self.wire,self.ring):
            if drawable and drawable.visible():
                drawable.draw(kwargs['draw_handle'])

    def _original(self, geo):
        self.before = geo
        self.wire.setGeometry(geo)
        self.wire.show(bool(self.node.evalParm('wire')))

    def _brush_ring(self, center, normal, radius):
        rotation = hou.Vector3(0,0,1).matrixToRotateTo(normal)
        self.ring.setTransform(hou.hmath.buildScale((radius,)*3) * rotation * hou.hmath.buildTranslate(center))
        self.ring.show(True)

    def _pick(self, geo, event, *, visible=True):
        if geo is None or not geo.findPrimAttrib(a.PART):
            raise ValueError('Parts are unavailable. Check Define Parts By and the input attribute.')
        o, d = event.ray()
        pos, n, uv = hou.Vector3(), hou.Vector3(), hou.Vector3()
        part = geo.findPrimAttrib(a.PART)
        # Use the native group on the live preview. For a reviewed stage, the
        # temporary group isn't in its history, so skip hidden hits by name.
        hidden_group = visible and self.hidden and geo.findPrimGroup(a.HIDDEN)
        pattern = '* ^' + a.HIDDEN if hidden_group else '*'
        for _ in range(64):
            i = geo.intersect(o,d,pos,n,uv,pattern=pattern,min_hit=0.,tolerance=1e-6)
            if i < 0:
                return None
            prim = geo.prim(i)
            if not isinstance(prim, hou.Polygon) or not prim.isClosed():
                raise ValueError('Pick a closed polygon (unpack packed surfaces before brushing).')
            name = prim.attribValue(part)
            if not visible or name not in self.hidden:
                return pos, prim.normal(), name, a.part_key(geo)
            pattern += f' ^{i}'
        return None

    def _pick_command(self, event):
        action = self.pick_action
        if action == 'rebind':
            stage = a.active(self.node)
            source = stage.node('bind').input(0).geometry()
        else:
            source = self.pick_geometry()
        hit = self._pick(source,event)
        if hit is None:
            return False
        pos, normal, name, key = hit
        if action == 'rebind':
            with hou.undos.group('Rebind KoAdjust Stroke'):
                bind=stage.node('bind')
                deform=stage.node('deform')
                direction=deform.parm('direction')
                if direction is not None and direction.eval()=='normal':
                    amount=hou.Vector3(deform.evalParmTuple('delta')).dot(hou.Vector3(bind.evalParmTuple('normal')))
                    deform.parmTuple('delta').set(normal*amount)
                bind.setParms({'center':tuple(pos),'normal':tuple(normal),'anchor_part':name,'definition':key})
        elif action == 'hide':
            self.set_hidden(self.hidden | {name})
        elif action == 'solo':
            self.set_hidden(set(a.parts(self.node)) - {name})
        self.pick_action = None
        self.refresh()
        return True

    def _ask_rewind(self,node,key,count):
        if self.pending_rewind==key:
            return
        self.pending_rewind=key

        def ask():
            if self.node!=node or self.pending_rewind!=key:
                return
            try:
                choice=hou.ui.displayMessage(f'Discard {count} later stroke(s) when the next stroke commits?',
                    buttons=('Continue','Cancel'),default_choice=1,close_choice=1,title='KoAdjust Rewind')
            finally:
                if self.pending_rewind==key:
                    self.pending_rewind=None
            if self.node!=node or node.evalParm('active_stage')!=key[0]:
                return
            if choice==0:
                self.approved_rewind=key
                self.refresh('KoAdjust · Ready to branch. Brush again to start; Esc cancels.')
            else:
                self.refresh('KoAdjust · Kept later strokes.')

        # A native modal interrupts the state and steals the release event.
        # Don't open it inside a mouse handler or start the stroke that opened it.
        hou.ui.postEventCallback(ask)

    def _begin(self, event):
        if self.pick_action:
            return self._pick_command(event)
        review_before=self.review
        node = self.node
        upstream = node.input(0)
        if upstream is None:
            raise ValueError('Connect an input mesh first.')
        geo = self.pick_geometry()
        hit = self._pick(geo,event)
        if hit is None:
            return False
        pos, normal, name, key = hit
        device=event.device()
        tool='smooth' if device.isShiftKey() else node.evalParm('tool')
        radius = node.evalParm('radius')
        future=a.future(node)
        previous=node.evalParm('active_stage')
        tail=node.node('stack/OUT')
        restore=tail.inputConnections()[0].inputItem()
        branch=None
        if future:
            rewind_key=(previous,tuple(s.sessionId() for s in future))
            if self.approved_rewind!=rewind_key:
                self.approved_rewind=None
                self._ask_rewind(node,rewind_key,len(future))
                return True
            branch=node.node('stack').indirectInputs()[0] if previous==a.BASE else a.active(node)
        else:
            self.approved_rewind=None
        vp = self.viewer.curViewport()
        revision = upstream.geometry().vexAttribDataId()
        camera = (vp.name(),vp.viewTransform().asTuple())
        self.handle.show(False)
        self.viewer.beginStateUndo('KoAdjust ' + a.LABEL[tool])
        try:
            stage = a.append(node,tool,mode=node.evalParm('influence'),anchor_part=name,definition=key,
                             center=pos,normal=normal,radius=radius,branch_input=branch,direction=node.evalParm('move_direction'))
        except Exception:
            self.viewer.endStateUndo()
            raise
        self.drag = dict(stage=stage,tool=tool,center=pos,normal=normal,plane=event.ray()[1].normalized(),
                         x=device.mouseX(),y=device.mouseY(),previous=previous,moved=False,last=0.,
                         upstream=upstream,revision=revision,camera=camera,radius=radius,
                         future=future,restore=restore,direction=node.evalParm('move_direction'),
                         invert=device.isCtrlKey(),review_before=review_before)
        self.review=False
        node.parm('active_stage').set('')
        self._preview_parts(stage)
        self._brush_ring(pos,normal,radius)
        return True

    def _update(self,event,final=False):
        drag = self.drag
        tick = time.monotonic()
        if not final and tick-drag['last'] < 1/45:
            return
        drag['last'] = tick
        deform = drag['stage'].node('deform')
        tool = drag['tool']
        if tool == 'grab':
            o,d = event.ray()
            n,p = drag['plane'],drag['center']
            denom = n.dot(d)
            if abs(denom)<1e-8:
                return
            delta = o+d*((p-o).dot(n)/denom)-p
            if drag['direction']=='normal':
                axis=drag['normal']
                dn=d.normalized()
                nd=axis.dot(dn)
                if abs(nd)<.94 and abs(axis.dot(n))<.94:
                    rel=o-p
                    amount=(axis.dot(rel)-nd*dn.dot(rel))/(1-nd*nd)
                else:
                    # An almost end-on normal has no stable screen projection.
                    amount=(event.device().mouseY()-drag['y'])*drag['radius']/100
                delta=axis*amount
            deform.parmTuple('delta').set(delta)
            value = delta.length()
        else:
            motion = event.device().mouseX()-drag['x']
            drag['has_motion'] = drag.get('has_motion',False) or abs(motion)>2
            fraction = motion/160
            if final and not drag['has_motion']:
                fraction = {'clay':.20,'inflate':.12,'smooth':.45,'relax':.45,'flatten':.6}[tool]
            if tool in ('clay','inflate'):
                value=fraction*drag['radius']*(-1 if drag['invert'] else 1)
            else:
                value=max(0.,min(1.,fraction))
            deform.parm('strength').set(value)
        drag['moved'] = abs(value)>1e-7
        self.viewer.setPromptMessage(f'KoAdjust {a.LABEL[tool]} · {value:.4g} · Wheel: radius · Esc: cancel')

    def _finish(self,cancel=False):
        drag,self.drag = self.drag,None
        if drag is None:
            return
        stage = drag['stage']
        issue=''
        try:
            if drag['future'] and not cancel and drag['moved']:
                try:
                    issue=a.status(stage)
                except hou.Error as exc:
                    issue=str(exc)
            if cancel or issue or not drag['moved']:
                if stage and self.node.node('stack/OUT').input(0) == stage:
                    self.node.node('stack/OUT').setInput(0,drag['restore'])
                    stage.destroy()
                    self.node.parm('active_stage').set(drag['previous'])
                    self.review=drag['review_before']
            else:
                for later in reversed(drag['future']):
                    later.destroy()
                if drag['future']:
                    self.approved_rewind=None
        except hou.ObjectWasDeleted:
            pass
        finally:
            if self.wire:
                self.wire.show(False)
            self.ghost.show(False)
            self.ring.show(False)
            self.before = self.ghost_mesh = None
            try:
                self._set_focus(set())
                self._editing()
            except hou.ObjectWasDeleted:
                pass
            self.viewer.endStateUndo()
        self.refresh(f'KoAdjust · Kept later strokes: {issue}' if issue else '')

    def onMouseEvent(self, kwargs):
        if self.node is None:
            return False
        event = kwargs['ui_event']
        reason = event.reason()
        try:
            if self.drag is None and reason in (hou.uiEventReason.Start,hou.uiEventReason.Picked) and event.device().isLeftButton():
                consumed = self._begin(event)
                if self.drag and reason==hou.uiEventReason.Picked:
                    self._update(event, final=True)
                    self._finish()
                return consumed
            if self.drag:
                if reason==hou.uiEventReason.Active:
                    self._update(event)
                elif reason==hou.uiEventReason.Changed:
                    vp = self.viewer.curViewport()
                    source = self.node.input(0)
                    if (source!=self.drag['upstream'] or source.geometry().vexAttribDataId()!=self.drag['revision'] or
                            (vp.name(),vp.viewTransform().asTuple())!=self.drag['camera']):
                        raise ValueError('Input or viewport changed during the stroke. Try again.')
                    self._update(event, final=True)
                    self._finish()
                return True
            if reason==hou.uiEventReason.Located and not self.review:
                tick = time.monotonic()
                if tick-self.last_hover>=.045:
                    self.last_hover = tick
                    hit = self.hover = self._pick(self.pick_geometry(),event)
                    if hit:
                        self._brush_ring(hit[0],hit[1],self.node.evalParm('radius'))
                        self._editing({hit[2]})
                        self.viewer.setPromptMessage(f'KoAdjust · {hit[2] or "(empty label)"} · LMB brush · Shift: Smooth · Wheel radius')
                    else:
                        self.ring.show(False)
                        self._editing()
        except (hou.Error,ValueError) as exc:
            self._finish(cancel=True)
            self.refresh(str(exc))
            return True
        return False

    def onMouseWheelEvent(self,kwargs):
        if self.node is None:
            return False
        event=kwargs['ui_event']
        step=event.device().mouseWheel()
        if not step:
            return False
        direction = 1 if step>0 else -1
        stage = self.drag['stage'] if self.drag else a.active(self.node) if self.review else None
        parm = stage.node('bind').parm('radius') if stage else self.node.parm('radius')
        value = min(1e8,max(1e-8,parm.eval()*1.15**direction))
        if self.drag:
            parm.set(value)
            self.drag['radius']=value
            self._preview_parts(self.drag['stage'],update_wire=False)
            self._brush_ring(self.drag['center'],self.drag['normal'],value)
        else:
            with hou.undos.group('KoAdjust Brush Radius'):
                parm.set(value)
            if stage:
                bind=stage.node('bind')
                self._brush_ring(hou.Vector3(bind.evalParmTuple('center')),hou.Vector3(bind.evalParmTuple('normal')),value)
            else:
                try:
                    self.hover=self._pick(self.pick_geometry(),event)
                except (hou.Error,ValueError):
                    self.hover=None
                if self.hover:
                    self._brush_ring(self.hover[0],self.hover[1],value)
                else:
                    self.ring.show(False)
        self.viewer.setPromptMessage(f'KoAdjust · Radius {value:.4g}')
        self.viewer.curViewport().draw()
        return True

    def onStateToHandle(self,kwargs):
        deform = self._grab()
        if deform is not None:
            p = hou.Vector3(deform.parent().node('bind').evalParmTuple('center'))+hou.Vector3(deform.evalParmTuple('delta'))
            kwargs['parms'].update(dict(tx=p[0],ty=p[1],tz=p[2]))

    def onHandleToState(self,kwargs):
        deform = self._grab()
        if deform is not None and set(kwargs['mod_parms']).intersection(('tx','ty','tz')):
            p = kwargs['parms']
            anchor = hou.Vector3(deform.parent().node('bind').evalParmTuple('center'))
            deform.parmTuple('delta').set(hou.Vector3(p['tx'],p['ty'],p['tz'])-anchor)

    def onBeginHandleToState(self,kwargs):
        deform = self._grab()
        if deform is not None:
            self.viewer.beginStateUndo('KoAdjust Move Handle')
            self.handle_undo = True
            self._preview_parts(deform.parent(),update_wire=False)
            if not self.hidden:
                self._original(deform.parent().node('bind').input(0).geometry())

    def onEndHandleToState(self,kwargs):
        if self.handle_undo:
            self.handle_undo = False
            if self.wire:
                self.wire.show(False)
            self.ghost.show(False)
            self.before = self.ghost_mesh = None
            try:
                self._set_focus(set())
                self._editing()
            except hou.ObjectWasDeleted:
                pass
            self.viewer.endStateUndo()

    def onKeyEvent(self,kwargs):
        key = kwargs['ui_event'].device().keyString().lower()
        if key in ('esc','escape') and (self.drag or self.pick_action or self.approved_rewind or self.pending_rewind):
            self.pick_action = self.approved_rewind = self.pending_rewind = None
            self._finish(cancel=True)
            self.refresh()
            return True
        return False

    def onInterrupt(self,kwargs):
        self._finish(cancel=True)
        self.onEndHandleToState(kwargs)
        if self.ring:
            self.ring.show(False)

    def onResume(self,kwargs):
        if self.node:
            self._expire_visibility()
            self.refresh()

    def onExit(self,kwargs):
        self.onInterrupt(kwargs)
        try:
            self.set_hidden(set())
            self._set_focus(set())
            self._editing()
        except hou.ObjectWasDeleted:
            pass
        self.node = self.before = None
        self.pending_rewind = self.approved_rewind = None
        self.viewer.hudInfo(show=False,panel=hou.hudPanel.ToolInfo)
        self.viewer.clearPromptMessage()

    def onCommand(self,kwargs):
        command,args = kwargs['command'],kwargs['command_args']
        if command=='tool' and args in a.TOOLS:
            self.onMenuAction({'menu_item':args})
        elif command=='review':
            self.onMenuAction({'menu_item':'review'})
        elif command=='inspect' and isinstance(args,dict):
            args.update(ready=self.node is not None,reviewing=self.review,
                        handle_visible=self.handle.visible() if self.handle else False)

    def onMenuPreOpen(self,kwargs):
        item=kwargs['menu_item_states'].get('toggle_move_direction')
        if item is not None:
            item['enable']=self.node is not None and self.node.evalParm('tool')=='grab' and self.drag is None

    def onMenuAction(self,kwargs):
        action = kwargs['menu_item']
        if self.node is None or self.drag:
            return
        node,stage = self.node,a.active(self.node)
        try:
            if action in a.TOOLS:
                node.parm('tool').set(action)
                self.pick_action,self.review = None,False
            elif action in MODES:
                parm = stage.node('bind').parm('mode') if self.review and stage else node.parm('influence')
                with hou.undos.group('KoAdjust Influence'):
                    parm.set(action)
            elif action=='radial':
                self.viewer.displayRadialMenu('ko_adjust')
            elif action in ('hide','solo','rebind'):
                if action!='rebind' or stage:
                    self.pick_action = action
            elif action=='show_all':
                self.set_hidden(set())
                self.pick_action = None
            elif action=='parts':
                from ko_adjust_parts import open_panel
                open_panel(node)
                return
            elif action=='toggle_move_direction':
                if node.evalParm('tool')!='grab':
                    return
                parm=node.parm('move_direction')
                direction='view' if parm.eval()=='normal' else 'normal'
                with hou.undos.group('KoAdjust Move Direction'):
                    parm.set(direction)
                self.refresh('KoAdjust Move · '+('Surface Normal' if direction=='normal' else 'View Plane'))
                return
            elif action in ('review','prev','next') and stage:
                stages = a.history(node)
                i = stages.index(stage)+{'review':0,'prev':-1,'next':1}[action]
                node.parm('active_stage').set(stages[max(0,min(i,len(stages)-1))].name())
                self.review,self.pick_action = True,None
            elif action in ('new','latest'):
                if action=='latest':
                    node.parm('active_stage').set('')
                self.review,self.pick_action = False,None
            elif action=='mute' and stage:
                with hou.undos.group('Mute KoAdjust Stroke'):
                    stage.bypass(not stage.isBypassed())
            elif action=='wire':
                node.parm('wire').set(not node.evalParm('wire'))
            self.refresh()
        except (hou.Error,ValueError) as exc:
            self.refresh(str(exc))


def template():
    template = hou.ViewerStateTemplate(a.STATE,'KoAdjust',hou.sopNodeTypeCategory())
    template.bindFactory(State)
    template.bindIcon('SOP_edit')
    template.bindHandle('xform','grab_move',handle_parms=['tx','ty','tz'])
    menu = hou.ViewerStateMenu('ko_adjust_menu','KoAdjust')
    context = a.STATE_CONTEXT
    keys = hou.PluginHotkeyDefinitions()
    keys.addContext(context,'KoAdjust','Active only inside KoAdjust')
    keys.addCommandCategory(context,'KoAdjust','KoAdjust viewer tools')
    sections = (
        ('brushes','Brushes',[(x,a.LABEL[x]) for x in a.TOOLS]+
            [('toggle_move_direction','Toggle Move Direction'),('radial','Open Brush Radial')]),
        ('influence','Affect',list(MODES.items())),
        ('parts','Parts',[('parts','Open Parts List'),('hide','Hide Picked Part'),('solo','Isolate Picked Part'),('show_all','Show All Parts')]),
        ('history','Stroke History',[('review','Review Current Stroke'),('prev','Review Previous Stroke'),('next','Review Next Stroke'),
            ('new','Brush From Rewind Point'),('latest','Return to Latest'),('rebind','Rebind Reviewed Stroke'),('mute','Mute / Unmute Reviewed Stroke')]),
        (None,None,[('wire','Toggle Before Wireframe')]),
    )
    for section,label,actions in sections:
        target = hou.ViewerStateMenu('ko_adjust_'+section,label) if section else menu
        for action,title in actions:
            symbol = context+'.'+action
            keys.addCommand(symbol,title,title)
            keys.addDefaultBinding(context,symbol,[a.MOVE_DIRECTION_KEY] if action=='toggle_move_direction' else [])
            target.addActionItem(action,title,symbol)
        if section:
            menu.addMenu(target)
    template.bindHotkeyDefinitions(keys)
    template.bindMenu(menu)
    return template
