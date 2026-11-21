"""KoAdjust viewer gestures and temporary part visibility."""
from functools import wraps
import json
import math
import sys
import time
import traceback

import hou
from hutil.Qt import QtCore, QtWidgets
from viewerstate.utils import hudHotkeyRef

import ko_adjust as a

MODES = {'part':'One Part','nearby':'Nearby in 3D','connected':'Topologically Connected'}


def cursor_feedback(strength,falloff):
    """Radius of the 50%-weight contour and stroke width in viewport pixels."""
    threshold=.5**(1/max(.2,falloff))
    lo,hi=0.,1.
    for _ in range(12):
        t=(lo+hi)*.5
        if t*t*(3-2*t)<threshold:
            lo=t
        else:
            hi=t
    return 1-(lo+hi)*.5, 1.0+4.0*min(1.,max(0.,strength))


def guarded_viewer_callback(handler):
    @wraps(handler)
    def run(self,kwargs):
        try:
            return handler(self,kwargs)
        except Exception:
            self._record_viewer_error(handler.__name__)
            if handler.__name__ in ('onMouseEvent','onMouseWheelEvent'):
                for cancel in (self._end_brush_adjust,self._finish):
                    try:
                        cancel(cancel=True)
                    except Exception:
                        pass  # Keep the original traceback, not a cleanup failure.
                return True
            return False
    return run


class State:
    def __init__(self, state_name, scene_viewer):
        self.viewer = scene_viewer
        self.node = self.handle = self.ring = self.inner_ring = self.wire = self.ghost = None
        self._ring_feedback=None
        self._ring_profile=None
        self._shift_smooth=False
        self._viewer_error=''
        self._error_reported={}
        self.drag = self.pick_action = None
        self.review = self.handle_undo = False
        self.hidden = set()
        self.last_hover = 0.
        self.hover = None
        self.brush_adjust = None
        self.pending_rewind = self.approved_rewind = None
        self._hud_callback=self._brush_parm_changed
        self._hud_pending=False
        self._hud_values=None

    def _record_viewer_error(self,where):
        self._viewer_error=f'KoAdjust {where}\n{traceback.format_exc()}'
        now=time.monotonic()
        if now-self._error_reported.get(where,-10.)<2.:
            return
        self._error_reported[where]=now
        try:
            sys.stderr.write(self._viewer_error)
            self.viewer.setPromptMessage('KoAdjust input error · RMB → Copy Last Viewer Error')
        except Exception:
            pass

    @guarded_viewer_callback
    def onEnter(self, kwargs):
        self.node = kwargs['node']
        a.update_saved_strokes(self.node)
        a.sync_rewind(self.node)
        self.review=bool(self.node.evalParm('active_stage'))
        self.handle = hou.Handle(self.viewer, 'grab_move')
        self.ring = hou.GeometryDrawable(self.viewer, hou.drawableGeometryType.Line, 'ko_adjust_radius')
        self.ring.setParams({'color1':(.32,.79,1.,.9),'line_width':1.})
        self.inner_ring=hou.GeometryDrawable(self.viewer,hou.drawableGeometryType.Line,'ko_adjust_strength')
        self._ring_feedback=None
        self._ring_profile=None
        self.circle = hou.Geometry()
        circle = self.circle.createPolygon()
        for point in self.circle.createPoints(tuple((math.cos(i*math.tau/64),math.sin(i*math.tau/64),0.) for i in range(64))):
            circle.addVertex(point)
        self.ring.setGeometry(self.circle)
        self.inner_ring.setGeometry(self.circle)
        self.wire = hou.GeometryDrawable(self.viewer,hou.drawableGeometryType.Line,'ko_adjust_before')
        self.wire.setParams({'color1':(.54,.75,.85,.22),'line_width':1.,'fade_factor':.2})
        self.ghost = hou.GeometryDrawable(self.viewer,hou.drawableGeometryType.Face,'ko_adjust_other_parts')
        self.ghost.setParams({'color1':(.54,.59,.65,.19),'fade_factor':.2})
        self._set_focus(set())
        self._set_view_active(True)
        self.set_hidden(a.hidden_parts(self.node))
        self.refresh()
        self.node.addEventCallback((hou.nodeEventType.ParmTupleChanged,),self._hud_callback)

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
        if self.hover and self.hover[2] in self.hidden:
            self.hover=None

    def _set_focus(self,parts):
        with hou.undos.disabler():
            self.node.parm('view_focus').set(json.dumps(dict.fromkeys(sorted(parts),1)) if parts else '')

    def _set_view_active(self,value):
        with hou.undos.disabler():
            parm=self.node.parm('view_active')
            if parm is not None:
                parm.set(value)

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
        if self.drag is not None:
            signature=(frozenset(affected),frozenset(others))
            if not update_wire and self.drag.get('display_parts')==signature:
                return
            self.drag['display_parts']=signature
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
        tool = a.choice(self.node,'tool')
        stage = a.active(self.node)
        issue = a.status(stage) if stage and not self.drag else ''
        if self.pick_action:
            usage = dict(rebind='Click a new anchor', hide='Click a part to hide', solo='Click a part to isolate')[self.pick_action] + ' · Esc cancels'
        else:
            usage = 'Drag to branch' if self.review else 'KoAdjust'
        self.viewer.setPromptMessage(message or issue or usage)
        label=a.LABEL[stage.userData('ko_adjust_tool')] if self.review and stage else a.LABEL[tool]
        self.viewer.hudInfo(show=True,template=self._hud_template(tool,label),panel=hou.hudPanel.ToolInfo)
        self._hud_values=None
        self._update_brush_hud()
        self.handle.show(self._grab() is not None and self.drag is None)
        if self._grab() is not None:
            self.handle.update()

    def _hud_template(self,tool,label):
        if self._shift_smooth and not self.review:
            return {'title':'KoAdjust · Smooth','icon':a.ICON,'rows':[
                {'id':'radius','label':'Radius','key':'Shift RMB'},
                {'id':'strength','label':'Strength (L/R)','key':'Shift MMB'},
                {'id':'falloff','label':'Falloff (U/D)','key':'Shift MMB'},
                {'label':'Smooth','key':'Shift LMB'}]}
        # Native Sculpt token syntax shows mouse glyphs instead of spelling chords out.
        rows=[{'id':'radius','label':'Radius','key':'mousewheel'},
              {'label':'Radius (drag)','key':'Ctrl Shift LMB'},
              {'id':'strength','label':'Strength (L/R)','key':'Ctrl Shift MMB'},
              {'id':'falloff','label':'Falloff (U/D)','key':'Ctrl Shift MMB'},
              {'label':'Smooth','key':'Shift LMB'},
              {'label':'Tools','key':'RMB'}]
        if tool=='grab' and not self.review:
            direction='Normal' if a.choice(self.node,'move_direction')=='normal' else 'View Plane'
            rows.append({'label':direction,'key':hudHotkeyRef(a.MOVE_DIRECTION_ACTION)})
        return {'title':'KoAdjust · '+label,'icon':a.ICON,'rows':rows}

    def _selected_profile(self):
        return a.SURFACE_REFINEMENT if self._shift_smooth else a.profile_for_tool(a.choice(self.node,'tool'))

    def _settings_stage(self):
        return (self.drag['stage'] if self.drag else a.active(self.node)
                if self.review and not self._shift_smooth else None)

    def _cursor_profile(self):
        if self.brush_adjust:
            return self.brush_adjust['profile']
        stage=self._settings_stage()
        return a.profile_for_tool(stage.userData('ko_adjust_tool')) if stage is not None else self._selected_profile()

    def _brush_settings(self):
        if self.brush_adjust:
            values=self.brush_adjust['values']
        else:
            stage=self._settings_stage()
            if stage is not None:
                bind=stage.node('bind')
                strength=bind.parm('brush_strength') or self.node.parm(a.setting_parm(self._cursor_profile(),'strength'))
                return bind.evalParm('radius'),strength.eval(),bind.evalParm('falloff')
            values=a.profile_settings(self.node,self._selected_profile())
        return tuple(values[name] for name in a.BRUSH_SETTINGS)

    def _update_brush_hud(self,settings=None):
        radius,strength,falloff=self._brush_settings() if settings is None else settings
        values=dict(radius=f'{radius:.4g}',strength=f'{strength:.2f}',falloff=f'{falloff:.2f}')
        if values!=self._hud_values:
            self.viewer.hudInfo(values=values,panel=hou.hudPanel.ToolInfo)
            self._hud_values=values

    def _observe_shift(self,device):
        self._shift_smooth=device.isShiftKey() and not device.isCtrlKey()

    def _brush_parm_changed(self,event_type,parm_tuple=None,**kwargs):
        if parm_tuple is not None and (parm_tuple.name()=='tool' or parm_tuple.name() in a.BRUSH_SETTING_PARMS):
            self._schedule_ring_refresh()

    def _schedule_ring_refresh(self):
        if self._hud_pending or self.node is None or self.brush_adjust is not None:
            return
        node=self.node
        self._hud_pending=True

        def update():
            self._hud_pending=False
            if self.node is None or self.node!=node:
                return
            try:
                if not self.drag and self.brush_adjust is None:
                    settings=self._brush_settings()
                    self._update_brush_hud(settings)
                    if self.hover and not self.review:
                        self._brush_ring(self.hover[0],self.hover[1],settings[0])
                        self.viewer.curViewport().draw()
            except hou.ObjectWasDeleted:
                pass
            except Exception:
                self._record_viewer_error('ring refresh')
        hou.ui.postEventCallback(update)

    @guarded_viewer_callback
    def onDraw(self, kwargs):
        for drawable in (self.ghost,self.wire,self.ring,self.inner_ring):
            if drawable and drawable.visible():
                drawable.draw(kwargs['draw_handle'])

    def _original(self, geo):
        self.before = geo
        self.wire.setGeometry(geo)
        self.wire.show(bool(self.node.evalParm('wire')))

    def _brush_ring(self, center, normal, radius):
        rotation=hou.Vector3(0,0,1).matrixToRotateTo(normal)*hou.hmath.buildTranslate(center)
        _,strength,falloff=self._brush_settings()
        params=cursor_feedback(strength,falloff)
        inner_scale,width=params
        profile=self._cursor_profile()
        color=a.PROFILE_COLORS[profile]
        if self._ring_profile!=profile:
            self.ring.setParams({'color1':(*color,.9),'line_width':1.})
        if self._ring_feedback!=params or self._ring_profile!=profile:
            self.inner_ring.setParams({'color1':(*color,1.),'line_width':width})
            self._ring_feedback=params
        self._ring_profile=profile
        self.ring.setTransform(hou.hmath.buildScale((radius,)*3)*rotation)
        self.inner_ring.setTransform(hou.hmath.buildScale((radius*inner_scale,)*3)*rotation)
        self.ring.show(True)
        self.inner_ring.show(True)

    def _hide_brush_ring(self):
        for ring in (self.ring,self.inner_ring):
            if ring is not None:
                ring.show(False)

    def _pick(self, geo, event, *, visible=True):
        return self._ray_pick(geo,*event.ray(),visible=visible)

    def _ray_pick(self,geo,o,d,*,visible=True,target_part=None):
        if geo is None or not geo.findPrimAttrib(a.PART):
            raise ValueError('Parts are unavailable. Check Define Parts By and the input attribute.')
        pos, n, uv = hou.Vector3(), hou.Vector3(), hou.Vector3()
        part = geo.findPrimAttrib(a.PART)
        # Use the native group on the live preview. For a reviewed stage, the
        # temporary group isn't in its history, so skip hidden hits by name.
        hidden_group = visible and self.hidden and geo.findPrimGroup(a.HIDDEN)
        pattern='*' if target_part is None else '@'+a.PART+'='+json.dumps(target_part,ensure_ascii=False)
        if hidden_group:
            pattern+=' ^'+a.HIDDEN
        for _ in range(64):
            i = geo.intersect(o,d,pos,n,uv,pattern=pattern,min_hit=0.,tolerance=1e-6)
            if i < 0:
                return None
            prim=geo.prim(i)
            name=prim.attribValue(part)
            if (target_part is None or name==target_part) and (not visible or name not in self.hidden):
                if not isinstance(prim,hou.Polygon) or not prim.isClosed():
                    raise ValueError('Pick a closed polygon (unpack packed surfaces before brushing).')
                return pos,prim.normal(),name,a.part_key(geo)
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
                if direction is not None and direction.evalAsString()=='normal':
                    amount=hou.Vector3(deform.evalParmTuple('delta')).dot(hou.Vector3(bind.evalParmTuple('normal')))
                    deform.parmTuple('delta').set(normal*amount)
                a.rebase_samples(bind,pos,normal)
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
        tool='smooth' if device.isShiftKey() else a.choice(node,'tool')
        settings=a.profile_settings(node,a.profile_for_tool(tool))
        radius=settings['radius']
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
        stroke_input=geo.freeze(read_only=True) if tool!='grab' else None
        vp = self.viewer.curViewport()
        revision = upstream.geometry().vexAttribDataId()
        camera = (vp.name(),vp.viewTransform().asTuple())
        self.handle.show(False)
        self.viewer.beginStateUndo('KoAdjust ' + a.LABEL[tool])
        try:
            stage = a.append(node,tool,mode=a.choice(node,'influence'),anchor_part=name,definition=key,
                             center=pos,normal=normal,radius=radius,branch_input=branch,direction=a.choice(node,'move_direction'),
                             falloff=settings['falloff'],brush_strength=settings['strength'])
        except Exception:
            self.viewer.endStateUndo()
            raise
        self.drag = dict(stage=stage,tool=tool,center=pos,normal=normal,plane=event.ray()[1].normalized(),
                         y=device.mouseY(),previous=previous,moved=False,last=0.,
                         upstream=upstream,revision=revision,camera=camera,radius=radius,
                         future=future,restore=restore,direction=a.choice(node,'move_direction'),
                         invert=device.isCtrlKey(),review_before=review_before,strength=settings['strength'],
                         stroke_input=stroke_input,samples=[],last_sample=pos,last_ray=event.ray(),part=name,
                         mode=a.choice(node,'influence'))
        if tool!='grab':
            self._set_dab_amount(self.drag)
        self.review=False
        a.set_rewind(node,'')
        self._preview_parts(stage)
        self._brush_ring(pos,normal,radius)
        self.viewer.setPromptMessage(f'KoAdjust {a.LABEL[tool]} · Wheel: radius · Esc: cancel')
        return True

    def _set_dab_amount(self,drag):
        amount=drag['strength']
        if drag['tool'] in ('clay','inflate'):
            amount*=drag['radius']*(.25 if drag['tool']=='clay' else .2)
            if drag['invert']:
                amount=-amount
        drag['stage'].node('deform').parm('strength').set(amount)
        drag['moved']=abs(amount)>1e-8

    def _stroke_pick(self,drag,origin,direction):
        geo=drag['stroke_input']
        hit=self._ray_pick(geo,origin,direction,target_part=drag['part'])
        if hit is None and drag['mode']!='part':
            hit=self._ray_pick(geo,origin,direction)
        return hit

    def _sample_stroke(self,drag,event,final):
        ray=event.ray()
        hit=self._stroke_pick(drag,*ray)
        if not hit:
            return
        pos=hit[0]
        spacing=max(1e-8,drag['radius']*.32)
        travel=(pos-drag['last_sample']).length()
        if travel<spacing*(.5 if final else 1.) or len(drag['samples'])>=a.MAX_STAMPS-1:
            return
        sections=min(16,max(1,math.ceil(travel/spacing)))
        old_o,old_d=drag['last_ray']
        new_o,new_d=ray
        changed=False
        for i in range(1,sections+1):
            if len(drag['samples'])>=a.MAX_STAMPS-1:
                break
            t=i/sections
            origin=old_o+(new_o-old_o)*t
            direction=(old_d+(new_d-old_d)*t).normalized()
            sample=hit if i==sections else self._stroke_pick(drag,origin,direction)
            if sample is None:
                continue
            p,n=sample[:2]
            if (p-drag['last_sample']).length()>2*drag['radius'] and drag['mode']!='nearby':
                break
            drag['samples'].append((*p,*n))
            drag['last_sample'],drag['last_ray']=p,(origin,direction)
            changed=True
        if changed:
            drag['stage'].node('bind').parm('samples').set(a.encode_samples(drag['samples']))
            self._preview_parts(drag['stage'],update_wire=False)
        self._brush_ring(pos,hit[1],drag['radius'])

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
            delta*=drag['strength']
            deform.parmTuple('delta').set(delta)
            drag['moved']=delta.length()>1e-7
        else:
            full=len(drag['samples'])>=a.MAX_STAMPS-1
            self._sample_stroke(drag,event,final)
            if not full and len(drag['samples'])>=a.MAX_STAMPS-1:
                self.viewer.setPromptMessage('Stroke full — release to finish')

    def _finish(self,cancel=False):
        drag,self.drag = self.drag,None
        if drag is None:
            return
        stage = drag['stage']
        issue=''
        try:
            if not cancel and drag['moved']:
                try:
                    issue=a.status(stage)
                    if not issue:
                        before=stage.node('bind').input(0).geometry()
                        after=stage.geometry()
                        if before is None or after is None:
                            issue='Stroke could not cook.'
                        elif (before.pointFloatAttribValuesAsString('P',hou.numericData.Float64)==
                              after.pointFloatAttribValuesAsString('P',hou.numericData.Float64)):
                            issue='Stroke did not move any points.'
                except hou.Error as exc:
                    issue=str(exc) or 'Stroke could not cook.'
            if cancel or issue or not drag['moved']:
                if stage and self.node.node('stack/OUT').input(0) == stage:
                    self.node.node('stack/OUT').setInput(0,drag['restore'])
                    stage.destroy()
                    a.set_rewind(self.node,drag['previous'])
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
            self._hide_brush_ring()
            self.before = self.ghost_mesh = None
            try:
                self._set_focus(set())
                self._editing()
            except hou.ObjectWasDeleted:
                pass
            self.viewer.endStateUndo()
        self.refresh(f'KoAdjust · Stroke not saved: {issue}' if issue else '')

    def _begin_brush_adjust(self,event,kind,profile):
        device=event.device()
        if self.hover is None:
            try:
                self.hover=self._pick(self.pick_geometry(),event)
            except (hou.Error,ValueError):
                pass
        initial=a.profile_settings(self.node,profile)
        self.brush_adjust=dict(kind=kind,profile=profile,x=device.mouseX(),y=device.mouseY(),
                               initial=initial,values=initial.copy(),shown=None,last_preview=0.)
        self._update_brush_adjust(event)

    def _update_brush_adjust(self,event,final=False):
        adjust=self.brush_adjust
        values=adjust['initial']
        device=event.device()
        dx=device.mouseX()-adjust['x']
        dy=device.mouseY()-adjust['y']
        if adjust['kind']=='radius':
            # HOM mouse Y increases upward: right/up grows, left/down shrinks.
            radius=min(1e8,max(1e-8,values['radius']*math.exp(max(-20,min(20,(dx+dy)/140)))))
            adjust['values']['radius']=radius
        else:
            strength=min(1.,max(0.,values['strength']+dx/200))
            # A lower exponent expands the inner falloff contour.
            falloff=min(5.,max(.2,values['falloff']-dy/120))
            adjust['values'].update(strength=strength,falloff=falloff)
        # Keep the exact drag value, but don't hammer native viewport updates at
        # the input device's polling rate. Always show the final release value.
        now=time.monotonic()
        shown=tuple(adjust['values'][name] for name in a.BRUSH_SETTINGS)
        if shown==adjust['shown'] or (not final and adjust['shown'] is not None and now-adjust['last_preview']<1/30):
            return
        adjust['shown'],adjust['last_preview']=shown,now
        # Live feedback belongs in the HUD, not Houdini's prompt/error buffer.
        self._update_brush_hud(shown)
        if self.hover:
            self._brush_ring(self.hover[0],self.hover[1],adjust['values']['radius'])

    def _end_brush_adjust(self,cancel=False):
        adjust,self.brush_adjust=self.brush_adjust,None
        if adjust is None:
            return
        if not cancel:
            values={a.setting_parm(adjust['profile'],key):value for key,value in adjust['values'].items()
                    if value!=adjust['initial'][key]}
            if values:
                try:
                    if any(self.node.parm(name).isLocked() for name in values):
                        raise ValueError('Brush settings are locked.')
                    with hou.undos.group('KoAdjust '+a.PROFILE_LABELS[adjust['profile']]):
                        self.node.setParms(values)
                except (hou.Error,ValueError) as exc:
                    self.viewer.setPromptMessage(f'KoAdjust · {exc}')
        self._update_brush_hud()
        self._schedule_ring_refresh()

    @guarded_viewer_callback
    def onMouseEvent(self, kwargs):
        if self.node is None:
            return False
        event = kwargs['ui_event']
        reason = event.reason()
        adjusting=self.brush_adjust is not None
        try:
            device=event.device()
            self._observe_shift(device)
            if adjusting:
                if reason==hou.uiEventReason.Active:
                    self._update_brush_adjust(event)
                elif reason==hou.uiEventReason.Changed:
                    self._update_brush_adjust(event,final=True)
                    self._end_brush_adjust()
                return True
            if self.drag is None and reason in (hou.uiEventReason.Start,hou.uiEventReason.Picked,hou.uiEventReason.Active):
                kind=None
                profile=None
                if self._shift_smooth and (device.isRightButton() or device.isMiddleButton()):
                    kind='radius' if device.isRightButton() else 'strength'
                    profile=a.SURFACE_REFINEMENT
                elif device.isCtrlKey() and device.isShiftKey() and (device.isLeftButton() or device.isMiddleButton()):
                    kind='radius' if device.isLeftButton() else 'strength'
                    profile=a.profile_for_tool(a.choice(self.node,'tool'))
                if kind:
                    adjusting=True
                    self._begin_brush_adjust(event,kind,profile)
                    if reason==hou.uiEventReason.Picked:
                        self._end_brush_adjust()
                    return True
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
                    self._update_brush_hud()
                    hit = self.hover = self._pick(self.pick_geometry(),event)
                    if hit:
                        self._brush_ring(hit[0],hit[1],self._brush_settings()[0])
                        self._editing({hit[2]})
                        self.viewer.setPromptMessage(hit[2] or 'Unnamed part')
                    else:
                        self._hide_brush_ring()
                        self._editing()
        except (hou.Error,ValueError) as exc:
            if adjusting:
                raise  # Store the traceback for Copy Last Viewer Error.
            self._end_brush_adjust(cancel=True)
            self._finish(cancel=True)
            self.refresh(str(exc))
            return True
        return False

    @guarded_viewer_callback
    def onMouseWheelEvent(self,kwargs):
        if self.node is None:
            return False
        event=kwargs['ui_event']
        device=event.device()
        self._observe_shift(device)
        if self.brush_adjust:
            return True
        step=device.mouseWheel()
        if not step:
            return False
        direction = 1 if step>0 else -1
        stage=self._settings_stage()
        profile=self._selected_profile()
        parm=stage.node('bind').parm('radius') if stage else self.node.parm(a.setting_parm(profile,'radius'))
        value = min(1e8,max(1e-8,parm.eval()*1.15**direction))
        if self.drag:
            parm.set(value)
            self.drag['radius']=value
            if self.drag['tool']!='grab':
                self._set_dab_amount(self.drag)
            self._preview_parts(self.drag['stage'],update_wire=False)
            self._brush_ring(self.drag['center'],self.drag['normal'],value)
        else:
            with hou.undos.group('KoAdjust Brush Radius'):
                parm.set(value)
            if stage:
                bind=stage.node('bind')
                self._brush_ring(hou.Vector3(bind.evalParmTuple('center')),hou.Vector3(bind.evalParmTuple('normal')),value)
            else:
                # Wheel events need not carry a valid pointing ray on every driver.
                if self.hover is None:
                    try:
                        self.hover=self._pick(self.pick_geometry(),event)
                    except (hou.Error,ValueError):
                        pass
                if self.hover:
                    self._brush_ring(self.hover[0],self.hover[1],value)
        self._update_brush_hud()
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

    @guarded_viewer_callback
    def onKeyEvent(self,kwargs):
        key = kwargs['ui_event'].device().keyString().lower()
        if key in ('esc','escape') and (self.drag or self.brush_adjust or self.pick_action or
                                       self.approved_rewind or self.pending_rewind):
            self._end_brush_adjust(cancel=True)
            self.pick_action = self.approved_rewind = self.pending_rewind = None
            self._finish(cancel=True)
            self.refresh()
            return True
        return False

    @guarded_viewer_callback
    def onInterrupt(self,kwargs):
        self._end_brush_adjust(cancel=True)
        self._finish(cancel=True)
        self.onEndHandleToState(kwargs)
        self._hide_brush_ring()

    @guarded_viewer_callback
    def onResume(self,kwargs):
        if self.node:
            self._expire_visibility()
            self.refresh()
            if self.hover and not self.review:
                self._brush_ring(self.hover[0],self.hover[1],self._brush_settings()[0])

    @guarded_viewer_callback
    def onExit(self,kwargs):
        self.onInterrupt(kwargs)
        if self.node is not None:
            try:
                self.node.removeEventCallback((hou.nodeEventType.ParmTupleChanged,),self._hud_callback)
            except (hou.ObjectWasDeleted,hou.OperationFailed):
                pass
        try:
            self._set_view_active(False)
            self._set_focus(set())
            self._editing()
        except hou.ObjectWasDeleted:
            pass
        self.node = self.before = None
        self._shift_smooth=False
        self.pending_rewind = self.approved_rewind = None
        self.viewer.hudInfo(show=False,panel=hou.hudPanel.ToolInfo)
        self.viewer.clearPromptMessage()

    def onCommand(self,kwargs):
        command,args = kwargs['command'],kwargs['command_args']
        if command=='tool' and args in a.TOOLS:
            self.onMenuAction({'menu_item':args})
        elif command=='review':
            self.onMenuAction({'menu_item':'review'})
        elif command=='rewind_selection' and self.node is not None:
            self.review=bool(self.node.evalParm('active_stage'))
            self.pick_action=None
            self.refresh()
        elif command=='inspect' and isinstance(args,dict):
            args.update(ready=self.node is not None,reviewing=self.review,
                        handle_visible=self.handle.visible() if self.handle else False,
                        viewer_error=self._viewer_error)

    def _shift_context_held(self):
        # A menu can open without a fresh viewer mouse event. Read the live
        # modifiers so a previous setting drag can't disable a plain RMB menu.
        mods=QtWidgets.QApplication.queryKeyboardModifiers()
        return bool(mods & QtCore.Qt.ShiftModifier) and not bool(mods & QtCore.Qt.ControlModifier)

    @guarded_viewer_callback
    def onMenuPreOpen(self,kwargs):
        if kwargs.get('menu')=='ko_adjust_menu':
            kwargs['menu_states']['enable']=not self._shift_context_held()
            if not kwargs['menu_states']['enable']:
                return
        error_item=kwargs['menu_item_states'].get('copy_viewer_error')
        if error_item is not None:
            error_item['enable']=bool(self._viewer_error)
        item=kwargs['menu_item_states'].get('toggle_move_direction')
        if item is not None:
            item['enable']=self.node is not None and a.choice(self.node,'tool')=='grab' and self.drag is None

    @guarded_viewer_callback
    def onMenuAction(self,kwargs):
        action = kwargs['menu_item']
        if self.node is None or self.drag or self.brush_adjust:
            return
        node,stage = self.node,a.active(self.node)
        try:
            if action=='copy_viewer_error':
                if self._viewer_error:
                    hou.ui.copyTextToClipboard(self._viewer_error)
                    self.viewer.setPromptMessage('KoAdjust traceback copied.')
                return
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
                if a.choice(node,'tool')!='grab':
                    return
                parm=node.parm('move_direction')
                direction='view' if parm.evalAsString()=='normal' else 'normal'
                with hou.undos.group('KoAdjust Move Direction'):
                    parm.set(direction)
                self.refresh('KoAdjust Move · '+('Surface Normal' if direction=='normal' else 'View Plane'))
                return
            elif action in ('review','prev','next') and stage:
                stages = a.history(node)
                i = stages.index(stage)+{'review':0,'prev':-1,'next':1}[action]
                a.set_rewind(node,stages[max(0,min(i,len(stages)-1))].name())
                self.review,self.pick_action = True,None
            elif action in ('new','latest'):
                if action=='latest':
                    a.set_rewind(node,'')
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
    template.bindIcon(a.ICON)
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
        (None,None,[('wire','Toggle Before Wireframe'),('copy_viewer_error','Copy Last Viewer Error')]),
    )
    for section,label,actions in sections:
        target = hou.ViewerStateMenu('ko_adjust_'+section,label) if section else menu
        for action,title in actions:
            symbol = context+'.'+action
            keys.addCommand(symbol,title,title)
            keys.addDefaultBinding(context,symbol,[a.MOVE_DIRECTION_KEY] if symbol==a.MOVE_DIRECTION_ACTION else [])
            target.addActionItem(action,title,symbol)
        if section:
            menu.addMenu(target)
    template.bindHotkeyDefinitions(keys)
    template.bindMenu(menu)
    return template
