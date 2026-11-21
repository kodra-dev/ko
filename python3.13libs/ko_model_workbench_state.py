"""Nodeless, viewport-local modeling gestures. No global shortcuts or scratch SOPs."""
import math

import hou

import ko_model_workbench as model

POINT_STATE = 'ko_model_point'


def launch(viewer, mode):
    viewer.setCurrentState(model.STATE if mode == 'grab' else POINT_STATE)


class State:
    def __init__(self, state_name, scene_viewer):
        self.viewer = scene_viewer
        self.mode = 'grab' if state_name == model.STATE else 'point'
        self.gesture = None
        self.scope = ''
        self.rebind_node = None

    def onGenerate(self, kwargs):
        source = model.displayed_source(self.viewer)
        size = source.geometry().boundingBox().sizevec().length()
        self.radius = max(size * .15, .001)
        self.drawable = hou.GeometryDrawable(self.viewer, hou.drawableGeometryType.Point,
                                             'ko_model_ghost')
        self.drawable.setParams({'color1': (1., .62, .15, 1.)})
        self.prompt()

    def prompt(self, message=''):
        usage = ('LMB drag | Wheel: radius | RMB: scope / radius' if self.mode == 'grab'
                 else 'LMB click surface | RMB: scope')
        scope = self.scope or 'unscoped'
        if self.rebind_node is not None:
            usage = 'Click new anchor | RMB: set scope or cancel rebind'
        self.viewer.setPromptMessage(message or f'Ko {model.LABELS[self.mode]} — {usage} | {scope}')

    def clear(self):
        self.gesture = None
        self.drawable.show(False)

    def onInterrupt(self, kwargs):
        self.clear()

    def onResume(self, kwargs):
        self.prompt()

    def onExit(self, kwargs):
        self.clear()
        self.viewer.clearPromptMessage()

    def onDraw(self, kwargs):
        self.drawable.draw(kwargs['draw_handle'])

    def onKeyEvent(self, kwargs):
        # Consume only the native cancel key, and only for a pending gesture.
        if self.gesture and kwargs['ui_event'].device().keyString().lower() in ('esc', 'escape'):
            self.clear()
            self.prompt('Ko Modeling — gesture canceled')
            return True
        return False

    def onMouseWheelEvent(self, kwargs):
        if self.mode != 'grab' or self.gesture is not None or self.rebind_node is not None:
            return False
        step = kwargs['ui_event'].device().mouseWheel()
        self.radius = min(1e8, max(1e-8, self.radius * (1.15 if step > 0 else 1 / 1.15)))
        self.prompt(f'Ko Grab — radius {self.radius:.4g} | LMB drag | RMB: settings')
        return True

    def onMenuAction(self, kwargs):
        action = kwargs['menu_item']
        if action == 'radial':
            self.clear()
            self.viewer.displayRadialMenu('ko_model_workbench')
            return
        if action in ('rebind', 'stop_rebind'):
            self.clear()
            if action == 'stop_rebind':
                self.rebind_node = None
                self.prompt()
                return
            try:
                source = model.displayed_source(self.viewer)
                nodes = [node for node in hou.selectedNodes() if node.parm('ko_radius') and
                         node.parm('ko_target') and node.input(0) == source]
                if len(nodes) != 1:
                    raise model.Unresolved('Display the Grab input and select exactly one downstream Ko Grab.')
                self.rebind_node = nodes[0]
                self.scope = self.rebind_node.evalParm('ko_group')
                self.prompt()
            except (ValueError, hou.Error) as exc:
                self.prompt(str(exc))
            return
        pressed, value = hou.ui.readInput('Primitive group (blank = unscoped)' if action == 'scope' else 'Brush radius (SOP units)',
                                         buttons=('Set', 'Cancel'), close_choice=1,
                                         initial_contents=self.scope if action == 'scope' else str(self.radius))
        if pressed != 0:
            return
        try:
            if action == 'scope':
                name = value.strip()
                if name:
                    model._members(model.displayed_source(self.viewer).geometry(), 'prim', name)
                self.scope = name
            else:
                radius = float(value)
                if not math.isfinite(radius) or radius <= 0:
                    raise ValueError('Radius must be positive.')
                self.radius = radius
            self.prompt()
        except (ValueError, hou.Error) as exc:
            self.prompt(str(exc))

    def _preview(self, gesture, delta):
        preview = hou.Geometry()
        preview.createPoints(tuple(p + delta * w for p, w in gesture['points']))
        self.preview = preview  # Keep the current drawable geometry alive.
        self.drawable.setGeometry(preview)
        self.drawable.setTransform(gesture['transform'])
        self.drawable.show(True)

    def _start(self, event):
        source = model.displayed_source(self.viewer)
        geo = source.geometry()
        if self.scope:
            model._members(geo, 'prim', self.scope)
        o, d = event.ray()
        hit, center = model._ray(geo, o, d, self.scope or None, 0., 1e18, 1e-6)
        if hit < 0:
            return False
        radius = self.radius if self.rebind_node is None else self.rebind_node.evalParm('ko_radius')
        spec = model.make_anchor(geo, hit, center, radius)
        prim, projected = model.resolve_anchor(geo, spec, self.scope)
        points = [(center, 1.)]
        if self.mode == 'grab' and self.rebind_node is None:
            group = geo.findPrimGroup(self.scope) if self.scope else None
            weights = model.influence(prim, projected, self.radius, group)
            stride = max(1, math.ceil(len(weights) / 1200))
            points = [(geo.point(i).position(), w) for i, w in list(weights.items())[::stride]]
        vp = self.viewer.curViewport()
        self.gesture = dict(source=source, revision=geo.vexAttribDataId(), center=center,
                            normal=d.normalized(), spec=spec, points=points,
                            viewport=vp.name(), transform=vp.modelToGeometryTransform().inverted(),
                            view=tuple(vp.viewTransform().asTuple()), delta=hou.Vector3())
        self._preview(self.gesture, hou.Vector3())
        return True

    def _drag(self, event):
        g = self.gesture
        o, d = event.ray()
        denominator = d.dot(g['normal'])
        if abs(denominator) < 1e-8:
            raise model.Unresolved('View changed during gesture.')
        delta = o + d * ((g['center'] - o).dot(g['normal']) / denominator) - g['center']
        g['delta'] = delta
        self._preview(g, delta)

    def _commit(self):
        g = self.gesture
        self.clear()
        if self.mode == 'grab' and self.rebind_node is None and g['delta'].length() < 1e-9:
            return
        source = model.displayed_source(self.viewer)
        vp = self.viewer.curViewport()
        if (source != g['source'] or source.geometry().vexAttribDataId() != g['revision'] or
                vp.name() != g['viewport'] or tuple(vp.viewTransform().asTuple()) != g['view'] or
                vp.modelToGeometryTransform().inverted().asTuple() != g['transform'].asTuple()):
            raise model.Unresolved('Input or viewport changed during gesture. Try again.')
        if self.rebind_node is not None:
            with hou.undos.group('Ko Rebind Grab'):
                model.rebind_grab(self.rebind_node, source, g['spec'], self.scope)
                model.show_node(self.viewer, source, self.rebind_node)
            self.rebind_node = None
            self.prompt()
            return
        with hou.undos.group('Ko ' + model.LABELS[self.mode]):
            node = model.create_operation(source, self.mode, g['spec'], group=self.scope,
                                          amount=0., radius=self.radius, delta=g['delta'])
            node.cook()
            if node.warnings() or node.errors():
                issue = '; '.join(node.errors() + node.warnings())
                node.destroy()
                raise model.Unresolved(issue)
            if self.mode == 'point':
                node.setTemplateFlag(True)
                node.setCurrent(True, clear_all_selected=True)
            else:
                model.show_node(self.viewer, source, node)
        self.prompt()

    def onMouseEvent(self, kwargs):
        event = kwargs['ui_event']
        reason = event.reason()
        try:
            if reason in (hou.uiEventReason.Start, hou.uiEventReason.Picked) and event.device().isLeftButton():
                consumed = self._start(event)
                if consumed and reason == hou.uiEventReason.Picked:
                    self._commit()
                return consumed
            if self.gesture is not None:
                if reason == hou.uiEventReason.Active and self.mode == 'grab' and self.rebind_node is None:
                    self._drag(event)
                elif reason == hou.uiEventReason.Changed:
                    if self.mode == 'grab' and self.rebind_node is None:
                        self._drag(event)
                    self._commit()
                return True
        except (ValueError, hou.Error) as exc:
            self.clear()
            self.prompt(str(exc))
            return True
        return False


def template(name):
    grab = name == model.STATE
    label = 'Ko Grab' if grab else 'Ko Place Point'
    tmpl = hou.ViewerStateTemplate(name, label, hou.sopNodeTypeCategory())
    tmpl.bindFactory(State)
    tmpl.bindIcon('SOP_edit' if grab else 'SOP_add')
    menu = hou.ViewerStateMenu(name + '_menu', label)
    context = 'h.pane.gview.state.sop.' + name
    hotkeys = hou.PluginHotkeyDefinitions()
    hotkeys.addContext(context, label, 'Modeling actions active only in this state')
    hotkeys.addCommandCategory(context, label, 'Ko Modeling Workbench actions')
    actions = [('radial', 'Open Modeling Workbench Radial'), ('scope', 'Set Primitive Scope…')]
    if grab:
        actions.extend((('radius', 'Set Brush Radius…'), ('rebind', 'Rebind Selected Grab…'),
                        ('stop_rebind', 'Cancel Rebind')))
    for action, title in actions:
        symbol = context + '.' + action
        hotkeys.addCommand(symbol, title, title)
        hotkeys.addDefaultBinding(context, symbol, [])
        menu.addActionItem(action, title, symbol)
    tmpl.bindHotkeyDefinitions(hotkeys)
    tmpl.bindMenu(menu)
    return tmpl
