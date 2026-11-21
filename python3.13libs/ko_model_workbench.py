"""Cook and authoring for the Ko Modeling Workbench (Houdini 22)."""
import hashlib
import heapq
import json
import math
import re
import struct
import uuid

import hou

STATE = 'ko_model_grab'
COOK = 'import ko_model_workbench as workbench\nworkbench.cook(hou.pwd())'
LABELS = dict(grab='Grab', extrude='Extrude', inset='Inset', bevel='Bevel',
              fill='Grid Fill', smooth='Smooth', point='Place Point')
DOMAINS = dict(extrude='prim', inset='prim', bevel='edge', fill='edge', smooth='prim')
IDENT = re.compile(r'[A-Za-z_][A-Za-z0-9_]*\Z')


class Unresolved(ValueError):
    """An edit cannot currently be applied safely."""


def _polygon(prim):
    return isinstance(prim, hou.Polygon) and prim.isClosed()


def exact_signature(geo):
    """Strict snapshot guard: position/number/order changes invalidate it."""
    digest = hashlib.sha256(geo.pointFloatAttribValuesAsString('P'))
    for prim in geo.iterPrims():
        if not _polygon(prim):
            raise Unresolved('Exact targets currently require closed polygons.')
        numbers = [v.point().number() for v in prim.vertices()]
        digest.update(struct.pack(f'<{len(numbers)+1}Q', len(numbers), *numbers))
    return digest.hexdigest()


def _groups(geo, domain):
    return geo.primGroups() if domain == 'prim' else geo.edgeGroups()


def _members(geo, domain, group):
    if not IDENT.fullmatch(group):
        raise Unresolved('Select a single literal group name, not a group expression.')
    found = geo.findPrimGroup(group) if domain == 'prim' else geo.findEdgeGroup(group)
    items = () if found is None else (found.prims() if domain == 'prim' else found.edges())
    if not items:
        raise Unresolved(f'Target group {group!r} is missing or empty. Rebind it.')
    return items


def _edge_key(edge):
    return tuple(sorted(p.number() for p in edge.points()))


def capture_selection(geo, domain, selection):
    kind = hou.geometryType.Primitives if domain == 'prim' else hou.geometryType.Edges
    if selection.selectionType() != kind:
        raise Unresolved(f'Select {"faces" if domain == "prim" else "edges"} first.')
    key = (lambda p: p.number()) if domain == 'prim' else _edge_key
    items = selection.prims(geo) if domain == 'prim' else selection.edges(geo)
    if not items:
        raise Unresolved('Nothing selected. The workbench never interprets this as everything.')
    wanted = {key(item) for item in items}
    names = []
    for group in _groups(geo, domain):
        if not IDENT.fullmatch(group.name()):
            continue
        members = group.prims() if domain == 'prim' else group.edges()
        if len(members) == len(wanted) and wanted == {key(item) for item in members}:
            names.append(group.name())
    if len(names) == 1:
        return {'kind': 'group', 'group': names[0]}
    return {'kind': 'exact', 'selection': selection.selectionString(geo, force_numeric=True),
            'signature': exact_signature(geo)}


def _selected(geo, domain, spec, group_override):
    if group_override or spec['kind'] == 'group':
        name = group_override or spec['group']
        return name, _members(geo, domain, name)
    if spec['kind'] != 'exact' or not spec.get('selection'):
        raise Unresolved('Target data is missing. Set a target group.')
    if exact_signature(geo) != spec['signature']:
        raise Unresolved('Exact selection input changed. Rebind to a named target group.')
    kind = hou.geometryType.Primitives if domain == 'prim' else hou.geometryType.Edges
    sel = hou.Selection(geo, kind, spec['selection'])
    items = sel.prims(geo) if domain == 'prim' else sel.edges(geo)
    if not items:
        raise Unresolved('Exact selection is empty.')
    return spec['selection'], items


def _single_chain(pairs, *, closed):
    links = {}
    for a, b in pairs:
        links.setdefault(a, set()).add(b)
        links.setdefault(b, set()).add(a)
    if not links or (closed and any(len(v) != 2 for v in links.values())):
        return False
    todo = [next(iter(links))]
    seen = set(todo)
    while todo:
        for n in links[todo.pop()] - seen:
            seen.add(n)
            todo.append(n)
    return len(seen) == len(links)


def _single_face_region(faces):
    ids = {p.number() for p in faces}
    edges = {}
    links = {n: set() for n in ids}
    for face in faces:
        if not _polygon(face):
            return False
        numbers = [p.number() for p in face.points()]
        for a, b in zip(numbers, numbers[1:] + numbers[:1]):
            edge = tuple(sorted((a, b)))
            other = edges.setdefault(edge, face.number())
            if other != face.number():
                links[other].add(face.number())
                links[face.number()].add(other)
    todo = [min(ids)]
    seen = set(todo)
    while todo:
        for n in links[todo.pop()] - seen:
            seen.add(n)
            todo.append(n)
    return seen == ids


def _verb(name, geo, **parms):
    verb = hou.sopNodeTypeCategory().nodeVerb(name)
    if verb is None:
        raise Unresolved(f'{name} is unavailable on this Houdini build.')
    verb.setParms(parms)
    out = hou.Geometry()
    verb.execute(out, [geo])
    return out


def _prefix(uid):
    if not re.fullmatch(r'[a-f0-9]{32}', uid):
        raise Unresolved('Invalid operation identity.')
    return 'ko_model_' + uid


def feature_groups(uid):
    return tuple(_prefix(uid) + suffix for suffix in ('_cap', '_side', '_rim'))


def topology_action(geo, mode, spec, group_override, uid, amount, divisions, corner):
    group, items = _selected(geo, DOMAINS[mode], spec, group_override)
    if DOMAINS[mode] == 'prim' and not _single_face_region(items):
        raise Unresolved('Select a single edge-connected region of polygon faces.')
    if not math.isfinite(amount) or (mode in ('bevel', 'smooth') and amount <= 0):
        raise Unresolved('Set a valid amount.')
    divisions = max(1, min(divisions, 32))
    if mode in ('extrude', 'inset'):
        if amount == 0:
            raise Unresolved('Set a nonzero amount.')
        cap, side, rim = feature_groups(uid)
        if geo.findPrimGroup(cap) or geo.findPrimGroup(side) or geo.findEdgeGroup(rim):
            raise Unresolved('Operation identity already exists upstream. Assign a new ID to the copied producer.')
        out = _verb('polyextrude::2.0', geo, group=group,
                    dist=amount if mode == 'extrude' else 0.,
                    inset=amount if mode == 'inset' else 0., divs=divisions,
                    outputfrontgrp=1, frontgrp=cap, outputsidegrp=1, sidegrp=side,
                    outputfrontseamgrp=1, frontseamgrp=rim)
        _members(out, 'prim', cap)
        _members(out, 'prim', side)
        _members(out, 'edge', rim)
        return out
    if mode == 'bevel':
        if any(not 1 <= len(edge.prims()) <= 2 for edge in items):
            raise Unresolved('Bevel target includes a non-manifold edge.')
        out = _verb('polybevel::3.0', geo, group=group, grouptype=3,
                    offset=amount, divisions=divisions)
        if out.pointCount() == geo.pointCount() and out.primCount() == geo.primCount():
            raise Unresolved('Bevel did not create new topology at this width.')
        return out
    if mode == 'smooth':
        return _verb('smooth::2.0', geo, group=group, attributes='P',
                     contrainedboundary=2, strength=amount)
    if len(items) < 4 or len(items) % 2 or any(len(e.prims()) != 1 for e in items):
        raise Unresolved('Grid Fill needs an even, closed boundary with at least four open edges.')
    if not _single_chain((_edge_key(e) for e in items), closed=True):
        raise Unresolved('Grid Fill needs exactly one closed boundary loop.')
    patch = _prefix(uid) + '_patch'
    if geo.findPrimGroup(patch):
        raise Unresolved('Patch identity already exists upstream.')
    out = _verb('polyfill', geo, group=group, fillmode=5, completeloops=0,
                corneroffset=corner, patchgrouptoggle=1, patchgroup=patch)
    if any(len(f.vertices()) != 4 for f in _members(out, 'prim', patch)):
        raise Unresolved('PolyFill did not produce an all-quad grid.')
    return out


def make_anchor(geo, hit, position, radius):
    prim = geo.prim(hit)
    if prim is None or not _polygon(prim):
        raise Unresolved('Pick a closed polygon.')
    return {'kind': 'anchor', 'position': tuple(position), 'normal': tuple(prim.normal()),
            'tolerance': radius * .25}


def _ray(geo, origin, direction, pattern, start, end, epsilon):
    pos, normal, uvw = hou.Vector3(), hou.Vector3(), hou.Vector3()
    hit = geo.intersect(origin, direction, pos, normal, uvw, pattern=pattern,
                        min_hit=start, max_hit=end, tolerance=epsilon)
    return hit, pos


def resolve_anchor(geo, spec, scope):
    """Bounded normal-line projection. Abstain on more than one compatible sheet."""
    if scope and (not IDENT.fullmatch(scope) or not geo.findPrimGroup(scope) or
                  not geo.findPrimGroup(scope).primCount()):
        raise Unresolved(f'Grab scope {scope!r} is missing or empty.')
    n = hou.Vector3(spec['normal'])
    p = hou.Vector3(spec['position'])
    limit = float(spec['tolerance'])
    if limit <= 0 or not math.isfinite(limit) or n.length() < .5:
        raise Unresolved('Invalid anchor frame or search distance.')
    n = n.normalized()
    eps = max(limit * 1e-5, 1e-9)
    origin = p + n * (limit + eps * 4)
    # Exclude tested primitive numbers rather than advancing the ray through a
    # hit; this exposes coincident independent shells instead of skipping them.
    excluded, matches = [], []
    for _ in range(24):
        pattern = (scope or '*') + ''.join(f' ^{i}' for i in excluded)
        i, q = _ray(geo, origin, -n, pattern, eps, 2 * limit + 8 * eps, eps)
        if i < 0:
            break
        if i in excluded:
            raise Unresolved('Could not isolate projection candidates.')
        excluded.append(i)
        prim = geo.prim(i)
        if not _polygon(prim) or prim.normal().dot(n) < .6:
            continue
        if not matches:
            matches.append((prim, q))
        else:
            first, pos = matches[0]
            # A hit on a tessellation edge may be reported by both adjacent faces.
            shared = {v.number() for v in first.points()} & {v.number() for v in prim.points()}
            if not shared or (q - pos).length() > eps * 8:
                raise Unresolved('Grab anchor matches multiple surfaces. Narrow its scope or rebind.')
    else:
        raise Unresolved('Grab anchor projection exceeded the candidate budget.')
    if not matches:
        raise Unresolved('Grab anchor lost its surface. Rebind or adjust Search Distance.')
    return matches[0]


def influence(prim, center, radius, group=None, max_points=25000):
    """Radius-bounded edge Dijkstra. No traversal through coincident, unshared points."""
    if not math.isfinite(radius) or radius <= 0:
        raise Unresolved('Brush radius must be positive.')
    geo = prim.geometry()
    best = {p.number(): (p.position() - center).length() for p in prim.points()}
    if min(best.values()) >= radius:
        raise Unresolved('Brush is smaller than the local point spacing. Increase its radius.')
    queue = [(d, i) for i, d in best.items() if d < radius]
    heapq.heapify(queue)
    weights = {}
    while queue:
        distance, i = heapq.heappop(queue)
        if best[i] != distance:
            continue
        if len(weights) >= max_points:
            raise Unresolved('Brush region exceeded its point budget. Use a smaller radius.')
        p = geo.point(i)
        position = p.position()
        t = 1 - distance / radius
        weights[i] = t * t * (3 - 2 * t)
        neighbors = set()
        for face in p.prims():
            if not _polygon(face) or (group is not None and not group.contains(face)):
                continue
            ring = [v.number() for v in face.points()]
            j = ring.index(i)
            neighbors.update((ring[j - 1], ring[(j + 1) % len(ring)]))
        for j in neighbors:
            d = distance + (geo.point(j).position() - position).length()
            if d < radius and d < best.get(j, math.inf):
                best[j] = d
                heapq.heappush(queue, (d, j))
    return weights


def cook(node):
    geo = node.geometry()  # Python SOP starts with an editable copy of input 0.
    if not node.inputs() or node.inputs()[0] is None:
        raise hou.NodeWarning('Connect the operation to its source mesh.')
    mode = None
    try:
        payload = json.loads(node.evalParm('ko_target'))
        if payload['version'] != 1:
            raise Unresolved('Unsupported target data version.')
        mode, spec = payload['mode'], payload['target']
        group = node.evalParm('ko_group').strip()
        if mode == 'grab':
            spec = dict(spec, tolerance=node.evalParm('ko_search'))
            prim, center = resolve_anchor(geo, spec, group)
            weights = influence(prim, center, node.evalParm('ko_radius'), geo.findPrimGroup(group) if group else None)
            delta = hou.Vector3(node.evalParmTuple('ko_delta'))
            if not all(map(math.isfinite, delta)):
                raise Unresolved('Invalid Grab displacement.')
            for i, weight in weights.items():
                point = geo.point(i)
                point.setPosition(point.position() + delta * weight)
        elif mode == 'point':
            # This operation deliberately emits a separate point branch.
            prim, center = resolve_anchor(geo, dict(spec, tolerance=node.evalParm('ko_search')), group)
            position = center + prim.normal() * node.evalParm('ko_amount')
            if not all(map(math.isfinite, position)):
                raise Unresolved('Invalid point offset.')
            frame = hou.Vector3(0, 0, 1).matrixToRotateTo(prim.normal())
            orient = tuple(hou.Quaternion(frame))
            geo.clear()
            attr = geo.addAttrib(hou.attribType.Point, 'orient', (0., 0., 0., 1.))
            point = geo.createPoint()
            point.setPosition(position)
            point.setAttribValue(attr, orient)
        else:
            out = topology_action(geo, mode, spec, group, node.evalParm('ko_uid'),
                                  node.evalParm('ko_amount'), node.evalParm('ko_divs'), node.evalParm('ko_corner'))
            geo.clear()
            geo.merge(out)
    except (ValueError, KeyError, TypeError, hou.OperationFailed, hou.GeometryPermissionError) as exc:
        if mode == 'point':
            geo.clear()  # A failed placement must never instance onto every source point.
        raise hou.NodeWarning(f'Ko Modeling: {exc}') from None


def create_operation(source, mode, spec, *, amount=None, divisions=1, group='', radius=.1,
                     delta=(0, 0, 0), identity=None):
    """Create without changing display flags; authoring handles the transaction."""
    node = source.parent().createNode('python', 'ko_' + mode)
    try:
        ptg = node.parmTemplateGroup()
        ptg.append(hou.StringParmTemplate('ko_target', 'Target Data (versioned)', 1,
                   default_value=(json.dumps({'version': 1, 'mode': mode, 'target': spec}),)))
        ptg.append(hou.StringParmTemplate('ko_uid', 'Operation ID', 1, default_value=(identity or uuid.uuid4().hex,)))
        ptg.append(hou.StringParmTemplate('ko_group', 'Target Group / Anchor Scope', 1, default_value=(group,)))
        if mode != 'grab':
            ptg.append(hou.FloatParmTemplate('ko_amount', 'Amount', 1, default_value=(amount if amount is not None else .1,)))
        if mode in DOMAINS:
            ptg.append(hou.IntParmTemplate('ko_divs', 'Divisions', 1, default_value=(divisions,), min=1, max=32))
            ptg.append(hou.IntParmTemplate('ko_corner', 'Grid Corner Offset', 1))
        if mode in ('grab', 'point'):
            ptg.append(hou.FloatParmTemplate('ko_search', 'Anchor Search Distance', 1, default_value=(spec['tolerance'],)))
        if mode == 'grab':
            ptg.append(hou.FloatParmTemplate('ko_radius', 'Brush Radius', 1, default_value=(radius,)))
            ptg.append(hou.FloatParmTemplate('ko_delta', 'Displacement (SOP Local)', 3, default_value=tuple(delta),
                                           naming_scheme=hou.parmNamingScheme.XYZW))
        node.setParmTemplateGroup(ptg)
        node.setInput(0, source)
        node.parm('python').set(COOK)
        node.moveToGoodPosition()
    except Exception:
        node.destroy()
        raise
    return node


def rebind_grab(node, source, spec, scope):
    """Replace only a Grab's binding; caller owns the undo transaction."""
    if node.input(0) != source or node.parm('ko_target') is None:
        raise Unresolved('Select a Ko Grab connected to the displayed input.')
    payload = json.loads(node.evalParm('ko_target'))
    if payload['version'] != 1 or payload['mode'] != 'grab':
        raise Unresolved('The selected operation is not a compatible Ko Grab.')
    geo = source.geometry()
    prim, center = resolve_anchor(geo, spec, scope)
    influence(prim, center, node.evalParm('ko_radius'), geo.findPrimGroup(scope) if scope else None)
    payload['target'] = spec
    node.setParms({'ko_target': json.dumps(payload), 'ko_group': scope, 'ko_search': spec['tolerance']})


def displayed_source(viewer):
    network = viewer.pwd()
    if network.childTypeCategory() != hou.sopNodeTypeCategory():
        raise Unresolved('Enter a Geometry object first.')
    source = network.displayNode()
    if source is None or source.geometry() is None:
        raise Unresolved('Display a polygon SOP first.')
    return source


def show_node(viewer, source, node):
    node.setDisplayFlag(True)
    if source.isRenderFlagSet():
        node.setRenderFlag(True)
    node.setCurrent(True, clear_all_selected=True)


def radial(kwargs, mode):
    viewer = kwargs.get('pane')
    if not isinstance(viewer, hou.SceneViewer):
        viewer = hou.ui.paneTabOfType(hou.paneTabType.SceneViewer)
    if viewer is None:
        return
    try:
        source = displayed_source(viewer)
        if mode in ('grab', 'point'):
            from ko_model_workbench_state import launch
            launch(viewer, mode)
            return
        geo = source.geometry()
        if mode == 'bevel_rim':
            target = source.parm('ko_target')
            if target is None or json.loads(target.eval())['mode'] not in ('extrude', 'inset'):
                raise Unresolved('Display the Ko Extrude or Inset whose rim you want to bevel.')
            rim = feature_groups(source.evalParm('ko_uid'))[2]
            _members(geo, 'edge', rim)
            spec = {'kind': 'group', 'group': rim}
            mode = 'bevel'
        else:
            selection = viewer.currentGeometrySelection()
            if selection is None or tuple(selection.nodes()) != (source,):
                raise Unresolved('Select components on the displayed SOP first.')
            spec = capture_selection(geo, DOMAINS[mode], selection.selections()[0])
        amount, divisions = .1, 1
        if mode != 'fill':
            hint = (f'Live group: {spec["group"]}' if spec['kind'] == 'group' else
                    'Exact selection: this edit stops if the input changes. Use a named group to keep it live.')
            pressed, text = hou.ui.readInput(f'{LABELS[mode]} amount\n{hint}', buttons=('Apply', 'Cancel'),
                                             close_choice=1, initial_contents='0.1')
            if pressed != 0:
                return
            amount = float(text)
            divisions = 2 if mode == 'bevel' else 1
        with hou.undos.group('Ko ' + LABELS[mode]):
            node = create_operation(source, mode, spec, amount=amount, divisions=divisions)
            node.cook()
            if node.warnings() or node.errors():
                issue = '; '.join(node.errors() + node.warnings())
                node.destroy()
                raise Unresolved(issue)
            show_node(viewer, source, node)
            if mode in ('extrude', 'inset'):
                cap = feature_groups(node.evalParm('ko_uid'))[0]
                kind = hou.geometryType.Primitives
                viewer.setCurrentNode(node, pick_node=True)
                viewer.setPickGeometryType(kind)
                viewer.setCurrentGeometrySelection(kind, (node,), (hou.Selection(node.geometry(), kind, cap),))
    except (ValueError, hou.Error) as exc:
        hou.ui.setStatusMessage(str(exc), severity=hou.severityType.Warning)
