"""Houdini's #bind directive and @-binding vocabulary.

Transcribed from SideFX's "OpenCL for VEX users" help page ($HFS help: vex/ocl).
A binding may also come from the node's Bindings multiparm, which wins over
#bind, so an @name that no #bind declares is never treated as an error.
"""
import re
from dataclasses import dataclass, field

# binding word -> (class used for @-methods, description)
BINDING_TYPES = {
    'parm': ('parm', 'A constant parameter value, the same for all work items. A type flag such as `float`, `float3` or `int` selects its type.'),
    'point': ('attribute', 'A point attribute of the geometry.'),
    'prim': ('attribute', 'A primitive attribute of the geometry.'),
    'primitive': ('attribute', 'A primitive attribute of the geometry.'),
    'vertex': ('attribute', 'A vertex attribute of the geometry.'),
    'detail': ('attribute', 'A detail (global) attribute of the geometry.'),
    'global': ('attribute', 'A detail (global) attribute of the geometry.'),
    'ramp': ('ramp', 'A ramp, as a 1D float array interpolated on lookup. The `float` or `float3` flag selects a scalar or vector ramp.'),
    'volume': ('volume', 'A volume primitive of the geometry.'),
    'vdb': ('vdb', 'A VDB primitive of the geometry (NanoVDB layout on the GPU). Bind as `float` or `float3`; without a type it is `any` with fewer methods.'),
    'option': ('option', 'A DOP Option (DOPs only). `geo=` selects the data, `name=` the option.'),
    'sfield': ('field', 'A scalar DOP field (DOPs only).'),
    'vfield': ('field', 'A vector DOP field (DOPs only).'),
    'mfield': ('field', 'A matrix DOP field (DOPs only).'),
    'scalarfield': ('field', 'A scalar DOP field (DOPs only).'),
    'vectorfield': ('field', 'A vector DOP field (DOPs only).'),
    'matrixfield': ('field', 'A matrix DOP field (DOPs only).'),
    'layer': ('layer', 'A Copernicus (COP) layer.'),
    'context': ('parm', 'A COP Context Option (COPs only).'),
}

DECORATIONS = {'&': 'write', '?': 'opt', '!': 'noread'}

# Boolean flags and their documented default. Each also has a "no" form.
FLAGS = {
    'read': 'Defaults to read. A readable binding has its OpenCL buffer initialized from the original value. If not readable, overwrite it, or arbitrary data is present.',
    'write': 'Defaults to nowrite. Only writable bindings are written back to the CPU.',
    'opt': 'Defaults to noopt. An optional binding need not exist on the geometry. `HAS_name` is defined when it does; with `def`, a missing binding is replaced by a default value.',
    'def': 'Defaults to def. If an optional binding is missing, a parameter is bound in its place, so `@name` reads the default value instead.',
    'fieldoffsets': 'Default fieldoffsets. For DOP field bindings, enable the offset parameters for this field.',
    'forcealign': 'Default forcealign (noforcealign for the first writable field or volume). Requires all fields or volumes to have the same resolution.',
    'resolution': 'Default noresolution (resolution for the first writable volume). Include the resolution of the field or volume; needed for bounds checking.',
    'voxelsize': 'Default novoxelsize. Include the voxel size of the volume (enables `@name.voxelsize`).',
    'xformtoworld': 'Default noxformtoworld. Include the transform from index space to geometry (SOP) space (enables `@name.xformtoworld` and `indexToWorld`).',
    'xformtovoxel': 'Default noxformtovoxel. Include the transform from geometry (SOP) space to index space (enables `@name.xformtovoxel` and `worldToIndex`).',
    'uipromote': 'Default uipromote (nouipromote for layers). Controls whether the generate-parameter button creates a spare parameter for this binding.',
    'osd': 'Default noosd. For point float attributes, build Open Subdiv data and enable the `osd_*` methods.',
    'bvh': 'Default nobvh. Build a BVH over the input triangles and enable `xyzdist`, `minpos`, `closest_idx` and their `_max` variants. Only valid on a float3 point or vertex attribute.',
    'pointbvh': 'Default nopointbvh. Build a BVH over the points and enable `nearpoint`, `nearpoints`, `minpos` and their `_max` variants. Only valid on a float3 point attribute; not combined with `bvh`.',
}

# key=value flags
KEY_FLAGS = {
    'val': 'Default value for the parameter, volume or VDB (default 0). A scalar is repeated for vectors; use braces for vectors: `val={1, 2, 3}`.',
    'name': 'The attribute, volume or VDB to bind to (defaults to the binding name). Prefix `group:` binds a group, `topo:` a topology attribute.',
    'input': 'In SOPs, the input to bind to (default 0). Only the first input can be written to.',
    'geo': 'In DOPs, the simulation geometry to bind to, relative to the simulation object (default Geometry).',
    'geometry': 'In DOPs, the simulation geometry to bind to, relative to the simulation object (default Geometry).',
    'port': 'In COPs, the input or output port to bind to (defaults to the binding name).',
    'portname': 'In COPs, the input or output port to bind to (defaults to the binding name).',
    'border': 'For layers: CONSTANT, CLAMP, MIRROR, WRAP or INPUT (default INPUT, the layer\'s own border).',
    'pointbvhmask': 'Restricts a point BVH to points whose integer point attribute is non-zero. Only meaningful with `pointbvh`.',
}
BORDERS = ('CONSTANT', 'CLAMP', 'MIRROR', 'WRAP', 'INPUT')

TYPE_FLAG = re.compile(r'^(u?(char|short|int|long)|half|float|double|fpreal|exint|bool)(\d+)?(\[\]|\?)?$|^any$|^mat[234]$')
TYPE_FLAG_DOC = ('An OpenCL type selects the vector size and precision, such as `float`, `float3` or `int`. '
                 '`float[]` binds an array attribute; `float?` and `int?` bind attributes of any tuple size (see `tuplesize`).')

RUNOVER_MODES = {
    'attribute': 'Run once per element of the first writable attribute.',
    'field': 'Run once per voxel of the first writable DOP field.',
    'vdb': 'Run once per voxel of the first writable VDB.',
    'volume': 'Run once per voxel of the first writable volume.',
    'layer': 'Run once per buffer element of the first writable layer.',
    'worksets': 'Run work items given by a pair of detail array attributes (the equivalent of For Each Number).',
}

DIRECTIVES = {
    'bind': '`#bind type name flags...` binds an attribute, parameter, volume, VDB, ramp, field or layer for `@name` use.',
    'runover': '`#runover mode` overrides the node\'s Run Over: attribute, field, vdb, volume, layer or worksets.',
    'include': 'Include a header, searched in `$HFS/houdini/ocl/include` and the node\'s include paths.',
    'import': 'Include a header once (like `#include` with an include guard).',
    'define': None, 'undef': None, 'if': None, 'ifdef': None, 'ifndef': None, 'elif': None,
    'else': None, 'endif': None, 'pragma': None, 'error': None,
}

KERNEL_MARKERS = {
    'KERNEL': 'Generate the kernel function, its parameters and the per-work-item boilerplate from the #bind directives; the block is its body.',
    'WRITEBACK': 'Generate the write-back kernel (SOPs with write-back kernels enabled); the block is its body.',
}

# name -> (type, where valid, description)
GLOBAL_BINDINGS = {
    'ix': ('int', 'volumes, VDBs, layers or fields', 'The integer x index of the voxel or buffer element being processed.'),
    'iy': ('int', 'volumes, VDBs, layers or fields', 'The integer y index of the voxel or buffer element being processed.'),
    'iz': ('int', 'volumes, VDBs or fields', 'The integer z index of the voxel being processed.'),
    'ixy': ('int2', 'volumes, VDBs, layers or fields', 'The integer xy index being processed.'),
    'ixyz': ('int3', 'volumes, VDBs or fields', 'The integer xyz index being processed.'),
    'xres': ('int', 'layers', 'The output buffer x resolution.'),
    'yres': ('int', 'layers', 'The output buffer y resolution.'),
    'res': ('int2', 'layers', 'The output buffer resolution.'),
    'tilesize': ('int2 / int4', 'layers / VDBs', 'The tile size; the kernel runs once per tile. For VDBs (with vdb tiles), xyz is the tile size and w the leaf count.'),
    'elemnum': ('int', 'volumes, attributes, VDBs, aligned fields', 'An integer index of the element being processed. For VDBs it varies with the topology.'),
    'P': ('float2/3', 'layers or fields (otherwise a P attribute binding)', 'The image/DOP/model space position of the processed voxel. For layers, `.image` (default), `.texture`, `.pixel` and `.world` select a space.'),
    'dPdx': ('float2', 'layers', 'The image-space change when the buffer x coordinate changes by one. `.image`, `.texture`, `.pixel` select a space.'),
    'dPdy': ('float2', 'layers', 'The image-space change when the buffer y coordinate changes by one. `.image`, `.texture`, `.pixel` select a space.'),
    'dPdxy': ('float2', 'layers', 'The image-space size of the output buffer element: (@dPdx.x, @dPdy.y).'),
    'Time': ('float', 'with Include Time on the node', 'The current time in seconds. Makes the node time dependent.'),
    'TimeInc': ('float', 'with Include Timestep on the node', 'The time step: usually 1/$FPS in SOPs, the simulation step in DOPs.'),
    'SimFrame': ('int', 'with Include Simulation Frame on the node', 'The current simulation frame, $SF.'),
}


def _methods(*rows):
    """(signature, description, requirement) rows -> {method name: row}."""
    result = {}
    for signature, description, requirement in rows:
        name = re.match(r'[A-Za-z_]\w*', signature)[0]
        result.setdefault(name, (signature, description, requirement))
    return result


_COMMON_ATTRIBUTE = (
    ('data', 'The raw attribute data: all elements flattened into one array. Prefer the bounds-checked methods.', ''),
    ('len', 'The number of points, primitives or vertices.', ''),
    ('get', 'The attribute value at the current index. Same as `@name`.', 'readable, not an array, matching index'),
    ('getAt(idx)', 'The attribute value at a specific index. Same as `@name(idx)`.', 'readable, not an array'),
    ('set(val)', 'Set the attribute at the current index.', 'writable, not an array, matching index'),
    ('setAt(idx, val)', 'Set the attribute at the given index. Not thread-safe: never let two work items write the same element.', 'writable, not an array'),
    ('tuplesize', 'The size of each element: 1 for float, 3 for a vector.', ''),
    ('tuple', 'Pointer to the current item\'s components; index with `[]` and bounds-check against `tuplesize`.', 'matching index'),
    ('tupleAt(idx)', 'Pointer to the components of the item at `idx`.', ''),
    ('bound', 'Whether the attribute is bound: 0 or 1 for optional bindings, always 1 otherwise.', ''),
    ('comp(compidx)', 'For an array attribute, one component of the current index\'s array.', 'array, matching index'),
    ('entries', 'The length of the current index\'s array.', 'array, matching index'),
    ('compAt(idx, compidx)', 'For an array attribute, one component of element `idx`\'s array.', 'array'),
    ('entriesAt(idx)', 'The length of element `idx`\'s array.', 'array'),
    ('osd_facecount()', 'The number of coarse faces in the Open Subdiv hull.', '`osd` flag'),
    ('osd_patchcount()', 'The total number of base-level patches.', '`osd` flag'),
    ('osd_firstpatch(face)', 'The first patch generated by coarse face `face`.', '`osd` flag'),
    ('osd_patchcount_face(face)', 'The number of patches generated by coarse face `face`.', '`osd` flag'),
    ('osd_lookuppatch(face, face_u, face_v, patch, patch_u, patch_v)', 'Map face coordinates to patch coordinates. Returns 1 on success.', '`osd` flag'),
    ('osd_lookupface(patch, patch_u, patch_v, face, face_u, face_v)', 'Map patch coordinates back to face coordinates. Returns 1 on success.', '`osd` flag'),
    ('osd_limitsurface(patch, u, v)', 'The attribute evaluated on the limit surface at patch coordinates u, v.', '`osd` flag; tuple size 1, 2, 3, 4 or 16'),
    ('osd_limit(patch, u, v, du, dv)', 'The limit-surface value, writing the u and v derivatives to `du` and `dv`.', '`osd` flag; tuple size 1, 2, 3, 4 or 16'),
    ('osd_limitnormal(patch, u, v)', 'The normalized limit-surface normal.', '`osd` flag; float3'),
    ('xyzdist(O, closetri, closeuv)', 'The closest position on the triangles to `O`; writes the triangle index and barycentric (u, v, w). Returns a position, not a distance.', '`bvh` flag; float3 point or vertex attribute'),
    ('xyzdist_max(O, tmax, closetri, closeuv)', 'Like `xyzdist` within distance `tmax`; returns {0,0,0} and closetri -1 if nothing is found.', '`bvh` flag'),
    ('minpos(O)', 'The closest position to `O` on the triangles (`bvh`) or points (`pointbvh`).', '`bvh` or `pointbvh` flag'),
    ('minpos_max(O, tmax)', 'Like `minpos` within distance `tmax`.', '`bvh` or `pointbvh` flag'),
    ('closest_idx(O, closetri)', 'Which corner (0, 1, 2) of the closest triangle is nearest to `O`, or -1.', '`bvh` flag'),
    ('closest_idx_max(O, tmax, closetri)', 'Like `closest_idx` within distance `tmax`.', '`bvh` flag'),
    ('nearpoint(O, closedist)', 'The nearest point number to `O`, or -1; writes the distance to `closedist`.', '`pointbvh` flag; float3 point attribute'),
    ('nearpoint_max(O, tmax, closedist)', 'Like `nearpoint` within distance `tmax`.', '`pointbvh` flag'),
    ('nearpoints(O, tmax, indices, dists, k)', 'Up to `k` nearest points within `tmax`, sorted; writes point numbers and squared distances. Returns the count.', '`pointbvh` flag'),
)

METHODS = {
    'parm': {},
    'attribute': _methods(*_COMMON_ATTRIBUTE),
    'ramp': _methods(
        ('data', 'The float array of ramp samples.', ''),
        ('len', 'The number of ramp samples.', ''),
        ('getAt(pos)', 'Look up the ramp at 0..1 position `pos`, interpolating. Same as `@name(pos)`.', ''),
    ),
    'field': _methods(
        ('data', 'The raw float data of the field.', ''),
        ('stride_x', 'The data step in x. Vector/matrix fields use `stride_x_y` style names.', ''),
        ('stride_y', 'The data step in y.', ''), ('stride_z', 'The data step in z.', ''),
        ('stride_offset', 'Offset from `data` to the 0,0,0 voxel.', ''),
        ('xres', 'The x resolution.', ''), ('yres', 'The y resolution.', ''), ('zres', 'The z resolution.', ''),
        ('get', 'The value at the current voxel. Same as `@name`.', 'aligned'),
        ('getAt(x, y, z)', 'The voxel at index x, y, z.', ''),
        ('set(val)', 'Set the current voxel.', 'aligned'),
        ('setAt(x, y, z, val)', 'Set the voxel at x, y, z.', ''),
        ('sample(xyz)', 'Evaluate the field in index space at `float3 xyz`.', ''),
    ),
    'volume': _methods(
        ('data', 'The raw volume data, indexed with the `stride_` values. Prefer `getAt`.', ''),
        ('bound', 'Whether the volume is bound: 0 or 1 for optional volumes, always 1 otherwise.', ''),
        ('stride_x', 'The data step in x.', ''), ('stride_y', 'The data step in y.', ''), ('stride_z', 'The data step in z.', ''),
        ('stride_offset', 'Offset of the 0,0,0 voxel from the start of `data`.', ''),
        ('xres', 'The x resolution (data may be padded; index with the strides).', ''),
        ('yres', 'The y resolution.', ''), ('zres', 'The z resolution.', ''),
        ('voxelsize', 'The voxel size; also `voxelsize_x/y/z`.', '`voxelsize` flag'),
        ('voxelsize_x', 'The voxel size along x.', '`voxelsize` flag'),
        ('voxelsize_y', 'The voxel size along y.', '`voxelsize` flag'),
        ('voxelsize_z', 'The voxel size along z.', '`voxelsize` flag'),
        ('xformtoworld', 'An fpreal16 mapping voxel index to model/SOP space.', '`xformtoworld` flag'),
        ('xformtovoxel', 'An fpreal16 mapping model/SOP space to index space.', '`xformtovoxel` flag'),
        ('get', 'The value at the current voxel. Same as `@name`.', 'readable, matching index'),
        ('valid(x, y, z)', 'True if voxel x, y, z is inside the volume.', ''),
        ('getAt(x, y, z)', 'The value at voxel x, y, z, clamped to the volume.', 'readable'),
        ('set(val)', 'Set the current voxel.', 'writable, matching index'),
        ('setAt(x, y, z, val)', 'Set voxel x, y, z; a no-op out of range.', 'writable'),
        ('sample(xyz)', 'Trilinear sample at index-space fpreal3 `xyz`, clamped.', 'readable'),
        ('sample2d(xy)', 'Bilinear sample at index-space fpreal2 `xy`, clamped.', 'readable'),
    ),
    'vdb': _methods(
        ('data', 'The raw NanoVDB data (see CNanoVDB.h). Prefer the methods.', ''),
        ('bound', 'Whether the VDB is bound: 0 or 1 for optional VDBs, always 1 otherwise.', ''),
        ('acc', 'Pointer to the canonical read accessor; null for a missing optional VDB.', ''),
        ('storage', 'The grid type (a CNANOVDB_GRIDTYPE define, set at compile time).', ''),
        ('toscalar(xyz)', 'A float3 converted to a scalar for this VDB\'s type: .x for float VDBs, the length for vector VDBs.', ''),
        ('voxelsize', 'The voxel size; also `voxelsize_x/y/z`.', '`voxelsize` flag'),
        ('voxelsize_x', 'The voxel size along x.', '`voxelsize` flag'),
        ('voxelsize_y', 'The voxel size along y.', '`voxelsize` flag'),
        ('voxelsize_z', 'The voxel size along z.', '`voxelsize` flag'),
        ('xformtoworld', 'An fpreal16 transform from index to model/SOP space.', '`xformtoworld` flag'),
        ('xformtovoxel', 'An fpreal16 transform from model/SOP space to index space.', '`xformtovoxel` flag'),
        ('get', 'The value at the current index. Same as `@name`.', 'readable, matching index'),
        ('leaf', 'Pointer to the current leaf node, or null.', 'matching index'),
        ('pos', 'The model/SOP position of the current index.', 'matching index'),
        ('active', 'Whether the current index is active.', '`float` or `float3` binding, matching index'),
        ('activeAt(x, y, z)', 'Whether index x, y, z is active.', '`float` or `float3` binding'),
        ('valid(x, y, z)', 'Whether index x, y, z is active.', '`float` or `float3` binding'),
        ('getAt(x, y, z)', 'The value at index x, y, z (the background value where inactive).', 'readable'),
        ('leafAt(x, y, z)', 'Pointer to the leaf containing x, y, z, or null.', ''),
        ('leafAtIndex(idx)', 'Pointer to leaf number `idx`, or null. Leaf order is not deterministic.', ''),
        ('leafCount', 'The number of leaves.', ''),
        ('set(val)', 'Set the current voxel. Cannot change topology.', 'writable, matching index'),
        ('setAt(x, y, z, val)', 'Set voxel x, y, z; does nothing outside the topology.', 'writable'),
        ('sample(xyz)', 'Trilinear sample at an index-space position.', 'readable'),
        ('worldSample(xyz)', 'Trilinear sample at a model/SOP-space position.', 'readable'),
        ('worldGradient(xyz)', 'The gradient at a model/SOP-space position, in model/SOP space.', '`float` binding, readable'),
        ('worldToIndex(xyz)', 'A world position as an index coordinate.', '`xformtovoxel` flag'),
        ('indexToWorld(xyz)', 'An index coordinate as a world position.', '`xformtoworld` flag'),
        ('worldToIndexDir(xyz)', 'A world vector in index space.', '`xformtovoxel` flag'),
        ('indexToWorldDir(xyz)', 'An index-space vector in world space.', '`xformtoworld` flag'),
    ),
    'option': _methods(
        ('get', 'The option value. Same as `@name`.', ''),
        ('bound', 'Whether the option is bound.', ''),
    ),
    'layer': _methods(
        ('data', 'The raw buffer data; its layout may differ from the bound type.', ''),
        ('stat', 'Pointer to the IMX_Stat metadata structure.', ''),
        ('bound', 'Whether an optional layer is bound; always 1 for mandatory layers.', ''),
        ('xres', 'Buffer x resolution.', ''), ('yres', 'Buffer y resolution.', ''), ('res', 'Buffer resolution.', ''),
        ('border', 'The BorderType: CONSTANT, CLAMP, MIRROR or WRAP.', ''),
        ('storage', 'The StorageType: INT8, INT16, INT32, FLOAT16, FLOAT32 (or CONSTANT_ variants).', ''),
        ('channels', 'The number of channels in the data (may differ from the #bind type).', ''),
        ('tuplesize', 'The number of channels in the data.', ''),
        ('imageToBuffer(xy)', 'Image to buffer coordinates.', ''), ('bufferToImage(xy)', 'Buffer to image coordinates.', ''),
        ('pixelToBuffer(xy)', 'Pixel to buffer coordinates.', ''), ('bufferToPixel(xy)', 'Buffer to pixel coordinates.', ''),
        ('imageToWorld(xy)', 'Image to world coordinates.', ''), ('image3ToWorld(xyz)', 'Image float3 to world coordinates.', ''),
        ('worldToImage(xyz)', 'World to image coordinates.', ''), ('worldToImage3(xyz)', 'World to image float3 coordinates.', ''),
        ('bufferIndex(ixy)', 'The buffer element at integer coordinates.', ''),
        ('bufferSample(xy)', 'Bilinear sample at buffer coordinates.', ''),
        ('imageSample(xy)', 'Bilinear sample in image space. `@name` is `@name.imageSample(@P)`.', ''),
        ('imageNearest(xy)', 'The nearest buffer element in image space.', ''),
        ('textureSample(xy)', 'Bilinear sample in texture space.', ''),
        ('textureNearest(xy)', 'The nearest buffer element in texture space.', ''),
        ('worldSample(xyz)', 'Bilinear sample in world space.', ''),
        ('worldNearest(xyz)', 'The nearest buffer element in world space.', ''),
        ('set(v)', 'Set the current buffer element.', 'writable, aligned to the output buffer'),
        ('setIndex(ixy, v)', 'Set the buffer element at integer `ixy`.', 'writable'),
        ('dCdx', 'The change of the values along the output buffer x axis (at @P, or `dCdx(xy)`).', 'non-integer layer'),
        ('dCdy', 'The change of the values along the output buffer y axis (at @P, or `dCdy(xy)`).', 'non-integer layer'),
        ('bufferSampleRect(centerxy, sizewh)', 'Sample a rectangle in buffer coordinates with the node\'s filter.', ''),
        ('bufferSampleRectClip(centerxy, sizewh)', 'Like bufferSampleRect, clipped at the image boundary.', ''),
        ('imageSampleRect(centerxy, sizewh)', 'Sample a rectangle in image coordinates with the node\'s filter.', ''),
        ('imageSampleRectClip(centerxy, sizewh)', 'Like imageSampleRect, clipped at the image boundary.', ''),
        ('textureSampleRect(centerxy, sizewh)', 'Sample a rectangle in texture coordinates with the node\'s filter.', ''),
        ('textureSampleRectClip(centerxy, sizewh)', 'Like textureSampleRect, clipped at the image boundary.', ''),
    ),
}
# Global layer-space selectors: @P.image, @P.texture, ...
SPACE_MEMBERS = {
    'image': 'Image space (the default).', 'texture': 'Texture space.', 'pixel': 'Pixel space.', 'world': 'World space (@P only).',
}


@dataclass
class Bind:
    """One parsed #bind line. Offsets are absolute in the document."""
    kind: str                     # binding word: point, parm, vdb...
    name: str                     # @-name, decorations removed
    start: int                    # directive start
    end: int
    name_start: int = 0
    name_end: int = 0
    flags: list = field(default_factory=list)       # (word, start, end)
    data_type: str = ''
    problems: list = field(default_factory=list)    # (start, end, severity, message)

    @property
    def binding_class(self):
        return BINDING_TYPES.get(self.kind, ('attribute', ''))[0]

    def flag_set(self):
        result = set()
        for word, _, _ in self.flags:
            if '=' not in word:
                result.add(word)
        return result

    def describe(self):
        words = [w for w, _, _ in self.flags] + ([self.data_type] if self.data_type else [])
        return f'#bind {self.kind} {self.name} {" ".join(words)}'.rstrip()


WORD = re.compile(r'\S+')
NAME = re.compile(r'^([&?!]*)([A-Za-z_][A-Za-z_0-9]*(?::[A-Za-z_][A-Za-z_0-9]*)?)([&?!]*)$')


def parse_bind(text, start):
    """Parse the #bind directive ``text`` starting at document offset ``start``."""
    body = text.split('//', 1)[0]
    words = [(m[0], start + m.start(), start + m.end()) for m in WORD.finditer(body)]
    bind = Bind('', '', start, start + len(text))
    # words[0] is "#bind" (or "#" followed by "bind").
    if words and words[0][0] == '#':
        words = words[1:]
    words = words[1:]
    if not words:
        bind.problems.append((start, start + len(text), 1, '#bind needs a binding type and a name: `#bind point &P float3`.'))
        return bind
    kind, ks, ke = words[0]
    bind.kind = kind
    if kind not in BINDING_TYPES:
        bind.problems.append((ks, ke, 1, f'Unknown binding type `{kind}`. Expected one of: ' + ', '.join(BINDING_TYPES) + '.'))
    if len(words) < 2:
        bind.problems.append((ks, ke, 1, f'#bind {kind} needs a binding name.'))
        return bind
    raw, ns, ne = words[1]
    match = NAME.match(raw)
    if not match:
        bind.problems.append((ns, ne, 1, f'Invalid binding name `{raw}`. Use a name, optionally decorated with & (write), ? (opt) or ! (noread).'))
        bind.name = raw.strip('&?!')
    else:
        decorations = match[1] + match[3]
        bind.name = match[2]
        bind.name_start, bind.name_end = ns + len(match[1]), ns + len(match[1]) + len(match[2])
        for char in decorations:
            bind.flags.append((DECORATIONS[char], ns, ne))
    if not bind.name_start:
        bind.name_start, bind.name_end = ns, ne
    seen = {}
    for word, ws, we in words[2:]:
        if '=' in word:
            key, _, value = word.partition('=')
            if key not in KEY_FLAGS:
                bind.problems.append((ws, we, 3, f'`{key}=` is not a documented #bind option (documented: ' + ', '.join(f'{k}=' for k in KEY_FLAGS) + ').'))
            elif not value:
                bind.problems.append((ws, we, 1, f'`{key}=` needs a value.'))
            elif key == 'border' and value.upper() not in BORDERS:
                bind.problems.append((ws, we, 2, f'Unknown border `{value}`. Expected ' + ', '.join(BORDERS) + '.'))
            elif key == 'input' and not value.isdigit():
                bind.problems.append((ws, we, 2, '`input=` expects an input number, such as `input=1`.'))
            bind.flags.append((word, ws, we))
            continue
        if word.startswith('{') or word.endswith('}') or word.endswith(','):
            # Part of a braced vector value: val={1, 2, 3}
            continue
        if TYPE_FLAG.match(word):
            if bind.data_type:
                bind.problems.append((ws, we, 2, f'More than one type flag: `{bind.data_type}` and `{word}`.'))
            bind.data_type = word
            continue
        base = word[2:] if word.startswith('no') and word[2:] in FLAGS else word
        if base not in FLAGS:
            bind.problems.append((ws, we, 3, f'`{word}` is not a documented #bind flag or type.'))
            continue
        positive = base == word
        if base in seen and seen[base] != positive:
            bind.problems.append((ws, we, 2, f'`{word}` contradicts an earlier `{"no" if positive else ""}{base}`.'))
        seen[base] = positive
        bind.flags.append((word, ws, we))
    if 'bvh' in seen and 'pointbvh' in seen and seen['bvh'] and seen['pointbvh']:
        bind.problems.append((start, start + len(text), 2, '`bvh` and `pointbvh` cannot be combined on one binding.'))
    return bind


def flag_doc(word):
    """Markdown for a #bind flag or option word, or None."""
    if word in DECORATIONS:
        return f'`{word}` decoration: implies `{DECORATIONS[word]}`.'
    if '=' in word:
        key = word.split('=', 1)[0]
        return f'`{key}=`: {KEY_FLAGS[key]}' if key in KEY_FLAGS else None
    if TYPE_FLAG.match(word):
        return f'`{word}` type flag. {TYPE_FLAG_DOC}'
    base = word[2:] if word.startswith('no') and word[2:] in FLAGS else word
    if base in FLAGS:
        return f'`{word}`: {FLAGS[base]}'
    return None
