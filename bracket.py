# /// script
# requires-python = ">=3.10,<3.14"
# dependencies = ["build123d"]
# ///
"""Corner bracket: three plates (bottom, side, front) meeting at a box corner.

The overlaps run in a cycle: each plate overlaps one neighbour (button head
bolts on that edge) and is overlapped by the other (captive hex-nut
slots on that edge). Along each edge the overlapping plate has a finger at
every bolt and the other plate fills the gaps. The post and rails sit inside
the corner against the plates: the post runs through, the rails butt into it.

    uv run bracket.py        # writes out/ (laser) and out/handibot/ (CNC)
"""

import json
import sys
from fractions import Fraction
from math import atan2, cos, degrees, radians, sqrt
from pathlib import Path

from build123d import (
    Align, Box, Circle, Compound, Cylinder, ExportDXF, ExportSVG, GeomType, Kind,
    LineType, Plane, Polygon, Pos, Rectangle, Rot, SlotOverall, Unit, Vector,
    export_step, export_stl, extrude, mirror, offset, scale,
)

MM = 1 / 25.4

# --- parameters (inches) -----------------------------------------------------

T = 7 / 16            # measured sheet thickness; it drives everything

# Outside envelope of the assembled corner.
SIZE = {"x": 5.75, "y": 7.75, "z": 5.75}  # side to side, front to back, top to bottom

# Each plate spans two axes (u, v) with the outside corner at the origin. The
# angled edge runs from cut_u along the far u edge to cut_v along the far v edge.
PLATES = {
    #          u    v    cut_u cut_v
    "bottom": ("x", "y", 4.5, 6.5),
    "side":   ("y", "z", 6.5, 4.5),
    "front":  ("x", "z", 4.5, 4.5),
}

# plate: the plate it overlaps. Must be a cycle through all three.
OVERLAPS = {"bottom": "side", "side": "front", "front": "bottom"}

# Post and rails: 30 x 30 mm extrusion, one slot per face (M6 T-nuts), M8
# tapped centre bore.
POST = "z"            # the post runs through the corner; the other two rails butt into it
RAIL = 30 * MM        # extrusion width
RAIL_SLOTS = (15 * MM,)    # slot centres across a face, from its edge
END_BORES = (15 * MM,)     # tapped end bores, from each edge
RAIL_HOLE_D = 6.6 * MM     # M6 clearance, into the slots
END_HOLE_D = 8.4 * MM      # M8 clearance, into the post's end bore
RAIL_FAR = True       # also put holes at the far end of each leg, for triangle strength
RAIL_MID = True       # and one halfway along each leg; these also line the plates up in the stack
RAIL_MARGIN = 0.25    # wood required around each rail hole

# Plate-to-plate joints: button head bolts into captive hex nuts, spread evenly
# along each edge.
BOLT_D = 0.19         # #10-24
BOLT_LEN = 2.0        # under-head length
HEAD_D = 0.375        # button head diameter
HEAD_H = 0.105        # button head height
HEAD_RECESS = 0.0     # counterbore depth: 0 = head on the face, HEAD_H = flush
NUT_AF = 0.375        # nut width across flats; corners may poke past a thin sheet
NUT_T = 0.125         # nut thickness
NUT_OFFSET = 0.75     # notch floor to near side of the nut pocket
JOINTS_PER_EDGE = 2
JOINT_ENDS = (1.5, 1.5)  # first bolt this far past the neighbour's inside face, last this far from the far end
FINGER_W = 1.0        # width of the finger around each bolt; 0 = plain overlap, no fingers

# The handle runs parallel to the angled edge and is as long as it can be while
# staying clear of the extrusions, the bolt channels and the far edges.
# On a plate too small for it, the handle and the wood around it shrink by
# one factor until it fits: the same shape, smaller.
HANDLE_W = 1.25       # 0 = no handle
HANDLE_WEB = 1.75     # wood left between the angled edge and the handle
HANDLE_EDGE = 1.0     # wood left between the far edges and the handle
HANDLE_GAP = 1 / 8    # wood left between the bolt channels and the handle
HANDLE_MIN = 3.0      # shortest handle allowed
HANDLE_SMALLEST = 0.5  # narrowest the shrunk handle may get; below that there's none

CLEAR = 0.012         # added to joint holes, pockets and finger notches for fit
KERF = 0.2 * MM       # laser kerf; cut files are offset by half of it. 0 if the laser software compensates

# Handibot: one flat setup per plate cuts the outline, the holes and the nut
# pockets. Then the three plates are bolted into a stack through the middle
# rail holes, with spacers between them, and two edge-up setups cut the finger
# notches and drill the bolt channels: one for the first joint edge of all
# three plates, one for the other.
BIT = 1 / 4           # the one bit for the whole job: smallest hole, slot and radius
WORK = (6.0, 8.0)     # Handibot travel, X by Y
GAP = 3 / 8           # spacer thickness between plates in the stack
STACK = ("bottom", "side", "front")  # stacking order, first plate at the bottom

OUT = Path(__file__).parent / "out"
HANDIBOT = True       # write out/handibot/; off for laser-only variants

# Named variants override the parameters above: `uv run bracket.py small`
# writes to out/small/.
VARIANTS = {
    # Laser only, one plate per 3.5" scrap square: 1/4" ply, single 20 x 20 mm
    # extrusion (one slot per face, M5 T-nuts, M5 tapped end bore), M3 x 20
    # button heads into captive M3 nuts, the only common nut that fits inside
    # 1/4" ply.
    "small": {
        "T": 0.25,
        "SIZE": {"x": 3.25, "y": 3.25, "z": 3.25},
        "PLATES": {
            "bottom": ("x", "y", 2.25, 2.25),
            "side":   ("y", "z", 2.25, 2.25),
            "front":  ("x", "z", 2.25, 2.25),
        },
        "RAIL": 20 * MM,
        "RAIL_SLOTS": (10 * MM,),
        "END_BORES": (10 * MM,),
        "RAIL_HOLE_D": 5.5 * MM,
        "END_HOLE_D": 5.5 * MM,
        "RAIL_MID": False,
        "RAIL_MARGIN": 0.15,
        "BOLT_D": 3 * MM,
        "BOLT_LEN": 20 * MM,
        "HEAD_D": 5.7 * MM,
        "HEAD_H": 1.65 * MM,
        "NUT_AF": 5.5 * MM,
        "NUT_T": 2.4 * MM,
        "NUT_OFFSET": 0.3,
        "JOINT_ENDS": (1.0, 1.0),
        "FINGER_W": 0.6,
        "BIT": 1 * MM,    # smallest laser feature, so it never sets a hole size
        "HANDIBOT": False,
    },
}
if len(sys.argv) > 1:
    if sys.argv[1] not in VARIANTS:
        sys.exit(f"unknown variant {sys.argv[1]!r}; one of: {', '.join(VARIANTS)}")
    globals().update(VARIANTS[sys.argv[1]])
    OUT = OUT / sys.argv[1]

# --- derived -----------------------------------------------------------------

AXES = {"x": Vector(1, 0, 0), "y": Vector(0, 1, 0), "z": Vector(0, 0, 1)}
# Nothing smaller than the bit can be cut.
HOLE_D = max(BOLT_D + CLEAR, BIT)
RAIL_HOLE = max(RAIL_HOLE_D, BIT)
END_HOLE = max(END_HOLE_D, BIT)
CBORE_D = HEAD_D + 2 * CLEAR
BITE = BOLT_LEN - T + HEAD_RECESS  # bolt length that ends up inside the neighbour's edge
RUN = BIT / 2 + 1 / 32  # how far edge-up pockets run past each face
CHANNEL = T + BITE + 1 / 16  # bolt channel depth from the outside edge
# The middle rail holes sit at the same spot on every plate, halfway between
# the corner hole and the far hole of the shortest leg.
MID_AT = (T + min(SIZE.values())) / 2


def rail_positions(length):
    mid = [MID_AT] if RAIL_MID else []
    far = [length - s for s in reversed(RAIL_SLOTS)] if RAIL_FAR else []
    return [T + s for s in RAIL_SLOTS] + mid + far


def joint_positions(length):
    first, last = T + JOINT_ENDS[0], length - JOINT_ENDS[1]
    if JOINTS_PER_EDGE == 1:
        return [(first + last) / 2]
    step = (last - first) / (JOINTS_PER_EDGE - 1)
    return [first + k * step for k in range(JOINTS_PER_EDGE)]


JOINTS = {a: joint_positions(n) for a, n in SIZE.items()}

assert set(OVERLAPS) == set(OVERLAPS.values()) == set(PLATES) == set(STACK)
assert all(OVERLAPS[OVERLAPS[p]] != p for p in PLATES), "overlaps must form a cycle"
assert NUT_AF <= T, "nut is wider across flats than the sheet is thick"
nut_proud = (NUT_AF / cos(radians(30)) - T) / 2
if nut_proud > 0:
    print(f"warning: hex nut corners stand {nut_proud:.3f} in proud of each face", file=sys.stderr)
assert NUT_OFFSET + NUT_T < BITE, "bolt too short to pass through the nut"
assert 0 <= HEAD_RECESS < T
assert not HANDLE_W or BIT <= HANDLE_SMALLEST <= HANDLE_W, "HANDLE_SMALLEST must lie between BIT and HANDLE_W"
assert GAP - RUN >= 1 / 8, "spacers too thin: less than 1/8 in between the bit and the next plate"

cbore_wood = (T - CBORE_D) / 2
if HEAD_RECESS and cbore_wood < 1 / 16:
    print(f"warning: counterbores leave {cbore_wood:.3f} in of wood at the outside edge", file=sys.stderr)


def axes(plate):
    return PLATES[plate][:2]


def normal_axis(plate):
    return ({"x", "y", "z"} - set(axes(plate))).pop()


def normal(plate):
    return AXES[normal_axis(plate)]


def shared(a, b):
    return (set(axes(a)) & set(axes(b))).pop()


# --- geometry ----------------------------------------------------------------
# Plates are drawn flat in (u, v). Edge features are drawn for the edge along u
# (at v = 0) and mirrored across u = v for the edge along v (at u = 0).

SWAP = Plane((0, 0, 0), z_dir=(1, -1, 0))


def fits(cut, body):
    return (cut - body).area < 1e-9


def outline(plate):
    u, v, cut_u, cut_v = PLATES[plate]
    U, V = SIZE[u], SIZE[v]
    return Polygon((0, 0), (U, 0), (U, cut_v), (cut_u, V), (0, V), align=None)


def holes(along):
    return [Pos(b, T / 2) * Circle(HOLE_D / 2) for b in along]


# Oval nut pocket, so a router needs no square corners. The round ends are set
# so the nut's corners just clear them; turning the nut swings its corners
# out past the ends, so it can't spin.
POCKET_W = max(NUT_T + CLEAR, BIT)
POCKET_L = NUT_AF + CLEAR + POCKET_W - 2 * sqrt((POCKET_W / 2) ** 2 - (NUT_T / 2) ** 2)
assert NUT_AF + CLEAR < NUT_AF / cos(radians(30)), "nut pocket too loose to stop the nut turning"


def nut_pockets(along):
    return [Pos(b, T + NUT_OFFSET + POCKET_W / 2) * SlotOverall(POCKET_L, POCKET_W) for b in along]


def nut_slots(along):
    """Bolt channel from the notch floor, plus the nut pocket."""
    tip = CHANNEL - HOLE_D / 2
    return [
        Pos(b, T) * Rectangle(HOLE_D, tip - T, align=(Align.CENTER, Align.MIN))
        + Pos(b, tip) * Circle(HOLE_D / 2) + pocket
        for b, pocket in zip(along, nut_pockets(along))
    ]


def rail_holes(plate):
    """{(u, v): diameter} for the members along the plate's edges, plus the
    post's end bores on the plate the post stands on."""
    u, v = axes(plate)
    across = [T + s for s in RAIL_SLOTS]
    points = {(a, c): RAIL_HOLE for a in rail_positions(SIZE[u]) for c in across}
    points |= {(c, a): RAIL_HOLE for a in rail_positions(SIZE[v]) for c in across}
    if normal_axis(plate) == POST:
        points |= {(T + a, T + b): END_HOLE for a in END_BORES for b in END_BORES}
    return {(round(a, 6), round(b, 6)): d for (a, b), d in points.items()}


def handle(plate):
    """(centre, direction, overall length, width) of the handle slot, or None
    when even the smallest one doesn't fit."""
    if not HANDLE_W:
        return None
    if fit := handle_at(plate, 1.0):
        return fit
    lo, hi = HANDLE_SMALLEST / HANDLE_W, 1.0  # scales that fit and don't
    if not handle_at(plate, lo):
        return None
    for _ in range(20):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if handle_at(plate, mid) else (lo, mid)
    return handle_at(plate, lo)


def handle_at(plate, k):
    """The handle with its width, length and surrounding wood scaled by k, or
    None if it doesn't fit."""
    u, v, cut_u, cut_v = PLATES[plate]
    a, b = Vector(SIZE[u], cut_v, 0), Vector(cut_u, SIZE[v], 0)
    d = (b - a).normalized()
    w = HANDLE_W * k
    r = w / 2
    line = (a + b) / 2 + Vector(-d.Y, d.X, 0) * (HANDLE_WEB * k + r)
    # Slide the end centres along the line until the slot touches the
    # extrusions' footprint or the bolt channels (low side) or the far-edge
    # web (high side). The channels run in from the nut edge.
    nut_edge = u if shared(plate, OVERLAPS[plate]) == v else v
    channels = CHANNEL + HANDLE_GAP
    lo, hi = -float("inf"), float("inf")
    for c, dc, top, across in ((line.X, d.X, SIZE[u], v), (line.Y, d.Y, SIZE[v], u)):
        floor = max(T + RAIL, channels if across == nut_edge else 0)
        t1, t2 = (floor + r - c) / dc, (top - HANDLE_EDGE * k - r - c) / dc
        lo, hi = max(lo, min(t1, t2)), min(hi, max(t1, t2))
    # Keep it centred on the angled edge: the tighter end sets the length.
    half = min(-lo, hi)
    length = 2 * half + w
    if lo > 0 or hi < 0 or length < HANDLE_MIN * k:
        return None
    return line, d, length, w


def handle_slot(h):
    centre, d, length, w = h
    return Pos(centre) * SlotOverall(length, w, rotation=degrees(atan2(d.Y, d.X)))


def footprint(plate):
    """Where the extrusions sit on the plate's inside face (preview only)."""
    u, v = axes(plate)
    lo = (Align.MIN, Align.MIN)
    return (Pos(T, T) * Rectangle(SIZE[u] - T, RAIL, align=lo)
            + Pos(T, T) * Rectangle(RAIL, SIZE[v] - T, align=lo))


def fingers(axis):
    """Stretches of the edge held by the bolting plate."""
    if not FINGER_W:
        return [(T, SIZE[axis])]
    return [(b - FINGER_W / 2, b + FINGER_W / 2) for b in JOINTS[axis]]


def gaps(axis):
    """Stretches of the edge held by the nut plate."""
    ends = [T] + [e for f in fingers(axis) for e in f] + [SIZE[axis]]
    return [(a, b) for a, b in zip(ends[::2], ends[1::2]) if b > a]


def notch_stretches(plate, axis):
    """Stretches of this plate's edge along `axis` cut back by one sheet
    thickness, including the open outside corner."""
    if shared(plate, OVERLAPS[plate]) == axis:
        stretches = gaps(axis)
    else:
        stretches = [(a - CLEAR / 2, b + CLEAR / 2) for a, b in fingers(axis)]
    merged = []
    for a, b in sorted([(0.0, T)] + stretches):
        if merged and a <= merged[-1][1] + 1e-9:
            merged[-1] = (merged[-1][0], max(b, merged[-1][1]))
        else:
            merged.append((a, b))
    return merged


def plate_face(plate):
    u, v = axes(plate)
    bolt_edge = shared(plate, OVERLAPS[plate])
    nut_edge = u if bolt_edge == v else v

    face = outline(plate)
    for axis in (u, v):
        for a, b in notch_stretches(plate, axis):
            r = Pos(a, 0) * Rectangle(b - a, T, align=(Align.MIN, Align.MIN))
            face -= r if axis == u else mirror(r, SWAP)
    if h := handle(plate):
        face -= handle_slot(h)

    joints = [
        c if edge == u else mirror(c, SWAP)
        for edge, cuts in ((bolt_edge, holes), (nut_edge, nut_slots))
        for c in cuts(JOINTS[edge])
    ]
    for c in joints:
        assert fits(c, face), f"{plate}: a joint cut runs into the outline or handle"
        face -= c

    rails = rail_holes(plate)
    for p, d in rails.items():
        assert fits(Pos(*p) * Circle(d / 2 + RAIL_MARGIN), face), \
            f"{plate}: hole at {p} is too close to another cut or the edge"
    for p, d in rails.items():
        face -= Pos(*p) * Circle(d / 2)

    assert len(face.faces()) == 1, f"{plate}: a cutout breaks through the outline"
    return face, len(rails)


def bolt_frames():
    """(plate holding the head, frame on its outside face pointing into the joint)"""
    for plate, under in OVERLAPS.items():
        edge = shared(plate, under)
        for b in JOINTS[edge]:
            yield plate, Plane(AXES[edge] * b + normal(under) * (T / 2), z_dir=normal(plate))


def cylinder(frame, d, z0, z1):
    return frame * Pos(0, 0, z0) * Cylinder(d / 2, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def solid(plate, face):
    u, v = axes(plate)
    frame = Plane((0, 0, 0), x_dir=AXES[u], z_dir=AXES[u].cross(AXES[v]))
    part = extrude(frame * face, amount=T, dir=normal(plate))
    for p, f in bolt_frames():
        if p == plate and HEAD_RECESS:
            part -= cylinder(f, CBORE_D, 0, HEAD_RECESS)
    return part


def bolts():
    top = HEAD_RECESS
    return [
        cylinder(f, HEAD_D, top - HEAD_H, top) + cylinder(f, BOLT_D, top, top + BOLT_LEN)
        for _, f in bolt_frames()
    ]


def extrusions():
    """Preview only: the post runs through, the rails butt into it."""
    boxes = []
    for axis, e in AXES.items():
        start = T if axis == POST else T + RAIL
        size = Vector(RAIL, RAIL, RAIL) + e * (SIZE[axis] - start - RAIL)
        boxes.append(Pos(Vector(T, T, T) + e * (start - T)) * Box(*size, align=(Align.MIN,) * 3))
    return boxes


def check(solids, others):
    names = list(solids)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            vol = (solids[a] & solids[b]).volume
            assert vol < 1e-6, f"{a} and {b} overlap by {vol:.4f} in^3"
    for s in others:
        for name, p in solids.items():
            vol = (s & p).volume
            assert vol < 1e-6, f"a bolt or rail hits {name} by {vol:.4f} in^3"


def iso_svg(shape, path, eye=(3, 2, 2.5)):
    visible, hidden = shape.project_to_viewport(Vector(eye) * max(SIZE.values()))
    extent = max(*Compound(children=visible + hidden).bounding_box().size)
    svg = ExportSVG(scale=150 / extent)
    svg.add_layer("visible", line_weight=0.3)
    svg.add_layer("hidden", line_color="darkgray", line_type=LineType.ISO_DOT, line_weight=0.15)
    svg.add_shape(visible, layer="visible")
    svg.add_shape(hidden, layer="hidden")
    svg.write(path)


# --- Handibot ----------------------------------------------------------------
# Flat files are drawn as seen from the face that is up on the Handibot.

def flat_ops(plate):
    """(layer, shape) for the flat setup: the outline as a tool-centre path plus
    every through cut. The notches and bolt channels are left for the edge-up
    setups."""
    u, v = axes(plate)
    bolt_edge = shared(plate, OVERLAPS[plate])
    nut_edge = u if bolt_edge == v else v

    def along(shape, edge):
        return shape if edge == u else mirror(shape, SWAP)

    ops = [("profile", offset(outline(plate), BIT / 2, kind=Kind.INTERSECTION).faces()[0].outer_wire())]
    ops += [("through", along(c, bolt_edge)) for c in holes(JOINTS[bolt_edge])]
    ops += [("through", along(c, nut_edge)) for c in nut_pockets(JOINTS[nut_edge])]
    ops += [("through", Pos(*p) * Circle(d / 2)) for p, d in rail_holes(plate).items()]
    if h := handle(plate):
        ops.append(("through", handle_slot(h)))
    return ops


def rotated(plate):
    """Turn the plate 90 degrees when it's wider than it is tall, so the long
    side runs along the Handibot's Y."""
    u, v = axes(plate)
    return SIZE[u] > SIZE[v]


def check_flat(plate, ops):
    size = Compound(children=[shape for _, shape in ops]).bounding_box().size
    w, h = (size.Y, size.X) if rotated(plate) else (size.X, size.Y)
    assert w <= WORK[0] + 1e-9 and h <= WORK[1] + 1e-9, \
        f"{plate}: tool path spans {w:.3f} x {h:.3f} in, more than the Handibot's {WORK[0]} x {WORK[1]}"


def gang(edge):
    """Notch pockets and channel drill points for one edge-up setup: edge 0 is
    each plate's u edge, edge 1 its v edge. x runs across the stack from the
    left plate's left face, y along the edges from the corner end. Returns
    [(plate, axis, x0, pockets, drills)] left to right, where x0 is that plate's
    left face."""
    # Looking down on the u edges, plates higher in the stack sit further left;
    # on the v edges they sit further right.
    order = list(STACK) if edge == 1 else list(reversed(STACK))
    boards = []
    for i, plate in enumerate(order):
        axis = axes(plate)[edge]
        x0 = i * (T + GAP)
        pockets = []
        for a, b in notch_stretches(plate, axis):
            a = -RUN if a <= 0 else a
            b = SIZE[axis] + RUN if b >= SIZE[axis] - 1e-9 else b
            assert b - a >= BIT, f"{plate} {axis} edge: a {b - a:.3f} in notch is narrower than the bit"
            pockets.append(Pos(x0 - RUN, a) * Rectangle(T + 2 * RUN, b - a, align=(Align.MIN, Align.MIN)))
        nut_edge = shared(plate, OVERLAPS[plate]) != axis
        drills = [Pos(x0 + T / 2, b) * Circle(HOLE_D / 2) for b in JOINTS[axis]] if nut_edge else []
        boards.append((plate, axis, x0, pockets, drills))
    # The bit's centre stays a radius inside the pocket outlines.
    width = len(order) * T + (len(order) - 1) * GAP + 2 * RUN - BIT
    length = max(SIZE[axis] for _, axis, _, _, _ in boards) + 2 * RUN - BIT
    assert width <= WORK[0] and length <= WORK[1], \
        f"edge-up setup {edge + 1} needs {width:.3f} x {length:.3f} in of travel"
    return boards


def steps():
    lines = [
        f"Sheet {T:.4f} in, bit {BIT:.4f} in, spacers {GAP:.4f} in.",
        "",
        "1. Flat, one per plate. The profile layer is the tool-centre path: cut on the",
        "   line, full depth, with tabs. The through layer (holes, nut pockets, handle):",
        "   cut inside, full depth.",
    ]
    for plate, (u, v, _, _) in PLATES.items():
        turn = f", turned 90 degrees so the {u} edge runs along +Y" if rotated(plate) else ""
        lines.append(f"   {plate}_flat.dxf: {SIZE[u]:.3f} x {SIZE[v]:.3f} in{turn}")
    lines += [
        "",
        f"2. Mark the up face of each plate. Stack them face-up as cut, {', '.join(STACK)}",
        "   from the bottom, with a spacer between each, bolted through the two middle",
        "   rail holes (the ones halfway along each leg).",
        "",
    ]
    for edge in (0, 1):
        boards = gang(edge)
        order = ", ".join(f"{p} ({a} edge)" for p, a, _, _, _ in boards)
        lines += [
            f"{3 + edge}. Edge-up, edges_{edge + 1}.dxf. Left to right: {order}.",
            f"   y = 0 at the corner end, x = 0 at the left plate's left face.",
            f"   pocket layer: {T:.4f} deep. Then drill layer: {CHANNEL:.4f} deep from the",
            f"   top of the edge. If the bit is shorter, start them and finish with a drill.",
        ]
    return "\n".join(lines) + "\n"


def write_handibot(out):
    out.mkdir(exist_ok=True)
    preview = ExportSVG(unit=Unit.IN)
    preview.add_layer("work", line_color="lightgray", line_weight=0.5)
    preview.add_layer("plate", line_color="gray", line_weight=0.5)
    preview.add_layer("profile", line_color="green", line_weight=1.0)
    preview.add_layer("through", line_color="blue", line_weight=1.0)
    preview.add_layer("pocket", line_color="red", line_weight=1.0)
    preview.add_layer("drill", line_color="purple", line_weight=1.0)
    box = Rectangle(*WORK, align=(Align.MIN, Align.MIN))

    for i, plate in enumerate(PLATES):
        ops = flat_ops(plate)
        check_flat(plate, ops)
        if rotated(plate):
            ops = [(layer, Rot(0, 0, 90) * shape) for layer, shape in ops]
        dxf = ExportDXF(unit=Unit.IN)
        for layer in ("profile", "through"):
            dxf.add_layer(layer)
        for layer, shape in ops:
            dxf.add_shape(shape, layer=layer)
        dxf.write(out / f"{plate}_flat.dxf")
        # Preview: each setup inside the Handibot's travel.
        corner = Compound(children=[s for layer, s in ops if layer == "profile"]).bounding_box().min
        at = Pos(i * (WORK[0] + 1) - corner.X, -corner.Y)
        preview.add_shape(Pos(i * (WORK[0] + 1), 0) * box, layer="work")
        for layer, shape in ops:
            preview.add_shape(at * shape, layer=layer)

    for edge in (0, 1):
        dxf = ExportDXF(unit=Unit.IN)
        dxf.add_layer("pocket")
        dxf.add_layer("drill")
        at = Pos(edge * (WORK[0] + 1), -WORK[1] - 1)
        preview.add_shape(at * box, layer="work")
        inside = at * Pos(RUN, RUN)
        for plate, axis, x0, pockets, drills in gang(edge):
            preview.add_shape(inside * Pos(x0, 0) * Rectangle(T, SIZE[axis], align=(Align.MIN, Align.MIN)),
                              layer="plate")
            for layer, shapes in (("pocket", pockets), ("drill", drills)):
                for shape in shapes:
                    dxf.add_shape(shape, layer=layer)
                    preview.add_shape(inside * shape, layer=layer)
        dxf.write(out / f"edges_{edge + 1}.dxf")
    preview.write(out / "preview.svg")
    (out / "steps.txt").write_text(steps())


# --- laser -------------------------------------------------------------------

def cut_path(face):
    """Laser path: outline moved out and holes moved in by half the kerf."""
    return offset(face, KERF / 2, kind=Kind.INTERSECTION) if KERF else face


def inches(x):
    """3.25 as 3-1/4", to the nearest 1/64."""
    f = Fraction(x).limit_denominator(64)
    whole, rest = divmod(f, 1)
    return (f"{whole}-{rest}" if whole and rest else f"{rest or whole}") + '"'


def write_model(parts, path):
    """The assembled corner for a browser that turns it: each part's triangle
    mesh, which hides what's behind it, and its edges, which get drawn. In mm.
    `parts` is {name: (shape, role)}; role "context" draws in a lighter ink.
    Seam edges, the line OCCT runs down the inside of every hole, are left out.
    """
    from OCP.BRep import BRep_Tool
    tol = 0.002
    out = []
    for name, (shape, role) in parts.items():
        shape = scale(shape, 25.4)
        points, triangles = shape.tessellate(tol * 25.4, 0.2)
        seams = {e for f in shape.faces() for e in f.edges()
                 if BRep_Tool.IsClosed_s(e.wrapped, f.wrapped)}
        edges = []
        for e in shape.edges():
            if e in seams:
                continue
            n = 1 if e.geom_type == GeomType.LINE else min(96, max(12, int(e.length / 1.0) + 1))
            edges.append([round(c, 2) for p in e.positions([i / n for i in range(n + 1)])
                          for c in (p.X, p.Y, p.Z)])
        out.append({"name": name, "role": role,
                    "points": [round(c, 2) for p in points for c in (p.X, p.Y, p.Z)],
                    "triangles": [i for t in triangles for i in t],
                    "edges": edges})
    path.write_text(json.dumps({"units": "mm", "parts": out}, separators=(",", ":")))


def main():
    OUT.mkdir(exist_ok=True)
    built = {p: plate_face(p) for p in PLATES}
    solids = {p: solid(p, face) for p, (face, _) in built.items()}
    shanks, rails = bolts(), extrusions()
    check(solids, shanks + rails)

    for name, (face, n_rail) in built.items():
        path = cut_path(face)
        dxf = ExportDXF(unit=Unit.IN)
        dxf.add_shape(path)
        dxf.write(OUT / f"{name}.dxf")
        svg = ExportSVG(unit=Unit.IN)
        svg.add_shape(path)
        svg.write(OUT / f"{name}.svg")
        size = face.bounding_box().size
        h = handle(name)
        grip = f", {h[3]:.3f} x {h[2]:.2f} in handle" if h else ", no handle"
        print(f"{name:7s} {size.X:.3f} x {size.Y:.3f} in, {n_rail} rail holes{grip}")
    print("joint bolts per edge: " + ", ".join(f"{a} {len(b)}" for a, b in JOINTS.items()))

    sheet, preview, x = ExportSVG(unit=Unit.IN), ExportSVG(unit=Unit.IN), 0.0
    preview.add_layer("extrusions", fill_color="lightsteelblue", line_color="steelblue")
    preview.add_layer("parts")
    for name, (face, _) in built.items():
        path = cut_path(face)
        at = Pos(x - path.bounding_box().min.X, 0)
        sheet.add_shape(at * path)
        preview.add_shape(at * footprint(name), layer="extrusions")
        preview.add_shape(at * path, layer="parts")
        x += path.bounding_box().size.X + 0.5
    sheet.write(OUT / "parts.svg")
    preview.write(OUT / "preview.svg")  # parts with the extrusion footprints shaded; not for cutting

    if HANDIBOT:
        write_handibot(OUT / "handibot")
    iso_svg(Compound(children=list(solids.values()) + shanks), OUT / "corner.svg")
    iso_svg(Compound(children=list(solids.values()) + shanks), OUT / "corner_outside.svg", eye=(-2.5, -3, -2))
    iso_svg(Compound(children=list(solids.values()) + rails), OUT / "corner_rails.svg")
    everything = Compound(children=list(solids.values()) + shanks + rails)
    export_step(scale(everything, 25.4), OUT / "corner.step")  # STEP and STL are read as mm
    export_stl(scale(everything, 25.4), OUT / "corner.stl")

    write_model({**{p: (s, "part") for p, s in solids.items()},
                 "bolts": (Compound(children=shanks), "part"),
                 "extrusion": (Compound(children=rails), "context")}, OUT / "model.json")
    # The drawing sheet this is published on at joelshetler.com: the title block as label and
    # value in reading order, and the files in groups.
    n_bolts = sum(len(b) for b in JOINTS.values())
    title_block = [
        ("Outside", f'{inches(SIZE["x"])} x {inches(SIZE["y"])} x {inches(SIZE["z"])} '
                    f'({SIZE["x"] * 25.4:.2f} x {SIZE["y"] * 25.4:.2f} x {SIZE["z"] * 25.4:.2f} mm)'),
        ("Stock", f"{inches(T)} ({T * 25.4:.2f} mm) plywood"),
        ("Plates", f"{len(PLATES)}, overlapping in a cycle"),
        ("Extrusion", f"{RAIL * 25.4:g} x {RAIL * 25.4:g} mm"),
        ("Joint bolts", f"{n_bolts} button heads, {BOLT_D * 25.4:.0f} x {BOLT_LEN * 25.4:.0f} mm, "
                        f"into captive nuts"),
        ("Kerf", f"{KERF * 25.4:g} mm, already in the files"),
    ]
    files = [
        {"label": "Cut one plate, in inches", "files": [f"{p}.{x}" for p in PLATES for x in ("svg", "dxf")]},
        {"label": "All three side by side, in inches", "files": ["parts.svg"]},
        {"label": "Model, in mm", "files": ["corner.step"]},
    ]
    (OUT / "sheet.json").write_text(json.dumps({"title_block": title_block, "files": files}, indent=2))


if __name__ == "__main__":
    main()
