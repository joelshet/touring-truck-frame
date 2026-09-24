# /// script
# requires-python = ">=3.10,<3.14"
# dependencies = ["build123d"]
# ///
"""Corner bracket: three plates (bottom, side, front) meeting at a box corner.

The overlaps run in a cycle: each plate overlaps one neighbour (button head
bolts on that edge) and is overlapped by the other (captive hex-nut
slots on that edge). Along each edge the overlapping plate has a finger at
every bolt and the other plate fills the gaps. The post and rails sit inside the corner against the
plates: the post runs through, the rails butt into it.

    uv run bracket.py        # writes out/
"""

import sys
from math import atan2, cos, degrees, hypot, radians
from pathlib import Path

from build123d import (
    Align, Box, Circle, Compound, Cylinder, ExportDXF, ExportSVG,
    LineType, Plane, Polygon, Pos, Rectangle, SlotOverall, Unit, Vector,
    Kind, export_step, export_stl, extrude, mirror, offset, scale,
)

# --- parameters (inches) -----------------------------------------------------

T = 7 / 16            # measured sheet thickness; it drives everything

# Outside envelope of the assembled corner.
SIZE = {"x": 9.0, "y": 10.0, "z": 8.0}   # side to side, front to back, top to bottom

# Each plate spans two axes (u, v) with the outside corner at the origin. The
# angled edge runs from cut_u along the far u edge to cut_v along the far v edge.
PLATES = {
    #          u    v    cut_u cut_v
    "bottom": ("x", "y", 6.0, 5.0),
    "side":   ("y", "z", 5.0, 5.0),
    "front":  ("x", "z", 4.0, 5.0),
}

# plate: the plate it overlaps. Must be a cycle through all three.
OVERLAPS = {"bottom": "side", "side": "front", "front": "bottom"}

# Post and rails: 80/20 15 series 3030, 5/16-18 hardware.
POST = "z"            # the post runs through the corner; the other two rails butt into it
RAIL = 3.0            # extrusion width
RAIL_SLOTS = (0.75, 2.25)  # slot centres across a face, from its edge
END_BORES = (0.75, 2.25)   # tapped end bores (0.262 thru, 5/16-18), from each edge
RAIL_HOLE_D = 0.33    # 5/16 clearance
RAIL_MARGIN = 0.25    # wood required around each rail hole

# Plate-to-plate joints: button head bolts into captive hex nuts, spread evenly
# from the corner to the far end of each edge.
BOLT_D = 0.25         # 1/4-20
BOLT_LEN = 2.0        # under-head length
HEAD_D = 0.437        # button head diameter
HEAD_H = 0.132        # button head height
HEAD_RECESS = 0.0     # counterbore depth: 0 = head on the face, HEAD_H = flush
NUT_AF = 0.4375       # nut width across flats; corners may poke past a thin sheet
NUT_T = 0.219         # nut thickness
NUT_OFFSET = 0.75     # edge to near side of the nut pocket
JOINTS_PER_EDGE = 3
FINGER_W = 1.5        # width of the finger around each bolt; 0 = plain overlap, no fingers

HANDLE_LEN = 4.0      # overall length, parallel to the angled edge
HANDLE_W = 1.25
HANDLE_WEB = 1.75     # wood left between the angled edge and the handle

CLEAR = 0.012         # added to joint holes, pockets and counterbores for fit
KERF = 0.2 / 25.4     # laser kerf; cut files are offset by half of it. 0 if the laser software compensates

OUT = Path(__file__).parent / "out"

# --- derived -----------------------------------------------------------------

AXES = {"x": Vector(1, 0, 0), "y": Vector(0, 1, 0), "z": Vector(0, 0, 1)}
HOLE_D = BOLT_D + CLEAR
CBORE_D = HEAD_D + 2 * CLEAR
BITE = BOLT_LEN - T + HEAD_RECESS  # bolt length that ends up inside the neighbour's edge


# Along each edge the member gets a pair of holes at each end of the plate
# (slot spacing, so the pair at the corner lands on the crossing member), and
# the joint bolts run from the middle of one pair to the middle of the other.
MID = sum(RAIL_SLOTS) / 2


def rail_positions(length):
    return [T + s for s in RAIL_SLOTS] + [length - s for s in reversed(RAIL_SLOTS)]


def joint_positions(length):
    first, last = T + MID, length - MID
    step = (last - first) / (JOINTS_PER_EDGE - 1)
    return [first + k * step for k in range(JOINTS_PER_EDGE)]


JOINTS = {a: joint_positions(n) for a, n in SIZE.items()}

assert set(OVERLAPS) == set(OVERLAPS.values()) == set(PLATES)
assert all(OVERLAPS[OVERLAPS[p]] != p for p in PLATES), "overlaps must form a cycle"
assert NUT_AF <= T, "nut is wider across flats than the sheet is thick"
nut_proud = (NUT_AF / cos(radians(30)) - T) / 2
if nut_proud > 0:
    print(f"warning: hex nut corners stand {nut_proud:.3f} in proud of each face", file=sys.stderr)
assert NUT_OFFSET + NUT_T < BITE, "bolt too short to pass through the nut"
assert 0 <= HEAD_RECESS < T

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


def holes(along):
    return [Pos(b, T / 2) * Circle(HOLE_D / 2) for b in along]


def nut_slots(along):
    return [
        Pos(b, T) * Rectangle(HOLE_D, BITE + 1 / 16, align=(Align.CENTER, Align.MIN))
        + Pos(b, T + NUT_OFFSET) * Rectangle(NUT_AF + CLEAR, NUT_T + CLEAR, align=(Align.CENTER, Align.MIN))
        for b in along
    ]


def rail_points(plate):
    """Hole pairs at both ends of each member along the plate's edges, plus
    the post's end bores on the plate the post stands on."""
    u, v = axes(plate)
    across = [T + s for s in RAIL_SLOTS]
    points = {(a, c) for a in rail_positions(SIZE[u]) for c in across}
    points |= {(c, a) for a in rail_positions(SIZE[v]) for c in across}
    if normal_axis(plate) == POST:
        points |= {(T + a, T + b) for a in END_BORES for b in END_BORES}
    return sorted({(round(a, 6), round(b, 6)) for a, b in points})


def handle(a, b):
    """Slot parallel to the edge a-b, set in from it toward the corner."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = hypot(dx, dy)
    inset = HANDLE_WEB + HANDLE_W / 2
    centre = ((a[0] + b[0]) / 2 - dy / n * inset, (a[1] + b[1]) / 2 + dx / n * inset)
    return Pos(*centre) * SlotOverall(HANDLE_LEN, HANDLE_W, rotation=degrees(atan2(dy, dx)))


def fingers(axis):
    """Stretches of the edge held by the bolting plate."""
    if not FINGER_W:
        return [(T, SIZE[axis])]
    return [(b - FINGER_W / 2, b + FINGER_W / 2) for b in JOINTS[axis]]


def gaps(axis):
    """Stretches of the edge held by the nut plate."""
    ends = [T] + [e for f in fingers(axis) for e in f] + [SIZE[axis]]
    return [(a, b) for a, b in zip(ends[::2], ends[1::2]) if b > a]


def notches(stretches, widen=0.0):
    return [
        Pos(a - widen, 0) * Rectangle(b - a + 2 * widen, T, align=(Align.MIN, Align.MIN))
        for a, b in stretches
    ]


def plate_face(plate):
    u, v, cut_u, cut_v = PLATES[plate]
    U, V = SIZE[u], SIZE[v]
    bolt_edge = shared(plate, OVERLAPS[plate])
    nut_edge = u if bolt_edge == v else v

    face = Polygon((0, 0), (U, 0), (U, cut_v), (cut_u, V), (0, V), align=None)
    face -= Rectangle(T, T, align=(Align.MIN, Align.MIN))  # the outside corner is left open
    # The neighbour's edge sits in each notch; the nut plate's notches get the clearance.
    cuts = [(bolt_edge, notches(gaps(bolt_edge))), (nut_edge, notches(fingers(nut_edge), CLEAR / 2))]
    for edge, rects in cuts:
        for r in rects:
            face -= r if edge == u else mirror(r, SWAP)
    face -= handle((U, cut_v), (cut_u, V))

    joints = [
        c if edge == u else mirror(c, SWAP)
        for edge, cuts in ((bolt_edge, holes), (nut_edge, nut_slots))
        for c in cuts(JOINTS[edge])
    ]
    for c in joints:
        assert fits(c, face), f"{plate}: a joint cut runs into the outline or handle"
        face -= c

    rails = rail_points(plate)
    for p in rails:
        assert fits(Pos(*p) * Circle(RAIL_HOLE_D / 2 + RAIL_MARGIN), face), \
            f"{plate}: rail hole at {p} is too close to another cut or the edge"
    for p in rails:
        face -= Pos(*p) * Circle(RAIL_HOLE_D / 2)

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


def cut_path(face):
    """Laser path: outline moved out and holes moved in by half the kerf."""
    return offset(face, KERF / 2, kind=Kind.INTERSECTION) if KERF else face


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
        print(f"{name:7s} {size.X:.3f} x {size.Y:.3f} in, {n_rail} rail holes")
    print("joint bolts per edge: " + ", ".join(f"{a} {len(b)}" for a, b in JOINTS.items()))

    sheet, x = ExportSVG(unit=Unit.IN), 0.0
    for face, _ in built.values():
        path = cut_path(face)
        sheet.add_shape(Pos(x - path.bounding_box().min.X, 0) * path)
        x += path.bounding_box().size.X + 0.5
    sheet.write(OUT / "parts.svg")

    iso_svg(Compound(children=list(solids.values()) + shanks), OUT / "corner.svg")
    iso_svg(Compound(children=list(solids.values()) + shanks), OUT / "corner_outside.svg", eye=(-2.5, -3, -2))
    iso_svg(Compound(children=list(solids.values()) + rails), OUT / "corner_rails.svg")
    everything = Compound(children=list(solids.values()) + shanks + rails)
    export_step(scale(everything, 25.4), OUT / "corner.step")  # STEP and STL are read as mm
    export_stl(scale(everything, 25.4), OUT / "corner.stl")


if __name__ == "__main__":
    main()
