# Touring truck frame: corner bracket

A parametric corner bracket for a touring truck frame. The frame uses 80/20 15-series 3030 extrusion, and each bracket is three plywood plates cut on a laser. The post runs through the corner and the two rails butt into it. The three plates overlap each other in a cycle. Button-head bolts go through the overlapping plate into captive hex nuts in the edge of the next plate.

## Cut files

These are ready to cut, in `out/`:

- `bottom.dxf`, `side.dxf`, `front.dxf`: one of each per corner, in inches, kerf-compensated
- `bottom.svg`, `side.svg`, `front.svg`: the same parts as SVG
- `parts.svg`: all three side by side

The cut files are built for 1/2" sheet and a 0.2 mm kerf. If your sheet measures differently, change `T` and regenerate.

## Regenerate

Needs [uv](https://docs.astral.sh/uv/). The first run downloads build123d.

```bash
uv run bracket.py
```

The script writes everything in `out/` and prints the part sizes. It stops with an error if any cut collides with another or breaks through an edge.

## Parameters to check before cutting

All of them are at the top of `bracket.py`, in inches.

- `T`: measured sheet thickness. The part sizes, bolt positions and nut slots all follow it.
- `KERF`: laser kerf. Outlines move out and holes move in by half of it. Set it to 0 if the laser software compensates.
- `CLEAR`: extra room in joint holes, nut pockets and counterbores for fit.
- `HEAD_RECESS`: counterbore depth for the joint bolt heads. 0 leaves the heads on the face, and `HEAD_H` sinks them flush. The laser doesn't cut counterbores. Flush counterbores leave less than 1/16" of wood at the outside edge on 1/2" ply, and the script warns about this.

## Hardware per corner

- 9 x 1/4-20 x 2" button head bolts, with 9 hex nuts, to join the plates
- 5/16-18 bolts with 15-series T-nuts into the extrusion slots. The four holes in the bottom plate's corner go into the post's tapped end bores.

## Other outputs

- `corner.svg` and `corner_rails.svg`: line drawings of the assembled corner, without and with the extrusions
- `corner.step` and `corner.stl`: 3D model in mm
