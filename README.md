# Touring truck frame: corner bracket

A parametric corner bracket for a touring truck frame built from aluminium T-slot extrusion. Each bracket is three plywood plates. The post runs through the corner and the two rails butt into it. The three plates overlap each other in a cycle. Along each edge, the overlapping plate has a finger at every bolt and the other plate fills the gaps. Button-head bolts go through those fingers into captive hex nuts in oval pockets in the next plate.

The parameters are currently set for a scaled-down test sized for 30 x 30 mm extrusion. Each plate fits the Handibot's 6 x 8" travel in one flat setup: the envelope is 5.75 x 7.75 x 5.75", with 7/16" sheet, #10-24 joint hardware and a 1/4" bit. The full-size bracket (9 x 10 x 8", 3030 extrusion) is the same script with different parameters.

## Handibot files

These are in `out/handibot/`. `steps.txt` has the exact order and orientation for each setup:

1. **One flat setup per plate** (`<plate>_flat.dxf`, outside corner at the origin).
   - The `profile` layer is the tool-centre path around the whole outline. Cut it on the line, with tabs.
   - The `through` layer holds every hole, oval nut pocket and handle. Cut it inside.
2. **Stack the three plates** with spacers between them. Bolt them together through the rail holes halfway along each leg, which sit at the same spot on every plate. Mark each plate's up face first. The holes are symmetric, so a plate turned face-down would still fit.
3. **Two edge-up setups** (`edges_1.dxf`, `edges_2.dxf`). Each one covers one joint edge of all three plates at once:
   - The `pocket` layer cuts the finger notches.
   - The `drill` layer drills the bolt channels into the end of the plate, down to the nut pocket.

`preview.svg` shows each setup inside the Handibot's travel. It's for checking only. Nothing is smaller than the bit: `BIT` sets the smallest hole, slot and inside radius, and the only square inside corners are the finger notches, cut edge-up.

## Laser files

These are in `out/`:

- `bottom.dxf`, `side.dxf`, `front.dxf`: one of each per corner, in inches, kerf-compensated
- `bottom.svg`, `side.svg`, `front.svg`: the same parts as SVG
- `parts.svg`: all three side by side
- `preview.svg`: the parts with the extrusion footprints shaded, for checking only. Don't cut this one.

## Small laser variant

`uv run bracket.py small` writes `out/small/`: the same bracket with each plate 3.25" square, so one plate comes out of a 3.5" scrap square. It's 1/4" ply around single 20 x 20 mm extrusion (2020, one slot per face), with no middle rail holes and no Handibot files. The handles come out 0.73 x 1.76". Its parameters are in `VARIANTS` in `bracket.py`. Per corner it takes 6 M3 x 20 button heads with M3 nuts, 8 M5 bolts with T-nuts into the slots, and 1 M5 bolt into the post's end bore.

## Regenerate

Needs [uv](https://docs.astral.sh/uv/). The first run downloads build123d.

```bash
uv run bracket.py
```

The script writes everything in `out/` and prints the part sizes. It stops with an error in any of these cases:

- a cut collides with another cut or breaks through an edge
- a feature is narrower than the bit
- a setup doesn't fit the Handibot's travel

## Parameters to check before cutting

All of them are at the top of `bracket.py`, in inches.

- `T`: measured sheet thickness. The part sizes, bolt positions, nut slots and notch depths all follow it.
- `SIZE` and `PLATES`: the outside envelope and where each angled corner starts.
- `BIT`: the one router bit for the job.
- `WORK`: the Handibot's travel. Each plate can be up to `WORK` minus one bit diameter.
- `GAP` and `STACK`: spacer thickness and stacking order for the edge-up setups. The build stops if the spacers leave less than 1/8" between the bit and the next plate.
- `RAIL`, `RAIL_SLOTS`, `END_BORES`, `RAIL_HOLE_D` and `END_HOLE_D`: the extrusion's size, slot positions and bolt clearances.
- `KERF`: laser kerf. Outlines move out and holes move in by half of it. Set it to 0 if the laser software compensates. It doesn't apply to the Handibot files.
- `CLEAR`: extra room in joint holes, nut pockets, counterbores and finger notches for fit.
- `FINGER_W` and `JOINTS_PER_EDGE`: finger width and bolt count per edge. `FINGER_W = 0` goes back to a plain overlap.
- `RAIL_FAR` and `RAIL_MID`: add rail holes at the far end of each leg, for triangle strength, and halfway along each leg. The middle holes also line the plates up in the stack.
- `HANDLE_W`, `HANDLE_WEB`, `HANDLE_EDGE` and `HANDLE_MIN`: handle width, the wood left around it, and its shortest length. On a plate too small for that handle, all four shrink by one factor until it fits, down to `HANDLE_SMALLEST` wide. `HANDLE_W = 0` means no handle.
- `HEAD_RECESS`: counterbore depth for the joint bolt heads. 0 leaves the heads on the face. Neither the laser nor the Handibot files cut counterbores.

## Hardware per corner

- 6 x #10-24 x 2" button head bolts with 6 hex nuts (3/8" across flats), to join the plates
- 14 M6 bolts with T-nuts into the extrusion slots: 4 through the bottom plate, 5 each through the side and front
- 1 M8 bolt through the bottom plate's corner hole into the post's tapped end bore
- 2 x M6 or 1/4" bolts, with 3/8" spacers, to hold the stack for the edge-up setups

## Other outputs

- `corner.svg`, `corner_outside.svg` and `corner_rails.svg`: line drawings of the assembled corner from inside, from outside, and with the extrusions
- `corner.step` and `corner.stl`: 3D model in mm
