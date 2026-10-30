# Rep5x scene hierarchy

The builder creates aliases exactly once and fails if `/Rep5x_Ender3V3SE` already exists. All motion is kinematic/passive-joint position control; proxy shapes are static and collision response is not part of this phase.

Required (contract) objects are shown in bold; everything else is cosmetic proxy geometry that hangs off the same links and never influences the kinematic chain.

```text
Rep5x_Ender3V3SE
├─ **Rep5x_FixedFrame** (dummy) — 4040 base/uprights, 4020 top bar, feet, Y rail + Y stepper,
│     Z stepper + lead screw, display, PSU, external Bowden extruder, spool holder + spool,
│     Rep5x_BowdenOutlet (dummy: tube anchor)
├─ **Rep5x_Y_bed_joint** [prismatic, scene Y = -machine Y]
│  └─ **Rep5x_MovingBed** (235 x 235 x 3 mm heated-bed plate, world aligned)
│     ├─ Rep5x_PrintSheet (1 mm build plate; its top face is TCP Z = 0)
│     ├─ **DrawBoard** (dummy at the G-code origin = front-left corner of the 220 x 220 print area;
│     │     parent of the deposited-filament drawing object)
│     └─ bed carriage, springs, levelling knobs, V-wheels, belt clamp
└─ **Rep5x_Z_joint** [prismatic]
   └─ **Rep5x_ZGantry** (2020 X extrusion, 115 mm above the zero-pose tip)
      ├─ Z brackets, lead nut, V-wheels, X stepper, X belt, idler
      └─ **Rep5x_X_joint** [prismatic; frame origin = zero-pose nozzle tip]
         └─ **Rep5x_XCarriage** (steel carriage plate, world aligned)
            ├─ X V-wheels, Rep5x carriage mount, C bearing housing, C stepper + pulley + belt, loom
            └─ **Rep5x_C_joint** [revolute about Z, cyclic; at (0, LC, LB) from the tip]
               └─ **Rep5x_CLink** (C driven pulley on the C axis, 40 mm above the B pivot P)
                  ├─ flanges, slip-ring holder + slip ring, Rep5x_BowdenInlet (dummy), upper PTFE tube
                  ├─ B arm plate + cheeks, B axle, B stepper + pulley + belt
                  └─ **Rep5x_B_joint** [revolute about rotated Y, -135..+135 deg; at P]
                     └─ **Rep5x_BLink** (hotend holder block, world aligned at B = 0)
                        ├─ heatsink + fins, hotend fan, heat break, silicone-socked heater block, wires
                        ├─ B driven pulley + flanges
                        ├─ **Rep5x_Nozzle** (brass cone, apex at the tip)
                        ├─ Rep5x_MeltDrop (1.2 mm sphere; visible only while extruding)
                        └─ **Rep5x_NozzleTip** (dummy marker at (0, 0, -LB) from P)
```

Movement ownership follows the tree: X carries carriage+C+B+hotend; Z carries gantry and the entire head; C carries its link+B+hotend; B carries the hotend. Y moves only the bed, and because the filament drawing object is parented to `DrawBoard`, printed material moves with the bed. `Rep5xSceneContract.descendants()` verifies the required sets headlessly.

## Joint frames

The joint frames are the phase-1 contract and did not move when the visual proxy was redesigned:

- Y joint at the root with `Rx(-90°)` (joint Z = world +Y); the bed is a world-aligned child.
- Z joint at the root (joint Z = world Z). The X joint is a child of the gantry shape, offset so that its origin stays at the zero-pose tip, with `Ry(90°)` (joint Z = world X).
- C joint at `[0, +LC, +LB]` in the carriage frame; the C link's children are placed relative to the B pivot `P = [0, -LC, 0]` in the C frame.
- B joint at P with `Rx(-90°)`; the B link is re-aligned with `Rx(+90°)` so its children use world-aligned offsets. The tip is at `[0, 0, -LB]` from P.

The alias shapes are real visible parts (carriage plate, C pulley, hotend holder). Their shape frames are their geometric centres, so child offsets are expressed through the `rel()` helpers in the builder rather than by hand.

## Atomic application

Lua `applyPose` computes the full five-axis state, validates finite values and soft limits, then writes all five joints in one `sysCall_actuation` callback. Playback interpolates every G-code segment linearly in tool space (X/Y/Z/C/B together) at the commanded feed rate (`F`, default 1500 mm/min; G0 travels at 6000 mm/min; pure rotations at 90 deg/s) times the UI speed factor. Jog is rejected while playback is running unless it is paused. The C++ bridge similarly adapts all samples before enqueuing any, preventing a partially converted queue.

## Filament deposition

- A segment deposits when it is a `G1` whose extruder delta is positive (`M82` absolute or `M83` relative E; the legacy FIBR3D `A` word is accepted as an alias; `G92 E` resets the counter). `G0` and retractions never deposit.
- Bead samples are spheres added to one drawing object (`sim.drawing_spherepoints`, per-item sizes when the installed CoppeliaSim supports `sim.drawing_itemsizes`) parented to `DrawBoard`. Items are passed in world coordinates and CoppeliaSim stores them relative to the parent, so the print follows the moving bed.
- Sample coordinates are the interpolated tool XYZ (metres), which by construction equal the nozzle tip relative to `DrawBoard`. The add-on cross-checks the measured tip against the commanded pose every tick and logs a warning once if they differ by more than 0.05 mm.
- Bead width is derived from the volumetric flow (`E` delta × filament cross-section / path length, scaled 1.4× and clamped to 0.15..1.2 mm); files without `E` use 0.45 mm.
- A 1.2 mm "melt drop" sphere at the tip is shown only while a depositing segment runs, and a white line strip draws the Bowden tube from the frame-mounted extruder to the head.
- **Clear print** empties the drawing object; the buffer holds 400 000 samples (about 100 m of path at 0.25 mm spacing) before a status warning stops further sampling.

## Visual vs kinematic assets

The included Rep5x STL conversions preserve 3MF transforms, but the source files do not provide validated printer-assembly mating transforms. The scene therefore uses dimensioned proxy geometry (Ender-3 V3 SE footprint, 2020/4040 extrusions, Rep5x printed parts in orange, pulleys and belts at their kinematic positions) and does not import those STLs at arbitrary identities. `assets/manifest.json` records each likely link and marks `assembly_transform: unknown`; the high-detail files are ready for a later measured visual-alignment pass. Collision proxies are the separate primitives created by the Lua builder.

## Pose reporting

The UI shows commanded TCP X/Y/Z/C/B, the nozzle-tip marker's physical world XYZ, and the extrusion counters (filament consumed, bead samples, extruding flag). Because Ender Y is a moving bed, world nozzle Y is not itself the commanded tool-relative Y; `nozzle_world_y - bed_world_y` is the coordinate that equals commanded Y.
