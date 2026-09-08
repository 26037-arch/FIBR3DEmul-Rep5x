# Rep5x scene hierarchy

The builder creates aliases exactly once and fails if `/Rep5x_Ender3V3SE` already exists. All motion is kinematic/passive-joint position control; proxy shapes are static and collision response is not part of phase 1.

```text
Rep5x_Ender3V3SE
├─ Rep5x_FixedFrame (placeholder primitives)
├─ Rep5x_Y_bed_joint [prismatic, scene Y = -machine Y]
│  └─ Rep5x_MovingBed [placeholder]
│     └─ DrawBoard
└─ Rep5x_Z_joint [prismatic]
   └─ Rep5x_ZGantry [placeholder]
      └─ Rep5x_X_joint [prismatic]
         └─ Rep5x_XCarriage [placeholder]
            └─ Rep5x_C_joint [revolute about Z, cyclic]
               └─ Rep5x_CLink [proxy; B pivot offset -LC in C frame]
                  └─ Rep5x_B_joint [revolute about rotated Y, -135°..+135°]
                     └─ Rep5x_BLink [proxy]
                        ├─ Rep5x_Nozzle [placeholder]
                        └─ Rep5x_NozzleTip [dummy marker]
```

Movement ownership follows the tree: X carries carriage+C+B+nozzle; Z carries gantry and the entire head; C carries its link+B+nozzle; B carries B link+nozzle. Y moves only the bed. `Rep5xSceneContract.descendants()` verifies those sets headlessly.

## Atomic application

Lua `applyPose` computes the full five-axis state, validates finite values and soft limits, then writes all five joints in one `sysCall_actuation` callback. Jog is rejected while playback is running unless it is paused. The C++ bridge similarly adapts all samples before enqueuing any, preventing a partially converted queue.

## Visual vs kinematic assets

Joint frames and offsets are independent of visual mesh origins. The included Rep5x STL conversions preserve 3MF transforms, but the source files do not provide validated printer-assembly mating transforms. The scene therefore uses honest proxy geometry and does not import those STLs at arbitrary identities. `assets/manifest.json` records each likely link and marks `assembly_transform: unknown`; the high-detail files are ready for a later measured visual-alignment pass. Collision proxies are the separate primitives created by the Lua builder.

## Pose reporting

The UI shows commanded TCP X/Y/Z/C/B and the nozzle-tip marker's physical world XYZ. Because Ender Y is a moving bed, world nozzle Y is not itself the commanded tool-relative Y; `nozzle_world_y - bed_world_y` is the coordinate that equals commanded Y.
