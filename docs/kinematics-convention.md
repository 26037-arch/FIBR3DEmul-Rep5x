# Rep5x kinematics and coordinate convention

## Canonical contract

G-code/TCP state is `(X',Y',Z',C,B)` in mm and degrees. C is yaw about +Z; B is tilt about the Y axis after C. FIBR3D's historical five-element arrays are physically ordered `[X,Y,Z,B,C]`; `Rep5xMotionAdapter` performs the explicit reorder.

For LC and LB in millimetres, the canonical tool-to-joint transform is:

```text
X = X' - sin(C) LC + cos(C) sin(B) LB
Y = Y' + (cos(C)-1) LC + sin(C) sin(B) LB
Z = Z' + (cos(B)-1) LB
```

This is copied semantically (not by linking code) from Rep5x `tools/shared/inverse-kinematics.js` at commit `9c437d5...` and matches `native_to_joint()` in Rep5x-Marlin `Marlin/src/module/penta_axis_head_head.cpp` at `d25ee4e...`.

The algebraic inverse used for round trips is:

```text
X' = X + sin(C) LC - cos(C) sin(B) LB
Y' = Y - (cos(C)-1) LC - sin(C) sin(B) LB
Z' = Z - (cos(B)-1) LB
```

The audited firmware's `joint_to_native()` currently includes the LB terms but not LC; therefore the phase-1 round-trip test uses the complete algebraic inverse from Rep5x shared JS, not that incomplete firmware forward path.

## Conflicting viewer reverser

`tools/gcode-viewer/js/inverse-kinematics-reverser.js` at the same Rep5x commit documents a different forward correction:

```text
X = X' + sin(C) LC + cos(C) sin(B) LB
Y = Y' + (cos(C)-1) LC - sin(C) sin(B) LB
```

Its LC-X and B-Y signs conflict with both Rep5x shared IK and firmware `native_to_joint()`. It is not averaged, mixed, or silently selected. The simulator uses the shared/firmware-native convention only.

## Unit boundary and signs

- Parser/JSON coordinates: XYZ mm; B/C degrees despite an inaccurate comment in `Translator.cs`.
- FIBR3D `Trajectory`: `[X,Y,Z,B,C]` as m/rad.
- Rep5x scene controller: `[X,Y,Z,C,B]` as m/rad.
- Conversion happens at the existing trajectory boundary. The C++ adapter receives SI and never divides XYZ by 1000 or multiplies angles by π/180 a second time.
- `scene_sign(Y)=-1` because the Ender bed translates opposite the commanded tool-relative Y. This is a joint sign, not a mesh flip.
- C and B scene signs default to +1 for the tested mathematical convention. They remain profile fields so a physical motor convention is never hidden in geometry.

## Joint frames that realize the equations

At zero pose, take the TCP as the carriage machine coordinate. Place the C pivot at `[0,+LC,+LB]`. In the C frame, place the B pivot at `[0,-LC,0]`, and the nozzle vector in the B frame at `[0,0,-LB]`. The tip displacement caused by rotations is:

```text
[0,+LC,+LB] + Rz(C) ([0,-LC,0] + Ry(B)[0,0,-LB])
```

Adding that displacement to the IK-corrected machine X/Z and subtracting the moving-bed Y yields the requested TCP exactly. `tests/test_scene_contract.py` calculates this independently of `Rep5xKinematics` and checks the result within 0.1 mm.

## Numeric acceptance points

With LC=10 mm and LB=54.67 mm, tests assert:

| B | C | ΔX | ΔY | ΔZ (mm) |
|---:|---:|---:|---:|---:|
| 0° | 0° | 0 | 0 | 0 |
| -90° | 0° | -54.67 | 0 | -54.67 |
| +90° | 0° | +54.67 | 0 | -54.67 |
| 0° | 90° | -10 | -10 | 0 |
| 0° | 180° | 0 | -20 | 0 |

One hundred deterministic random poses inside the configured limits must round-trip with maximum XYZ error <= `1e-6` mm.
