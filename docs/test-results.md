# Verification results

Run date: 2026-09-08 (Asia/Seoul) on Windows x64, Python 3.12.10, CMake/GNU 13.2 from Strawberry Perl. No CoppeliaSim installation was present.

## Passing headless suite

Command:

```powershell
python -m unittest discover -s tests -v
```

Result: **16 tests passed** in 0.017 seconds.

Coverage includes:

- exact numeric kinematics cases and 100 seeded forward/inverse round trips under `1e-6 mm`;
- G-code absolute/relative and metric/inch modes, modal axes, XYZEF compatibility, `CONICAL_META` warning, and golden pose sequences;
- explicit 100 mm → 0.1 m and 90° → π/2 rad boundaries plus FIBR3D `[X,Y,Z,B,C]` reordering without double conversion;
- requested-pose and IK-result machine soft limits, NaN/Infinity refusal, and invalid trajectory schema;
- unique/complete hierarchy, expected parent-child movement sets, independent nozzle/TCP reconstruction, atomic writes, and a 101-sample continuous-C sweep without NaN, empty history, or a ±360° normalization jump;
- 3MF unit/transform composition, loud rejection of malformed transforms, and SHA-256 verification of all ten converted STL files against the audit report.

## Fixture replay

```powershell
python -m rep5x_runtime.cli tests\fixtures\rep5x_axis_smoke.gcode --json
```

Result: 9 five-axis moves. Final tool pose `(100,90,130,135,-35)` mm/deg; scene joints `(0.122173047,-0.067826953,0.120113042,2.356194490,-0.610865238)` m/rad. Independent TCP reconstruction relative to the bed returned exactly `(100,90,130)` mm to displayed precision.

```powershell
python -m rep5x_runtime.cli tests\fixtures\slicer_xyz_smoke.gcode --json
```

Result: 6 moves in XYZ-only compatibility mode. B/C remained zero; M82 and ignored `CONICAL_META` produced explicit warnings. Final scene joints were `(0.02,-0.02,0.0004,0,0)` m/rad.

`python tools\generate_evidence.py` completed and produced JSON, CSV, SVG, and optional PNG records under `evidence/`. Maximum independent TCP reconstruction error across both fixtures is below `1e-12 mm` (floating-point noise).

## Build/runtime checks not completed on this host

Plugin configure reached dependency discovery:

```powershell
cmake -S v-rep_plugin -B v-rep_plugin\build-rep5x -G "MinGW Makefiles"
```

GNU C/C++ 13.2 was detected, then configuration correctly stopped at the declared prerequisite: `Could NOT find Boost (missing: Boost_INCLUDE_DIR system)`. A direct adapter compilation stopped on the same absent `boost/property_tree/json_parser.hpp` header. No plugin DLL was produced.

The original C# solution could not build. `dotnet msbuild` reports that no .NET SDK is installed; the installed Visual Studio MSBuild also reports `MSB3644` because the .NET Framework 4.5.2 targeting pack is absent.

CoppeliaSim scene/API execution, screenshot capture, plugin load, and end-to-end TCP playback remain **not run**, because CoppeliaSim is not installed. The supplied installer, generated scene add-on, and explicit validation paths are ready for that final environment-specific check. No result in this report treats a checked-in legacy binary as a successful build or runtime test.

## 2026-10-30 update: realistic proxy model and nozzle deposition

Run on Linux x64, Python 3.13, lupa 2.8 (Lua 5.4-compatible runtime), numpy, matplotlib. CoppeliaSim is still not installed on this host.

```bash
python -m unittest discover -s tests -v
python tools/generate_evidence.py
python tools/lua_mock_harness.py --json evidence/lua-mock-self-test.json
python tools/lua_mock_harness.py --gcode tests/fixtures/rep5x_axis_smoke.gcode
python tools/lua_mock_harness.py --gcode tests/fixtures/slicer_xyz_smoke.gcode
python tools/lua_mock_harness.py --render evidence/scene-preview-mock.png
python tools/lua_mock_harness.py --render evidence/scene-preview-mock_c135_b-35.png --c 135 --b -35
```

Results: **18 tests passed** (two new extrusion-mode tests). The Lua add-on executed end to end inside the mock `sim` API:

| run | moves | ticks | tip relative to bed vs commanded | bead samples | bead width |
|---|---:|---:|---|---:|---|
| built-in test print | 18 | 706 | exact (80, 80, 120) | 1768 on the two Z = 0.2 mm squares | 0.446–0.447 mm |
| `rep5x_axis_smoke.gcode` | 9 | 278 | exact (100, 90, 130) at C = 135°, B = -35° | 0 (no E words) | – |
| `slicer_xyz_smoke.gcode` | 6 | 88 | exact (20, 20, 0.4) | 324 on the 20..40 mm square | 0.49 mm |

Every bead sample landed on the commanded path, the add-on's own tip-vs-command cross-check never fired, 124 proxy shapes were created with unique aliases, and slanted members (belts, filament strand) were verified to end exactly at their anchor points.

Not verified here: rendering and the exact CoppeliaSim 4.10 behaviour of `sim.addDrawingObject` with a parent (the add-on follows the legacy plugin, which passed world coordinates with the bed as parent), `sim.drawing_itemsizes`/`sim.drawing_linestrip` availability (both are guarded with fallbacks), and the cone primitive's apex direction. The next CoppeliaSim session should run the built-in test print and confirm that beads sit on the build plate under the nozzle and move with the bed.
