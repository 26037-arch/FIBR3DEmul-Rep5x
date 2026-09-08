# Rep5x virtual printer — phase 1

This branch turns FIBR3DEmul into a Rep5x-on-Ender-3-V3-SE virtual printer without talking to physical hardware. It keeps the upstream parser → JSON/socket → listener → trajectory path, adds an explicit Rep5x motion adapter, and supplies a generated CoppeliaSim scene with X/Y/Z/C/B controls. A local Lua G-code runner provides an independently usable test path when the legacy C# parser/plugin is not built.

## What is implemented

- Explicit units and axis boundary: G-code `X/Y/Z` are mm, `B/C` are degrees; CoppeliaSim receives metres/radians in scene order `X/Y/Z/C/B`. Upstream trajectory samples remain `X/Y/Z/B/C` and are reordered once.
- Rep5x inverse kinematics with `LC=0 mm`, `LB=54.67 mm`, C yaw about Z, then B tilt about the rotated Y axis.
- Generated Ender-style proxy scene with a moving Y bed, Z gantry, X carriage, continuous C joint, limited B joint, nozzle, TCP marker, and duplicate/missing-handle validation.
- Add-on UI: build/connect scene, open G-code, start/pause/resume/stop, `0.1×–5×` speed, home, per-axis jog, current pose, and nozzle world position.
- G-code subset: `G0/G1`, `G20/G21` (also legacy `G70/G71`), `G90/G91`, `G92`, modal `X/Y/Z/B/C`, standard `E/F`, and non-motion `M` warnings. XYZ-only slicer files leave B/C at zero. `CONICAL_META` is intentionally ignored in phase 1.
- Input, soft-limit, NaN/Infinity, trajectory shape, empty queue, profile, scene-object, and joint-handle checks fail with line/object context.
- Deterministic kinematics, parser, unit-boundary, hierarchy, continuous-motion, STL-conversion, and slicer-compatibility tests.

Extrusion-mesh generation, collision/material simulation, firmware emulation, calibration, and real-printer control are outside phase 1. For the Rep5x profile the legacy extrusion writer is deliberately disabled because it assumes the original Cartesian joint order.

## Quick verification (no CoppeliaSim required)

From a clean checkout in PowerShell:

```powershell
python -m unittest discover -s tests -v
python -m rep5x_runtime.cli tests\fixtures\rep5x_axis_smoke.gcode --json
python -m rep5x_runtime.cli tests\fixtures\slicer_xyz_smoke.gcode --json
python tools\generate_evidence.py
```

Python 3.10+ is sufficient; the headless verifier uses only the standard library. Generated evidence is written to `evidence/`.

## Windows build and CoppeliaSim setup

Install these prerequisites:

- CoppeliaSim 4.6 or newer with the simUI module. The plugin retains the vendored legacy 4.0 regular-API ABI; runtime validation is still required for the exact CoppeliaSim release you deploy.
- Visual Studio 2022 “Desktop development with C++”, CMake 3.16+, and a compiled Boost distribution with the `system` component (Boost 1.64+).
- To rebuild the original desktop parser: Visual Studio/MSBuild plus the .NET Framework 4.5.2 Developer Pack. NuGet restore supplies Newtonsoft.Json 10.0.2.

Build the plugin in an x64 Developer PowerShell. Adjust the two installation paths:

```powershell
$env:COPPELIASIM_ROOT_DIR = 'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu'
$env:BOOST_ROOT = 'C:\local\boost_1_84_0'
cmake -S v-rep_plugin -B v-rep_plugin\build-rep5x -A x64
cmake --build v-rep_plugin\build-rep5x --config Release
powershell -ExecutionPolicy Bypass -File tools\install_coppeliasim.ps1 -CoppeliaRoot $env:COPPELIASIM_ROOT_DIR
```

Build the original parser if you want to exercise its socket path:

```powershell
nuget restore gcode_interpreter\GCodeInterpreter.sln
msbuild gcode_interpreter\GCodeInterpreter.sln /p:Configuration=Release
```

Restart CoppeliaSim. Open **Add-ons → Rep5x Scene Builder**, click **Build scene**, and then start simulation. The add-on remains useful without the DLL: open either file from `tests\fixtures` and use its local playback controls. With the DLL present, simulation start calls `simFIBR3D.init('Rep5x_Ender3V3SE', 5, <profile>)`; launch the original GCodeInterpreter afterward to exercise the preserved network pipeline.

No serial port, USB device, firmware endpoint, or printer network address is opened anywhere in the Rep5x add-on/runtime.

## Machine contract

The single source of truth is `config/rep5x_ender3_v3_se.json`. The generated Lua copy is refreshed with:

```powershell
python tools\generate_lua_profile.py
```

The canonical tool-to-machine equations are:

```text
Xmachine = Xtip - sin(C)·LC + cos(C)·sin(B)·LB
Ymachine = Ytip + (cos(C)-1)·LC + sin(C)·sin(B)·LB
Zmachine = Ztip + (cos(B)-1)·LB
```

Angles in these equations are radians internally. At `C=0`, positive B tilts the nozzle toward negative X in the generated scene. Positive C is mathematical counter-clockwise viewed from +Z. Motor-driver inversion from firmware configuration is not applied to this digital kinematic coordinate system; change only the profile `scene_sign` values if a target scene uses opposite joint signs.

Detailed derivation and ambiguity notes are in `docs/kinematics-convention.md`; the object tree and joint behavior are in `docs/scene-hierarchy.md`.

## Rep5x CAD audit

`tools/convert_3mf_assets.py` converts all ten audited Rep5x `.3mf` sources to binary STL while applying 3MF unit/build/component transforms and failing on unresolved references. Reproduce it against the pinned Rep5x checkout:

```powershell
python tools\convert_3mf_assets.py --rep5x-root C:\path\to\Rep5x --output assets\rep5x_stl --report assets\conversion-report.json
```

The converted files are included for inspection, but are not auto-attached to the generated scene: the Rep5x repository does not provide authoritative assembly mating transforms. Guessing them would make the visual assembly less trustworthy than the dimensioned proxy geometry. Provenance, hashes, confidence, and this limitation are recorded in `assets/manifest.json` and `THIRD_PARTY_NOTICES.md`.

## Evidence and known limitations

- `evidence/axis-motion-log.json` and `.csv`: every fixture move, scene joint value, and independently reconstructed TCP error.
- `evidence/axis-motion-evidence.svg` (and optional PNG when matplotlib is installed): visual trace of the isolated/compound five-axis smoke fixture.
- `docs/test-results.md`: commands, results, and environment-specific blockers.
- `docs/baseline-audit.md`: pinned repository commits, original data flow, units, scene contract, and license audit.

The current delivery was fully headless-tested. It was not rendered in CoppeliaSim on the delivery machine because CoppeliaSim is not installed, and its C++/C# projects could not be rebuilt there because the MSVC C++ workload and .NET Framework 4.5.2 targeting pack are absent. Those gaps are recorded rather than represented as successful runtime verification.
