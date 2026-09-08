# Baseline audit

Audit date: 2026-09-08 (Asia/Seoul). All statements below are either tied to an inspected file/commit or explicitly marked **unknown**.

## Source baselines

| Repository | default branch observed after clone | commit | license observed |
|---|---|---|---|
| `neuebot/FIBR3DEmul` | `master` | `bd2e549ddcc47e295edfb803f8d82aa1a0194147` | GPL-3.0 text in root `LICENSE` |
| `dennisklappe/Rep5x` | `main` | `9c437d5cce7766d641e7aa35ced2c63231148a1f` | README declares GPL v3, but the referenced root `LICENSE` file is absent at this commit |
| `KJH-code/slicer` | `main` | `0d028f6fd325faed13d450a3163887e17d5e9767` | MIT in root `LICENSE` |
| `dennisklappe/Rep5x-Marlin` (supporting verification only) | requested branch `Marlin2ForPipetBot` | `d25ee4e4cc62c8f8f9ae5eec774d6fe6c6bb5da2` | GPL-3.0-or-later headers in `penta_axis_head_head.cpp` |

The slicer source was inspected only; it was not modified or copied into this implementation.

## Original FIBR3DEmul build and loading contract

The upstream README requires Windows, CoppeliaSim 4.0.0 minimum, CMake, Visual Studio, Boost >=1.64, and Eigen 3. The plugin itself rejects versions older than **4.0.0 rev1** in `simExtFIBR3D.cpp:842-850`. Upstream instructs the user to copy `simExtFIBR3D.dll` into the CoppeliaSim application root, open `example_scene.ttt`, start simulation, then launch the C# interpreter.

The checked-in upstream CMake configuration is not self-contained: it calls `find_package(CoppeliaSim 4.0.0.4 REQUIRED)` through `programming/libPlugin` and references `callbacks.xml` and `simExtPluginSkeletonNG.lua`, but those files are absent from the audited checkout. The source also vendors an older regular-API binding in `v-rep_plugin/coppelia`. Phase 1 therefore adds a narrow bundled-API CMake compatibility path and retains the original `simStart` / `simMessage` entrypoints.

## Unmodified baseline attempts on this host

Environment observed:

- Windows x64; CMake 3.29.2; Git 2.55.0; Python 3.12.10.
- Visual Studio 2022 Build Tools/MSBuild 17.14 is present, but the C/C++ compiler workload is not installed.
- .NET 8 runtimes are present, but no .NET SDK and no .NET Framework 4.5.2 targeting pack are installed.
- No `CoppeliaSim*` directory was found under `C:\Program Files` or `C:\Program Files (x86)`.

Reproduction and result:

```powershell
cmake -S v-rep_plugin -B baseline-cmake -G "Visual Studio 17 2022" -A x64
```

After redirecting `TEMP` into the writable workspace, configure stopped with `No CMAKE_C_COMPILER could be found` and `No CMAKE_CXX_COMPILER could be found`. This occurs before the upstream missing-libPlugin issue can be reached on this host.

```powershell
& 'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\amd64\MSBuild.exe' `
  gcode_interpreter\GCodeInterpreter.sln /t:Rebuild /p:Configuration=Release /v:minimal
```

This stopped at `MSB3644`: `.NETFramework,Version=v4.5.2` reference assemblies are missing. The committed upstream executable and DLL were not treated as a successful baseline because CoppeliaSim is absent and their provenance/runtime compatibility cannot be verified here.

## Parser -> JSON -> plugin -> scene data flow

1. `Parser.cs` normalizes motion words, holds a five-element modal `PrintState.position`, and maps it explicitly as `[X,Y,Z,B,C]` (`Parser.cs:610-612`, `1071-1085`). Omitted axes retain their previous value because `GetReferencePosition` only overwrites words present in the line.
2. `Translator.cs` serializes `PackageJSON` with Newtonsoft.Json. It sends one object containing `keycode`, `clkwise`, `interpol`, `extrude`, `text`, `aux`, `pos1[5]`, `pos2[5]`, `plane`, `acc[3]`, `vel[3]`, and `line` (`Translator.cs:48-81`). The code comment saying “mm and radians” is inaccurate for rotary positions: the trajectory layer later treats B/C as degrees.
3. `SocketClient.cs` connects to TCP ports 1313 (command/echo) and 1315 (line/collision status). `Server.cpp` creates the two acceptors. The command side uses `async_read_some` and assumes one complete JSON object per read; there is no explicit length/delimiter framing. That pre-existing limitation is retained and documented.
4. `Listener.cpp` deserializes JSON, passes the requested move to `Trajectory`, extends extrusion and line-number arrays to the same sample count, then calls `v_repExtPQSetJointTrajectory`.
5. `Trajectory.cpp` creates time samples. It converts XYZ mm to m and source indices 3/4 degrees to radians at sample creation (`Trajectory.cpp:193-204`, `259-265` and corresponding arc/circle paths).
6. `simExtFIBR3D.cpp` queues samples, and `simMessage(...modulehandle...)` consumes one sample per simulator callback and calls `simSetJointPosition` for every joint (`simExtFIBR3D.cpp:1094-1108` after this change). The Rep5x adapter now sits immediately before that queue.

The outbound status logger performs the reverse display conversions at `simExtFIBR3D.cpp:1082-1090`: m to mm and rad to deg. `Listener.cpp:109-114` similarly converts its last SI trajectory sample back to mm/deg before generating the next move. No position-unit conversion occurs in the C# translator.

## Original scene contract

Names that are provably referenced by source are:

- printer root supplied by Lua to `simFIBR3D.init`;
- `AxisX_joint`, `AxisY_joint`, `AxisZ_joint`, and for 5 DoF `AxisB_joint`, `AxisC_joint`;
- `Extruder`, `DrawBoard`, and optional `FilamentOctree`.

The exact parent-child hierarchy inside `example_scene.ttt` is **unknown on this host**. The file is a binary V-REP scene (`VREP` header); no object aliases are visible as plain text, and CoppeliaSim is not installed to query the tree. The new Rep5x scene does not claim the old hierarchy; it creates and validates a separate explicit hierarchy documented in `scene-hierarchy.md`.

## Current CoppeliaSim compatibility conclusion

Runtime compatibility of the original DLL is **not verified** because CoppeliaSim is absent. The original API calls such as `simGetObjectHandle` are still documented but deprecated in current manuals in favor of `sim.getObject`; the legacy remote API was deprecated as of 4.4 in favor of ZeroMQ. This project does not replace the internal FIBR3D TCP protocol in phase 1. It uses current Lua regular-API calls for the scene builder and keeps a bounded legacy C++ compatibility build for the original parser path.

References: [CoppeliaSim `sim.getObjectHandle` deprecation](https://manual.coppeliarobotics.com/en/sim/simGetObjectHandle.htm), [legacy remote API status](https://manual.coppeliarobotics.com/en/legacyRemoteApiOverview.htm), [current remote API overview](https://manual.coppeliarobotics.com/en/remoteApiOverview.htm).

## Rep5x mechanical findings

- C is the head yaw about Z; B is the downstream tilt axis, parallel to Y at the zero pose (`tools/firmware-builder/js/config-generator.js:317-321`). Rotation order is therefore C then B.
- The audited Rep5x configuration gives X `[0,200]` mm, Y `[-40,200]` mm, Z `[0,174.6]` mm, C `[-360,360]` in that example, and B `[-135,135]` degrees (`build-guide/universal-parts/firmware/Configuration.h:1939-1949`). The project README describes the design intent as continuous C rotation, so the simulator profile treats C as cyclic rather than enforcing the example firmware window.
- `LC` is the Y distance between C and B centerlines; `LB` is the Z distance from B centerline to the vertical zero-pose nozzle tip (`Configuration.h:1959-1971`). Phase-1 defaults are LC=0 and LB=54.67 mm as requested.
- The Rep5x setup UI says positive-test C should be counter-clockwise from above and positive B tilts the nozzle left from a front view (`tools/printer-setup/index.html:208-239`, `283-319`). Another firmware-builder note says its *motor default* is inverted to obtain CNC-positive clockwise C. Because those statements describe different layers and conflict if read as one scene convention, the profile makes signs explicit and uses mathematical/right-hand positive C and positive-B-left; motor wiring inversion remains outside this simulation.
- Asset attachment and confidence are recorded per file in `assets/manifest.json`. None of the supplied 3MF files supplies the complete Ender-3 V3 SE CAD or a validated assembly mating transform.
