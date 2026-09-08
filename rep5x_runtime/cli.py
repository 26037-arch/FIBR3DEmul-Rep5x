from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from .gcode import load_gcode
from .motion_adapter import Rep5xMotionAdapter
from .profile import MachineProfile
from .scene_contract import Rep5xSceneContract


def main(argv: list[str] | None = None) -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Validate and replay Rep5x G-code without hardware I/O")
    parser.add_argument("gcode", type=Path)
    parser.add_argument("--profile", type=Path, default=root / "config" / "rep5x_ender3_v3_se.json")
    parser.add_argument("--json", action="store_true", help="emit machine-readable final pose")
    args = parser.parse_args(argv)

    try:
        profile = MachineProfile.load(args.profile)
        program = load_gcode(args.gcode, profile)
        adapter = Rep5xMotionAdapter(profile)
        contract = Rep5xSceneContract(profile)
        final_move = program.moves[-1]
        scene = adapter.tool_to_scene(final_move.pose, line=final_move.line)
        nozzle_world = contract.nozzle_world_mm(final_move.pose)
        nozzle_bed = contract.nozzle_relative_to_bed_mm(final_move.pose)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    result = {
        "mode": program.compatibility_mode,
        "moves": len(program.moves),
        "warnings": list(program.warnings),
        "final_tool_pose_mm_deg": final_move.pose.__dict__,
        "final_scene_joints_m_rad": scene.as_tuple(),
        "nozzle_world_mm": nozzle_world,
        "nozzle_relative_to_bed_mm": nozzle_bed,
    }
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print(f"mode: {program.compatibility_mode}")
        print(f"moves: {len(program.moves)}")
        for warning in program.warnings:
            print(f"WARNING: {warning}")
        print("final X/Y/Z/C/B: " + ", ".join(f"{v:.6f}" for v in scene.as_tuple()))
        print("nozzle world mm: " + ", ".join(f"{v:.6f}" for v in nozzle_world))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
