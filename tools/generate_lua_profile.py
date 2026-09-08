from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "rep5x_ender3_v3_se.json"
OUTPUT = ROOT / "coppeliasim" / "rep5x_profile.lua"


def q(value: str) -> str:
    return "'" + value.replace("\\", "\\\\").replace("'", "\\'") + "'"


def main() -> None:
    data = json.loads(CONFIG.read_text(encoding="utf-8"))
    axes, scene, kin = data["axes"], data["scene"], data["kinematics"]
    text = f"""-- Generated from config/rep5x_ender3_v3_se.json; do not edit by hand.
return {{
    schemaVersion={data['schema_version']},
    lc={kin['lc_mm'] / 1000.0!r},
    lb={kin['lb_mm'] / 1000.0!r},
    root={q(scene['root'])},
    joints={{X={q(axes['X']['scene_joint'])},Y={q(axes['Y']['scene_joint'])},Z={q(axes['Z']['scene_joint'])},C={q(axes['C']['scene_joint'])},B={q(axes['B']['scene_joint'])}}},
    objects={{frame={q(scene['fixed_frame'])},bed={q(scene['bed'])},gantry={q(scene['gantry'])},carriage={q(scene['carriage'])},cLink={q(scene['c_link'])},bLink={q(scene['b_link'])},nozzle={q(scene['nozzle'])},tip={q(scene['nozzle_tip'])},drawBoard={q(scene['draw_board'])}}},
    signs={{X={axes['X']['scene_sign']:g},Y={axes['Y']['scene_sign']:g},Z={axes['Z']['scene_sign']:g},C={axes['C']['scene_sign']:g},B={axes['B']['scene_sign']:g}}},
    limits={{X={{{axes['X']['soft_min']/1000.0!r},{axes['X']['soft_max']/1000.0!r}}},Y={{{axes['Y']['soft_min']/1000.0!r},{axes['Y']['soft_max']/1000.0!r}}},Z={{{axes['Z']['soft_min']/1000.0!r},{axes['Z']['soft_max']/1000.0!r}}},B={{{axes['B']['soft_min']}*math.pi/180,{axes['B']['soft_max']}*math.pi/180}}}},
}}
"""
    OUTPUT.write_text(text, encoding="utf-8", newline="\n")
    print(OUTPUT)


if __name__ == "__main__":
    main()
