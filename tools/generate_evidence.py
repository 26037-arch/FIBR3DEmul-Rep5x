from __future__ import annotations

import csv
import html
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rep5x_runtime.gcode import load_gcode
from rep5x_runtime.motion_adapter import Rep5xMotionAdapter
from rep5x_runtime.profile import MachineProfile
from rep5x_runtime.scene_contract import Rep5xSceneContract


OUT = ROOT / "evidence"


def main() -> None:
    profile = MachineProfile.load(ROOT / "config" / "rep5x_ender3_v3_se.json")
    adapter = Rep5xMotionAdapter(profile)
    contract = Rep5xSceneContract(profile)
    records: list[dict[str, object]] = []

    for fixture in ("rep5x_axis_smoke.gcode", "slicer_xyz_smoke.gcode"):
        program = load_gcode(ROOT / "tests" / "fixtures" / fixture, profile)
        for sample, move in enumerate(program.moves, 1):
            joints = adapter.tool_to_scene(move.pose, line=move.line)
            nozzle = contract.nozzle_relative_to_bed_mm(move.pose)
            records.append({
                "fixture": fixture,
                "sample": sample,
                "gcode_line": move.line,
                "mode": program.compatibility_mode,
                "tool_X_mm": move.pose.x_mm,
                "tool_Y_mm": move.pose.y_mm,
                "tool_Z_mm": move.pose.z_mm,
                "tool_C_deg": move.pose.c_deg,
                "tool_B_deg": move.pose.b_deg,
                "joint_X_m": joints.x_m,
                "joint_Y_m": joints.y_m,
                "joint_Z_m": joints.z_m,
                "joint_C_rad": joints.c_rad,
                "joint_B_rad": joints.b_rad,
                "tcp_error_mm": max(abs(nozzle[i] - (move.pose.x_mm, move.pose.y_mm, move.pose.z_mm)[i]) for i in range(3)),
            })

    OUT.mkdir(exist_ok=True)
    (OUT / "axis-motion-log.json").write_text(json.dumps(records, indent=2) + "\n", encoding="utf-8")
    with (OUT / "axis-motion-log.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)

    rep5x = [r for r in records if r["fixture"] == "rep5x_axis_smoke.gcode"]
    width, height = 960, 520
    left, top, plot_w, plot_h = 74, 62, 820, 350
    colors = {"X": "#42a5f5", "Y": "#66bb6a", "Z": "#ffa726", "C": "#ab47bc", "B": "#ef5350"}
    series = {axis: [float(r[f"tool_{axis}_{'mm' if axis in 'XYZ' else 'deg'}"]) for r in rep5x] for axis in "XYZCB"}
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
             '<rect width="100%" height="100%" fill="#10151d"/>',
             '<text x="48" y="34" fill="#f5f7fa" font-family="Segoe UI, sans-serif" font-size="22">Rep5x phase-1 isolated-axis replay</text>',
             f'<rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="#171e29" stroke="#445064"/>']
    for tick in range(5):
        y = top + tick * plot_h / 4
        lines.append(f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_w}" y2="{y:.1f}" stroke="#293445"/>')
    n = len(rep5x)
    for axis, values in series.items():
        lo, hi = min(values), max(values)
        span = hi - lo or 1.0
        points = []
        for i, value in enumerate(values):
            x = left + i * plot_w / max(n - 1, 1)
            y = top + plot_h - (value - lo) * plot_h / span
            points.append(f"{x:.1f},{y:.1f}")
        lines.append(f'<polyline points="{html.escape(" ".join(points))}" fill="none" stroke="{colors[axis]}" stroke-width="3"/>')
    for i, axis in enumerate("XYZCB"):
        x = left + i * 125
        unit = "mm" if axis in "XYZ" else "deg"
        lines.append(f'<line x1="{x}" y1="448" x2="{x+24}" y2="448" stroke="{colors[axis]}" stroke-width="4"/>')
        lines.append(f'<text x="{x+31}" y="454" fill="#dbe4f0" font-family="Segoe UI, sans-serif" font-size="15">{axis} ({unit})</text>')
    lines.append(f'<text x="{left}" y="493" fill="#a9b6c8" font-family="Segoe UI, sans-serif" font-size="14">{n} atomic pose samples; TCP reconstruction error max {max(float(r["tcp_error_mm"]) for r in records):.3e} mm</text>')
    lines.append('</svg>')
    (OUT / "axis-motion-evidence.svg").write_text("\n".join(lines) + "\n", encoding="utf-8")

    # PNG is a convenience artifact when matplotlib is available; JSON/CSV/SVG
    # generation deliberately remains standard-library-only.
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    figure, axis_plot = plt.subplots(figsize=(9.6, 5.2), dpi=120)
    for axis_name, values in series.items():
        axis_plot.plot(range(1, n + 1), values, marker="o", label=axis_name, color=colors[axis_name])
    axis_plot.set_title("Rep5x phase-1 isolated-axis replay")
    axis_plot.set_xlabel("Atomic pose sample")
    axis_plot.set_ylabel("Per-series value (XYZ mm; C/B deg)")
    axis_plot.grid(alpha=0.25)
    axis_plot.legend(ncol=5)
    figure.tight_layout()
    figure.savefig(OUT / "axis-motion-evidence.png")
    plt.close(figure)


if __name__ == "__main__":
    main()
