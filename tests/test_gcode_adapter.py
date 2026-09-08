from __future__ import annotations

import json
import math
from pathlib import Path
import unittest

from rep5x_runtime import MachineProfile, Rep5xMotionAdapter, ToolPose, parse_gcode


ROOT = Path(__file__).resolve().parents[1]
PROFILE = MachineProfile.load(ROOT / "config" / "rep5x_ender3_v3_se.json")


class GCodeAndAdapterTests(unittest.TestCase):
    def test_units_modal_absolute_relative(self) -> None:
        program = parse_gcode("G21\nG90\nG1 X100 Y20 Z10 C90 B-30 F600\nG1 X120\nG91\nG1 Y5 C10\n", PROFILE)
        self.assertEqual(program.moves[1].pose, ToolPose(120, 20, 10, 90, -30))
        self.assertEqual(program.moves[2].pose, ToolPose(120, 25, 10, 100, -30))

    def test_100_mm_and_90_deg_boundary(self) -> None:
        adapter = Rep5xMotionAdapter(PROFILE)
        scene = adapter.tool_to_scene(ToolPose(100, 0, 100, 90, 0))
        self.assertAlmostEqual(scene.x_m, 0.1)
        self.assertAlmostEqual(scene.c_rad, math.pi / 2)

    def test_fibr_schema_order_and_no_double_conversion(self) -> None:
        adapter = Rep5xMotionAdapter(PROFILE)
        direct = adapter.tool_to_scene(ToolPose(100, 20, 100, 30, -20))
        fibr = adapter.fibr_sample_to_scene([0.1, 0.02, 0.1, math.radians(-20), math.radians(30)])
        for got, want in zip(fibr.as_tuple(), direct.as_tuple()):
            self.assertAlmostEqual(got, want, places=12)

    def test_xyz_only_fixture_keeps_rotaries_zero_and_warns(self) -> None:
        text = (ROOT / "tests" / "fixtures" / "slicer_xyz_smoke.gcode").read_text(encoding="utf-8")
        program = parse_gcode(text, PROFILE)
        self.assertEqual(program.compatibility_mode, "XYZ-only compatibility mode")
        self.assertTrue(any("CONICAL_META" in warning for warning in program.warnings))
        self.assertTrue(all(move.pose.c_deg == 0 and move.pose.b_deg == 0 for move in program.moves))

    def test_golden_sequences(self) -> None:
        fixture_dir = ROOT / "tests" / "fixtures"
        golden = json.loads((fixture_dir / "expected_pose_sequence.json").read_text(encoding="utf-8"))
        program = parse_gcode((fixture_dir / "rep5x_axis_smoke.gcode").read_text(encoding="utf-8"), PROFILE)
        actual = [{"X": m.pose.x_mm, "Y": m.pose.y_mm, "Z": m.pose.z_mm, "C": m.pose.c_deg, "B": m.pose.b_deg} for m in program.moves]
        self.assertEqual(actual, golden["rep5x_axis_smoke.gcode"])

    def test_limit_and_nonfinite_fail_loudly(self) -> None:
        with self.assertRaisesRegex(Exception, "soft limit"):
            parse_gcode("G1 X201", PROFILE)
        with self.assertRaisesRegex(Exception, "below soft limit"):
            Rep5xMotionAdapter(PROFILE).tool_to_scene(ToolPose(0, 20, 100, 0, -90))
        with self.assertRaisesRegex(Exception, "NaN/Infinity"):
            Rep5xMotionAdapter(PROFILE).tool_to_scene(ToolPose(math.nan, 0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
