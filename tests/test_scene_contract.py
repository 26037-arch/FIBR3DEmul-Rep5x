from __future__ import annotations

import math
from pathlib import Path
import unittest

from rep5x_runtime import MachineProfile, Rep5xMotionAdapter, ToolPose
from rep5x_runtime.scene_contract import AtomicSceneController, Rep5xSceneContract


ROOT = Path(__file__).resolve().parents[1]
PROFILE = MachineProfile.load(ROOT / "config" / "rep5x_ender3_v3_se.json")


class SceneContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.scene = Rep5xSceneContract(PROFILE)

    def test_required_handles_are_unique_and_complete(self) -> None:
        self.scene.validate_unique()
        required = set(PROFILE.scene.values()) | {axis.scene_joint for axis in PROFILE.axes.values()}
        self.assertTrue(required - {"DrawBoard"} <= {node.name for node in self.scene.nodes})

    def test_parent_child_motion_sets(self) -> None:
        a, s = PROFILE.axes, PROFILE.scene
        self.assertIn(s["carriage"], self.scene.descendants(a["X"].scene_joint))
        self.assertEqual(self.scene.descendants(a["Y"].scene_joint), {s["bed"]})
        self.assertIn(s["nozzle"], self.scene.descendants(a["Z"].scene_joint))
        self.assertIn(a["B"].scene_joint, self.scene.descendants(a["C"].scene_joint))
        self.assertIn(s["nozzle"], self.scene.descendants(a["B"].scene_joint))

    def test_nozzle_tcp_matches_independent_geometry(self) -> None:
        for pose in (ToolPose(100, 90, 130, 135, -35), ToolPose(20, -10, 100, 90, 45)):
            relative = self.scene.nozzle_relative_to_bed_mm(pose)
            for got, want in zip(relative, (pose.x_mm, pose.y_mm, pose.z_mm)):
                self.assertLessEqual(abs(got - want), 0.1)

    def test_atomic_sweep_no_nan_no_360_jump_or_underflow(self) -> None:
        adapter = Rep5xMotionAdapter(PROFILE)
        controller = AtomicSceneController()
        prior_c = 0.0
        for index in range(101):
            c = -180.0 + 360.0 * index / 100.0
            b = 90.0 * math.sin(math.radians(c))
            sample = adapter.tool_to_scene(ToolPose(100, 80, 120, c, b))
            controller.apply_tick(sample)
            self.assertTrue(all(math.isfinite(v) for v in sample.as_tuple()))
            self.assertLessEqual(abs(c - prior_c), 180.0 if index == 0 else 3.6000001)
            prior_c = c
        self.assertEqual(len(controller.history), 101)
        self.assertEqual(controller.tick, 101)


if __name__ == "__main__":
    unittest.main()
