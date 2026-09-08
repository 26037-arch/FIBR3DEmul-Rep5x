from __future__ import annotations

import math
import random
import unittest

from rep5x_runtime import Rep5xKinematics, ToolPose


class KinematicsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.kin = Rep5xKinematics(lc_mm=10.0, lb_mm=54.67)

    def assertDelta(self, b: float, c: float, expected: tuple[float, float, float]) -> None:
        tip = ToolPose(100.0, 80.0, 60.0, c, b)
        machine = self.kin.inverse(tip)
        actual = (machine.x_mm - tip.x_mm, machine.y_mm - tip.y_mm, machine.z_mm - tip.z_mm)
        for got, want in zip(actual, expected):
            self.assertAlmostEqual(got, want, places=9)

    def test_numeric_reference_poses(self) -> None:
        lb, lc = 54.67, 10.0
        self.assertDelta(0, 0, (0, 0, 0))
        self.assertDelta(-90, 0, (-lb, 0, -lb))
        self.assertDelta(90, 0, (lb, 0, -lb))
        self.assertDelta(0, 90, (-lc, -lc, 0))
        self.assertDelta(0, 180, (0, -2 * lc, 0))

    def test_100_random_round_trips_below_one_e_minus_6_mm(self) -> None:
        rng = random.Random(0x5A17)
        max_error = 0.0
        for _ in range(100):
            tip = ToolPose(rng.uniform(0, 200), rng.uniform(-40, 200), rng.uniform(0, 174.6), rng.uniform(-720, 720), rng.uniform(-135, 135))
            recovered = self.kin.forward(self.kin.inverse(tip))
            error = max(abs(recovered.x_mm - tip.x_mm), abs(recovered.y_mm - tip.y_mm), abs(recovered.z_mm - tip.z_mm))
            max_error = max(max_error, error)
        self.assertLessEqual(max_error, 1e-6)


if __name__ == "__main__":
    unittest.main()
