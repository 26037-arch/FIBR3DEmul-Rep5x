from __future__ import annotations

import math

from .model import MachinePose, ToolPose


class Rep5xKinematics:
    """TCP control matching Rep5x-Marlin native_to_joint()."""

    def __init__(self, lc_mm: float = 0.0, lb_mm: float = 54.67) -> None:
        if not math.isfinite(lc_mm) or not math.isfinite(lb_mm) or lb_mm < 0:
            raise ValueError("LC/LB must be finite and LB must be non-negative")
        self.lc_mm = float(lc_mm)
        self.lb_mm = float(lb_mm)

    def inverse(self, tip: ToolPose) -> MachinePose:
        tip.validate()
        c = math.radians(tip.c_deg)
        b = math.radians(tip.b_deg)
        x = tip.x_mm - math.sin(c) * self.lc_mm + math.cos(c) * math.sin(b) * self.lb_mm
        y = tip.y_mm + (math.cos(c) - 1.0) * self.lc_mm + math.sin(c) * math.sin(b) * self.lb_mm
        z = tip.z_mm + (math.cos(b) - 1.0) * self.lb_mm
        return MachinePose(x, y, z, tip.c_deg, tip.b_deg)

    def forward(self, joints: MachinePose) -> ToolPose:
        c = math.radians(joints.c_deg)
        b = math.radians(joints.b_deg)
        return ToolPose(
            joints.x_mm + math.sin(c) * self.lc_mm - math.cos(c) * math.sin(b) * self.lb_mm,
            joints.y_mm - (math.cos(c) - 1.0) * self.lc_mm - math.sin(c) * math.sin(b) * self.lb_mm,
            joints.z_mm - (math.cos(b) - 1.0) * self.lb_mm,
            joints.c_deg,
            joints.b_deg,
        )
