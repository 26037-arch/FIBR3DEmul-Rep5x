from __future__ import annotations

import math
from collections.abc import Sequence

from .kinematics import Rep5xKinematics
from .model import SceneJointPose, ToolPose, require_finite
from .profile import MachineProfile


class Rep5xMotionAdapter:
    """Explicit boundary between FIBR3D trajectory samples and the Rep5x scene."""

    def __init__(self, profile: MachineProfile) -> None:
        self.profile = profile
        self.kinematics = Rep5xKinematics(profile.lc_mm, profile.lb_mm)

    def tool_to_scene(self, tip: ToolPose, *, line: int | None = None) -> SceneJointPose:
        """Convert mm/deg exactly once at the CoppeliaSim boundary."""
        self.profile.validate_tool_pose(tip, line=line)
        machine = self.kinematics.inverse(tip)
        self.profile.validate_machine_pose(machine, line=line)
        a = self.profile.axes
        return SceneJointPose(
            (machine.x_mm + a["X"].zero_offset) * a["X"].scene_sign / 1000.0,
            (machine.y_mm + a["Y"].zero_offset) * a["Y"].scene_sign / 1000.0,
            (machine.z_mm + a["Z"].zero_offset) * a["Z"].scene_sign / 1000.0,
            math.radians(machine.c_deg + a["C"].zero_offset) * a["C"].scene_sign,
            math.radians(machine.b_deg + a["B"].zero_offset) * a["B"].scene_sign,
        )

    def fibr_sample_to_scene(self, sample: Sequence[float], *, line: int | None = None) -> SceneJointPose:
        """Map FIBR3D's [X,Y,Z,B,C] SI sample to scene [X,Y,Z,C,B].

        FIBR3D Trajectory has already converted linear values to metres and
        angles to radians.  This method does not apply that conversion again.
        """
        if len(sample) != 5:
            raise ValueError(f"FIBR3D trajectory schema mismatch: expected 5 coordinates, got {len(sample)}")
        x_m, y_m, z_m, b_rad, c_rad = map(float, sample)
        require_finite(x_m=x_m, y_m=y_m, z_m=z_m, b_rad=b_rad, c_rad=c_rad)
        tip = ToolPose(x_m * 1000.0, y_m * 1000.0, z_m * 1000.0, math.degrees(c_rad), math.degrees(b_rad))
        return self.tool_to_scene(tip, line=line)
