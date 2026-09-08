from __future__ import annotations

from dataclasses import dataclass
import math


class MotionContractError(ValueError):
    """A pose cannot safely cross the motion boundary."""


def require_finite(**values: float) -> None:
    bad = [name for name, value in values.items() if not math.isfinite(value)]
    if bad:
        raise MotionContractError(f"NaN/Infinity is not allowed for: {', '.join(bad)}")


@dataclass(frozen=True)
class ToolPose:
    """Requested TCP pose in G-code units: millimetres and degrees."""

    x_mm: float = 0.0
    y_mm: float = 0.0
    z_mm: float = 0.0
    c_deg: float = 0.0
    b_deg: float = 0.0

    def validate(self) -> "ToolPose":
        require_finite(
            x_mm=self.x_mm,
            y_mm=self.y_mm,
            z_mm=self.z_mm,
            c_deg=self.c_deg,
            b_deg=self.b_deg,
        )
        return self


@dataclass(frozen=True)
class MachinePose:
    """Rep5x machine joint coordinates in millimetres and degrees."""

    x_mm: float
    y_mm: float
    z_mm: float
    c_deg: float
    b_deg: float


@dataclass(frozen=True)
class SceneJointPose:
    """CoppeliaSim boundary values in X,Y,Z,C,B order (metres/radians)."""

    x_m: float
    y_m: float
    z_m: float
    c_rad: float
    b_rad: float

    def as_tuple(self) -> tuple[float, float, float, float, float]:
        return (self.x_m, self.y_m, self.z_m, self.c_rad, self.b_rad)
