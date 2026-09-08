from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

from .model import MachinePose, ToolPose


class ProfileError(ValueError):
    pass


@dataclass(frozen=True)
class AxisProfile:
    name: str
    scene_joint: str
    kind: str
    scene_sign: float
    zero_offset: float
    soft_min: float | None
    soft_max: float | None
    unit: str
    continuous: bool

    @classmethod
    def from_json(cls, name: str, data: dict[str, Any]) -> "AxisProfile":
        required = {"scene_joint", "kind", "scene_sign", "zero_offset", "soft_min", "soft_max", "unit", "continuous"}
        missing = sorted(required - data.keys())
        if missing:
            raise ProfileError(f"axis {name}: missing keys {missing}")
        sign = float(data["scene_sign"])
        if sign not in (-1.0, 1.0):
            raise ProfileError(f"axis {name}: scene_sign must be +1 or -1")
        return cls(name=name, scene_sign=sign, **{k: data[k] for k in required if k != "scene_sign"})

    def check(self, value: float, *, line: int | None = None) -> None:
        if self.continuous:
            return
        if self.soft_min is not None and value < self.soft_min - 1e-12:
            where = f" at G-code line {line}" if line is not None else ""
            raise ProfileError(f"{self.name}={value:g}{self.unit}{where} is below soft limit {self.soft_min:g}{self.unit}")
        if self.soft_max is not None and value > self.soft_max + 1e-12:
            where = f" at G-code line {line}" if line is not None else ""
            raise ProfileError(f"{self.name}={value:g}{self.unit}{where} exceeds soft limit {self.soft_max:g}{self.unit}")


@dataclass(frozen=True)
class MachineProfile:
    path: Path
    profile_id: str
    lc_mm: float
    lb_mm: float
    axes: dict[str, AxisProfile]
    scene: dict[str, str]
    raw: dict[str, Any]

    @classmethod
    def load(cls, path: str | Path) -> "MachineProfile":
        resolved = Path(path).resolve()
        try:
            raw = json.loads(resolved.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ProfileError(f"cannot load machine profile {resolved}: {exc}") from exc
        if raw.get("schema_version") != 1:
            raise ProfileError(f"unsupported profile schema_version: {raw.get('schema_version')!r}")
        axis_data = raw.get("axes", {})
        if set(axis_data) != {"X", "Y", "Z", "C", "B"}:
            raise ProfileError("profile axes must be exactly X,Y,Z,C,B")
        axes = {name: AxisProfile.from_json(name, axis_data[name]) for name in axis_data}
        kinematics = raw.get("kinematics", {})
        return cls(
            path=resolved,
            profile_id=str(raw["profile_id"]),
            lc_mm=float(kinematics["lc_mm"]),
            lb_mm=float(kinematics["lb_mm"]),
            axes=axes,
            scene=dict(raw["scene"]),
            raw=raw,
        )

    def validate_tool_pose(self, pose: ToolPose, *, line: int | None = None) -> None:
        pose.validate()
        values = {"X": pose.x_mm, "Y": pose.y_mm, "Z": pose.z_mm, "C": pose.c_deg, "B": pose.b_deg}
        for name, value in values.items():
            self.axes[name].check(value, line=line)

    def validate_machine_pose(self, pose: MachinePose, *, line: int | None = None) -> None:
        values = {"X": pose.x_mm, "Y": pose.y_mm, "Z": pose.z_mm, "C": pose.c_deg, "B": pose.b_deg}
        for name, value in values.items():
            self.axes[name].check(value, line=line)
