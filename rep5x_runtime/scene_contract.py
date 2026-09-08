from __future__ import annotations

from dataclasses import dataclass
import math

from .kinematics import Rep5xKinematics
from .model import SceneJointPose, ToolPose
from .profile import MachineProfile


@dataclass(frozen=True)
class SceneNode:
    name: str
    parent: str | None


class SceneContractError(RuntimeError):
    pass


class Rep5xSceneContract:
    """Headless mirror of the Lua scene tree used for deterministic tests."""

    def __init__(self, profile: MachineProfile) -> None:
        s = profile.scene
        a = profile.axes
        self.profile = profile
        self.nodes = (
            SceneNode(s["root"], None),
            SceneNode(s["fixed_frame"], s["root"]),
            SceneNode(a["Y"].scene_joint, s["root"]),
            SceneNode(s["bed"], a["Y"].scene_joint),
            SceneNode(a["Z"].scene_joint, s["root"]),
            SceneNode(s["gantry"], a["Z"].scene_joint),
            SceneNode(a["X"].scene_joint, s["gantry"]),
            SceneNode(s["carriage"], a["X"].scene_joint),
            SceneNode(a["C"].scene_joint, s["carriage"]),
            SceneNode(s["c_link"], a["C"].scene_joint),
            SceneNode(a["B"].scene_joint, s["c_link"]),
            SceneNode(s["b_link"], a["B"].scene_joint),
            SceneNode(s["nozzle"], s["b_link"]),
            SceneNode(s["nozzle_tip"], s["nozzle"]),
        )
        self.validate_unique()

    def validate_unique(self) -> None:
        names = [node.name for node in self.nodes]
        duplicates = sorted({name for name in names if names.count(name) > 1})
        if duplicates:
            raise SceneContractError(f"duplicate required scene objects: {duplicates}")
        known = set(names)
        missing_parents = sorted({node.parent for node in self.nodes if node.parent is not None and node.parent not in known})
        if missing_parents:
            raise SceneContractError(f"missing required scene parents: {missing_parents}")

    def descendants(self, name: str) -> set[str]:
        children: dict[str, set[str]] = {}
        for node in self.nodes:
            if node.parent is not None:
                children.setdefault(node.parent, set()).add(node.name)
        result: set[str] = set()
        pending = list(children.get(name, ()))
        while pending:
            child = pending.pop()
            if child in result:
                continue
            result.add(child)
            pending.extend(children.get(child, ()))
        return result

    def nozzle_world_mm(self, tool: ToolPose) -> tuple[float, float, float]:
        """Physical nozzle world position; the bed carries machine Y compensation."""
        kin = Rep5xKinematics(self.profile.lc_mm, self.profile.lb_mm)
        machine = kin.inverse(tool)
        c = math.radians(tool.c_deg)
        b = math.radians(tool.b_deg)
        lc = self.profile.lc_mm
        lb = self.profile.lb_mm
        # C pivot at [0,+LC,+LB], B pivot at Rz(C)[0,-LC,0],
        # tip vector Rz(C)Ry(B)[0,0,-LB].
        gx = lc * math.sin(c) - lb * math.cos(c) * math.sin(b)
        gy = lc * (1.0 - math.cos(c)) - lb * math.sin(c) * math.sin(b)
        gz = lb * (1.0 - math.cos(b))
        return (machine.x_mm + gx, gy, machine.z_mm + gz)

    def bed_world_y_mm(self, tool: ToolPose) -> float:
        machine = Rep5xKinematics(self.profile.lc_mm, self.profile.lb_mm).inverse(tool)
        return -machine.y_mm

    def nozzle_relative_to_bed_mm(self, tool: ToolPose) -> tuple[float, float, float]:
        x, y, z = self.nozzle_world_mm(tool)
        return (x, y - self.bed_world_y_mm(tool), z)

    @staticmethod
    def orientation_matrix(tool: ToolPose) -> tuple[tuple[float, float, float], ...]:
        c = math.radians(tool.c_deg)
        b = math.radians(tool.b_deg)
        cc, sc, cb, sb = math.cos(c), math.sin(c), math.cos(b), math.sin(b)
        return (
            (cc * cb, -sc, cc * sb),
            (sc * cb, cc, sc * sb),
            (-sb, 0.0, cb),
        )


class AtomicSceneController:
    """Test double enforcing one complete five-joint write per simulation tick."""

    def __init__(self) -> None:
        self.tick = 0
        self.current = SceneJointPose(0.0, 0.0, 0.0, 0.0, 0.0)
        self.history: list[tuple[int, SceneJointPose]] = []

    def apply_tick(self, pose: SceneJointPose) -> None:
        if not all(math.isfinite(v) for v in pose.as_tuple()):
            raise SceneContractError("refusing non-finite joint write")
        self.tick += 1
        self.current = pose
        self.history.append((self.tick, pose))
