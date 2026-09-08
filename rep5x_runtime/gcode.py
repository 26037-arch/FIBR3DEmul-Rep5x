from __future__ import annotations

from dataclasses import dataclass
import math
import re
from pathlib import Path

from .model import ToolPose
from .profile import MachineProfile


class GCodeError(ValueError):
    def __init__(self, line: int, raw: str, message: str) -> None:
        super().__init__(f"line {line}: {message}; source={raw.rstrip()!r}")
        self.line = line
        self.raw = raw


@dataclass(frozen=True)
class Move:
    line: int
    raw: str
    motion: str
    pose: ToolPose
    feed_mm_min: float | None
    extrusion: float | None


@dataclass(frozen=True)
class GCodeProgram:
    moves: tuple[Move, ...]
    warnings: tuple[str, ...]
    compatibility_mode: str


_WORD = re.compile(r"([A-Za-z])\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)")
_PAREN_COMMENT = re.compile(r"\([^)]*\)")


def _strip_comments(raw: str) -> str:
    return _PAREN_COMMENT.sub("", raw).split(";", 1)[0].strip()


def parse_gcode(source: str, profile: MachineProfile) -> GCodeProgram:
    absolute = True
    linear_scale = 1.0
    current = {"X": 0.0, "Y": 0.0, "Z": 0.0, "C": 0.0, "B": 0.0}
    feed: float | None = None
    moves: list[Move] = []
    warnings: list[str] = []
    has_rotary = False
    saw_conical_meta = False

    for line_no, raw in enumerate(source.splitlines(), 1):
        if "CONICAL_META" in raw.upper():
            saw_conical_meta = True
        code = _strip_comments(raw)
        if not code or code.startswith("%"):
            continue
        words = [(letter.upper(), float(number)) for letter, number in _WORD.findall(code)]
        if not words:
            raise GCodeError(line_no, raw, "unsupported or malformed command")
        letters = {letter for letter, _ in words}
        unknown = sorted(letters - {"G", "M", "X", "Y", "Z", "B", "C", "E", "F", "N", "S", "T", "P"})
        if unknown:
            raise GCodeError(line_no, raw, f"unknown axis/word {','.join(unknown)}")

        g_codes = [int(value) for letter, value in words if letter == "G"]
        m_codes = [int(value) for letter, value in words if letter == "M"]
        for g in g_codes:
            if g == 90:
                absolute = True
            elif g == 91:
                absolute = False
            elif g in (20, 70):
                linear_scale = 25.4
            elif g in (21, 71):
                linear_scale = 1.0
            elif g in (0, 1, 92):
                pass
            else:
                raise GCodeError(line_no, raw, f"unsupported G-code G{g}")
        for m in m_codes:
            warnings.append(f"line {line_no}: skipped non-motion M{m}: {raw.rstrip()}")

        motion_codes = [g for g in g_codes if g in (0, 1)]
        if not motion_codes:
            if 92 in g_codes:
                for axis, value in words:
                    if axis in current:
                        current[axis] = value * linear_scale if axis in "XYZ" else value
            continue
        if len(motion_codes) != 1:
            raise GCodeError(line_no, raw, "multiple motion commands in one block")

        target = dict(current)
        extrusion: float | None = None
        for letter, value in words:
            if letter in current:
                scaled = value * linear_scale if letter in "XYZ" else value
                target[letter] = scaled if absolute else current[letter] + scaled
                if letter in ("B", "C"):
                    has_rotary = True
            elif letter == "F":
                feed = value * linear_scale
            elif letter == "E":
                extrusion = value
        pose = ToolPose(target["X"], target["Y"], target["Z"], target["C"], target["B"])
        profile.validate_tool_pose(pose, line=line_no)
        moves.append(Move(line_no, raw, f"G{motion_codes[0]}", pose, feed, extrusion))
        current = target

    if not moves:
        raise GCodeError(0, "", "trajectory queue is empty: no G0/G1 moves were parsed")
    mode = "5-axis" if has_rotary else "XYZ-only compatibility mode"
    if not has_rotary:
        warnings.append("XYZ-only compatibility mode: B/C were absent and remain at their modal initial value 0 deg")
    if saw_conical_meta:
        warnings.append("CONICAL_META detected but ignored in phase 1; no B/C orientation was inferred")
    return GCodeProgram(tuple(moves), tuple(warnings), mode)


def load_gcode(path: str | Path, profile: MachineProfile) -> GCodeProgram:
    resolved = Path(path).resolve()
    try:
        source = resolved.read_text(encoding="utf-8-sig")
    except OSError as exc:
        raise GCodeError(0, str(path), f"cannot open G-code: {exc}") from exc
    return parse_gcode(source, profile)
