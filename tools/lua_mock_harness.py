"""Run the Rep5x CoppeliaSim add-on headlessly against a mock `sim` API.

The mock reproduces the subset of the CoppeliaSim regular API the add-on uses:
an object tree with CoppeliaSim Euler angles (R = Rx(a)·Ry(b)·Rz(g)), passive
prismatic/revolute joints moving along/about their local Z, parent-relative
placement, parented drawing objects (world coordinates converted into the
parent frame at insertion time) and the named-parameter self-test hooks.

It does not replace a CoppeliaSim run; it catches Lua errors, broken frames and
mis-parented beads before the scene is opened in the real simulator.

Requires: pip install lupa numpy   (matplotlib only for --render)

Examples:
    python tools/lua_mock_harness.py
    python tools/lua_mock_harness.py --gcode tests/fixtures/slicer_xyz_smoke.gcode
    python tools/lua_mock_harness.py --render evidence/scene-preview-mock_c135_b-35.png --c 135 --b -35
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

try:
    import lupa
except ImportError as exc:  # pragma: no cover
    raise SystemExit("lupa is required: pip install lupa") from exc

ROOT = Path(__file__).resolve().parents[1]
ADDON_DIR = ROOT / "coppeliasim"


def rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def euler(o) -> np.ndarray:
    return rx(o[0]) @ ry(o[1]) @ rz(o[2])


def tf(rot: np.ndarray, pos) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = pos
    return m


CONSTANT_NAMES = [
    "boolparam_headless", "stringparam_application_path", "intparam_program_full_version",
    "verbosity_scripterrors", "verbosity_scriptinfos", "verbosity_scriptwarnings", "handle_all",
    "jointmode_passive", "joint_prismatic", "joint_revolute", "primitiveshape_cuboid",
    "primitiveshape_cylinder", "primitiveshape_cone", "primitiveshape_sphere", "shapeintparam_static",
    "shapeintparam_respondable", "colorcomponent_ambient_diffuse", "colorcomponent_specular",
    "objintparam_visibility_layer",
]


class MockSim:
    def __init__(self, params: dict[str, str]) -> None:
        self.objects: dict[int, dict] = {}
        self.next_handle = 100
        self.params = params
        self.logs: list[tuple[int, str]] = []
        self.drawings: dict[int, dict] = {}
        self.t = 0.0
        self.dt = 0.05
        self.C = {name: 1000 + i for i, name in enumerate(CONSTANT_NAMES)}
        # Values from the vendored simConst.h (drawing object flags are real bit values).
        self.C.update(drawing_points=0, drawing_lines=1, drawing_spherepoints=7, drawing_itemsizes=0x80,
                      drawing_persistent=0x100000, drawing_linestrip=None)

    # -- object tree -------------------------------------------------------
    def new(self, kind: str, **kw) -> int:
        h = self.next_handle
        self.next_handle += 1
        obj = dict(kind=kind, parent=-1, pos=np.zeros(3), ori=np.zeros(3), alias="", q=0.0, jkind=None)
        obj.update(kw)
        self.objects[h] = obj
        return h

    def local_tf(self, h: int) -> np.ndarray:
        o = self.objects[h]
        return tf(euler(o["ori"]), o["pos"])

    def world(self, h: int) -> np.ndarray:
        """Transform seen by children (includes the joint's own motion)."""
        o = self.objects[h]
        parent = np.eye(4) if o["parent"] == -1 else self.world(o["parent"])
        m = parent @ self.local_tf(h)
        if o["kind"] == "joint":
            m = m @ (tf(np.eye(3), [0, 0, o["q"]]) if o["jkind"] == "prismatic" else tf(rz(o["q"]), [0, 0, 0]))
        return m

    def frame(self, h: int) -> np.ndarray:
        """Object frame reported by getObjectPosition/Matrix (no own joint motion)."""
        o = self.objects[h]
        parent = np.eye(4) if o["parent"] == -1 else self.world(o["parent"])
        return parent @ self.local_tf(h)

    def rel(self, h: int, ref: int) -> np.ndarray:
        w = self.frame(h)
        return w if ref == -1 else np.linalg.inv(self.frame(ref)) @ w

    def by_alias(self, alias: str) -> int:
        for h, o in self.objects.items():
            if o["alias"] == alias:
                return h
        raise KeyError(alias)

    # -- API table -----------------------------------------------------------
    def api(self, lua) -> dict:
        C, m = self.C, self

        def lt(values):
            return lua.table_from([float(v) for v in values])

        def getBoolParam(_p):
            return True

        def getStringParam(_p):
            return m.params["app"]

        def getInt32Param(_p):
            return 4100000

        def getNamedStringParam(name):
            return m.params.get(name)

        def addLog(verbosity, msg):
            m.logs.append((verbosity, msg))

        def createDummy(_size):
            return m.new("dummy")

        def createPrimitiveShape(kind, size, _options):
            return m.new("shape", size=[float(v) for v in size.values()], prim=kind)

        def createJoint(kind, _mode, _options, _sizes):
            return m.new("joint", jkind="prismatic" if kind == C["joint_prismatic"] else "revolute")

        def setObjectAlias(h, name):
            m.objects[h]["alias"] = name

        def getObjectAlias(h):
            return m.objects[h]["alias"]

        def setObjectParent(h, parent, _keep):
            m.objects[h]["parent"] = parent

        def setObjectPosition(h, ref, pos):
            if ref != m.objects[h]["parent"]:
                raise RuntimeError("mock supports parent-relative placement only")
            m.objects[h]["pos"] = np.array(list(pos.values()), float)

        def setObjectOrientation(h, ref, ori):
            if ref != m.objects[h]["parent"]:
                raise RuntimeError("mock supports parent-relative placement only")
            m.objects[h]["ori"] = np.array(list(ori.values()), float)

        def getObjectPosition(h, ref):
            return lt(m.rel(h, ref)[:3, 3])

        def getObjectMatrix(h, ref):
            return lt(m.rel(h, ref)[:3, :].reshape(-1))

        def setObjectInt32Param(h, param, value):
            m.objects[h].setdefault("params", {})[param] = value

        def setShapeColor(h, _name, component, color):
            m.objects[h].setdefault("color", {})[component] = [float(v) for v in color.values()]

        def setJointInterval(h, cyclic, interval):
            m.objects[h]["interval"] = (bool(cyclic), [float(v) for v in interval.values()])

        def setJointPosition(h, q):
            o = m.objects[h]
            if o["kind"] != "joint":
                raise RuntimeError("not a joint")
            cyclic, (lo, rng) = o["interval"]
            if not cyclic and not (lo - 1e-9 <= q <= lo + rng + 1e-9):
                raise RuntimeError(f"joint {o['alias']} q={q} outside [{lo},{lo + rng}]")
            o["q"] = float(q)

        def getJointPosition(h):
            return m.objects[h]["q"]

        def getObject(path):
            try:
                return m.by_alias(path.lstrip("/"))
            except KeyError:
                raise RuntimeError("object does not exist: " + path) from None

        def getObjectsInTree(root, _kind, _options):
            out: list[int] = []

            def walk(h):
                out.append(h)
                for c, o in m.objects.items():
                    if o["parent"] == h:
                        walk(c)

            walk(root)
            return lua.table_from(out)

        def setBoolProperty(h, name, value):
            m.objects[h][name] = value

        def addDrawingObject(flags, size, _tol, parent, max_items, color=None):
            h = m.next_handle
            m.next_handle += 1
            m.drawings[h] = dict(flags=int(flags), size=size, parent=parent, maxn=max_items, items=[],
                                 color=None if color is None else [float(v) for v in color.values()])
            return h

        def addDrawingObjectItem(h, item):
            d = m.drawings[h]
            if item is None:
                d["items"] = []
                return 1
            vals = [float(v) for v in item.values()]
            base = d["flags"] & 0x1F
            need = 3 if base in (0, 7) else 6
            if d["flags"] & 0x80:
                need += 1
            if len(vals) != need:
                raise RuntimeError(f"drawing item needs {need} values, got {len(vals)}")
            p = np.array(vals[:3])
            if d["parent"] != -1:
                p = (np.linalg.inv(m.frame(d["parent"])) @ np.append(p, 1.0))[:3]
            d["items"].append((p, vals[3:]))
            if len(d["items"]) > d["maxn"]:
                raise RuntimeError("drawing object overflow")
            return 1

        def removeDrawingObject(h):
            m.drawings.pop(h, None)

        def getSimulationTime():
            return m.t

        def getSimulationTimeStep():
            return m.dt

        api = dict(C)
        api.update({k: v for k, v in locals().items() if callable(v) and k not in ("lt", "m", "C", "lua")})
        return api


def run(gcode: str | None = None, dt: float = 0.05, tmax: float = 600.0) -> tuple[MockSim, dict, int]:
    app = tempfile.mkdtemp(prefix="rep5x-mock-")
    os.symlink(ADDON_DIR, os.path.join(app, "addOns"))
    out = os.path.join(app, "selftest.json")
    params = {"app": app, "rep5x.selfTest": out}
    if gcode:
        params["rep5x.gcode"] = str(Path(gcode).resolve())
    mock = MockSim(params)
    mock.dt = dt
    lua = lupa.LuaRuntime(unpack_returned_tuples=True)
    raw = lua.table_from(mock.api(lua))
    wrap = lua.eval(
        """function(raw)
            local t={}
            for k,v in pairs(raw) do
                if type(v)=='userdata' or type(v)=='function' then
                    t[k]=function(...) local ok,r=pcall(v,...) if not ok then error(tostring(r),2) end return r end
                else t[k]=v end
            end
            return t end"""
    )
    sim_table = wrap(raw)
    lua.globals()["package"]["preload"]["sim"] = lua.eval("function(t) return function() return t end end")(sim_table)
    lua.execute((ADDON_DIR / "simAddOnRep5xSceneBuilder.lua").read_text(encoding="utf-8"))
    g = lua.globals()
    g.sysCall_init()
    g.sysCall_beforeSimulation()
    steps = 0
    while mock.t < tmax:
        g.sysCall_actuation()
        g.sysCall_sensing()
        mock.t += dt
        steps += 1
        if any("trajectory complete" in msg or "ERROR" in msg for _, msg in mock.logs[-2:]):
            break
    g.sysCall_cleanup()
    with open(out, encoding="utf-8") as stream:
        result = json.load(stream)
    return mock, result, steps


def bead_items(mock: MockSim) -> list[tuple[np.ndarray, list[float]]]:
    for d in mock.drawings.values():
        if d["parent"] != -1:
            return d["items"]
    return []


def check(gcode: str | None) -> dict:
    mock, result, steps = run(gcode)
    problems = [msg for _, msg in mock.logs if "ERROR" in msg or "differs" in msg or "rejected" in msg or "visual disabled" in msg]
    if not result["ok"] or problems:
        raise SystemExit(f"FAILED: {result['message']} {problems}")
    final, bed = result["final_pose_mm_deg"], result["tcp_relative_bed_mm"]
    for axis, value in zip("XYZ", bed):
        if abs(final[axis] - value) > 1e-6:
            raise SystemExit(f"tip relative to bed {value} differs from commanded {axis}={final[axis]}")
    aliases = [o["alias"] for o in mock.objects.values()]
    duplicates = sorted({a for a in aliases if a and aliases.count(a) > 1})
    if duplicates:
        raise SystemExit(f"duplicate aliases: {duplicates}")
    beads = bead_items(mock)
    summary = {
        "gcode": gcode or "built-in test trajectory",
        "ok": result["ok"],
        "moves_completed": result["moves_completed"],
        "simulation_steps": steps,
        "final_pose_mm_deg": final,
        "tcp_relative_bed_mm": bed,
        "scene_objects": len(mock.objects),
        "shapes": sum(o["kind"] == "shape" for o in mock.objects.values()),
        "bead_samples": len(beads),
        "filament_mm": result["extrusion"]["filament_mm"],
    }
    if beads:
        pts = np.array([p for p, _ in beads]) * 1000.0
        sizes = [s[0] * 1000.0 for _, s in beads if s]
        summary["bead_bed_mm_min"] = pts.min(0).round(6).tolist()
        summary["bead_bed_mm_max"] = pts.max(0).round(6).tolist()
        if sizes:
            summary["bead_width_mm_range"] = [round(min(sizes), 3), round(max(sizes), 3)]
    return summary


def render(out: Path, c_deg: float, b_deg: float) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    mock, _, _ = run(tmax=0.0)
    C = mock.C
    poses = {"Rep5x_X_joint": 0.10, "Rep5x_Y_bed_joint": -0.10, "Rep5x_Z_joint": 0.03,
             "Rep5x_C_joint": math.radians(c_deg), "Rep5x_B_joint": math.radians(b_deg)}
    for alias, q in poses.items():
        mock.objects[mock.by_alias(alias)]["q"] = q

    def faces(o):
        kind = o["prim"]
        sx, sy, sz = o["size"]
        if kind == C["primitiveshape_cuboid"]:
            x, y, z = sx / 2, sy / 2, sz / 2
            v = np.array([[-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z], [-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z]])
            return [v[i] for i in ([0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [2, 3, 7, 6], [1, 2, 6, 5], [0, 3, 7, 4])]
        r = sx / 2
        if kind == C["primitiveshape_sphere"]:
            n = 8
            def pt(th, ph):
                return np.array([r * math.sin(th) * math.cos(ph), r * math.sin(th) * math.sin(ph), r * math.cos(th)])
            return [np.array([pt(math.pi * i / n, 2 * math.pi * j / n), pt(math.pi * (i + 1) / n, 2 * math.pi * j / n),
                              pt(math.pi * (i + 1) / n, 2 * math.pi * (j + 1) / n), pt(math.pi * i / n, 2 * math.pi * (j + 1) / n)])
                    for i in range(n) for j in range(n)]
        n = 16
        top = 0.0 if kind == C["primitiveshape_cone"] else r
        ring = lambda rr, z: [np.array([rr * math.cos(2 * math.pi * i / n), rr * math.sin(2 * math.pi * i / n), z]) for i in range(n)]
        bottom, upper = ring(r, -sz / 2), ring(top, sz / 2)
        out_faces = [np.array([bottom[i], bottom[(i + 1) % n], upper[(i + 1) % n], upper[i]]) for i in range(n)]
        out_faces.append(np.array(bottom))
        if top > 0:
            out_faces.append(np.array(upper))
        return out_faces

    fig = plt.figure(figsize=(14, 9))
    for k, (el, az) in enumerate([(22, -55), (18, -140)]):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d")
        polys, cols = [], []
        for h, o in mock.objects.items():
            if o["kind"] != "shape" or o.get("params", {}).get(C["objintparam_visibility_layer"], 1) == 0:
                continue
            w = mock.frame(h)
            col = o.get("color", {}).get(C["colorcomponent_ambient_diffuse"], [0.5, 0.5, 0.5])
            for f in faces(o):
                polys.append((w @ np.c_[f, np.ones(len(f))].T).T[:, :3])
                cols.append(col)
        ax.add_collection3d(Poly3DCollection(polys, facecolors=cols, edgecolors="none"))
        ax.set_xlim(-0.15, 0.40)
        ax.set_ylim(-0.30, 0.25)
        ax.set_zlim(-0.12, 0.60)
        ax.set_box_aspect((0.55, 0.55, 0.72))
        ax.view_init(el, az)
        ax.set_axis_off()
        ax.set_title(f"Rep5x / Ender-3 V3 SE proxy, mock render (C={c_deg:g} deg, B={b_deg:g} deg)")
    plt.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out, dpi=110)
    print(f"wrote {out}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--gcode", help="G-code file to replay instead of the built-in test trajectory")
    parser.add_argument("--json", help="write the check summary to this file")
    parser.add_argument("--render", type=Path, help="write a PNG preview of the proxy scene (needs matplotlib)")
    parser.add_argument("--c", type=float, default=0.0, help="C angle (deg) for --render")
    parser.add_argument("--b", type=float, default=0.0, help="B angle (deg) for --render")
    args = parser.parse_args(argv)
    if args.render:
        render(args.render, args.c, args.b)
        return 0
    summary = check(args.gcode)
    text = json.dumps(summary, indent=2)
    print(text)
    if args.json:
        Path(args.json).write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
