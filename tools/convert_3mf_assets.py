from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import zipfile


UNIT_TO_MM = {
    "micron": 0.001,
    "millimeter": 1.0,
    "centimeter": 10.0,
    "inch": 25.4,
    "foot": 304.8,
    "meter": 1000.0,
}
IDENTITY = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)


class ConversionError(RuntimeError):
    pass


def parse_transform(value: str | None) -> tuple[float, ...]:
    if value is None:
        return IDENTITY
    try:
        result = tuple(float(x) for x in value.split())
    except ValueError as exc:
        raise ConversionError(f"invalid 3MF transform {value!r}") from exc
    if len(result) != 12 or not all(math.isfinite(x) for x in result):
        raise ConversionError(f"3MF transform must contain 12 finite numbers: {value!r}")
    return result


def apply_transform(point: tuple[float, float, float], m: tuple[float, ...]) -> tuple[float, float, float]:
    # 3MF specifies row-vector affine transforms with translation in m[9:12].
    x, y, z = point
    return (
        x * m[0] + y * m[3] + z * m[6] + m[9],
        x * m[1] + y * m[4] + z * m[7] + m[10],
        x * m[2] + y * m[5] + z * m[8] + m[11],
    )


def compose(first: tuple[float, ...], second: tuple[float, ...]) -> tuple[float, ...]:
    """Return the transform that applies first and then second."""
    basis = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
    origin = apply_transform(apply_transform((0.0, 0.0, 0.0), first), second)
    columns = []
    for point in basis:
        end = apply_transform(apply_transform(point, first), second)
        columns.append(tuple(end[i] - origin[i] for i in range(3)))
    return (
        columns[0][0], columns[0][1], columns[0][2],
        columns[1][0], columns[1][1], columns[1][2],
        columns[2][0], columns[2][1], columns[2][2],
        origin[0], origin[1], origin[2],
    )


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def read_model(path: Path) -> tuple[list[tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]], dict]:
    try:
        with zipfile.ZipFile(path) as archive:
            model_names = [name for name in archive.namelist() if name.lower().endswith(".model") and name.lower().startswith("3d/")]
            if len(model_names) != 1:
                raise ConversionError(f"expected exactly one 3D/*.model part, found {model_names}")
            root = ET.fromstring(archive.read(model_names[0]))
    except (OSError, zipfile.BadZipFile, ET.ParseError, KeyError) as exc:
        raise ConversionError(f"cannot read 3MF {path}: {exc}") from exc

    unit = root.attrib.get("unit")
    if unit not in UNIT_TO_MM:
        raise ConversionError(f"3MF unit/transform verification failed: unsupported or missing unit {unit!r}")
    scale = UNIT_TO_MM[unit]
    objects: dict[str, ET.Element] = {}
    build: ET.Element | None = None
    for element in root.iter():
        name = _local_name(element.tag)
        if name == "object" and "id" in element.attrib:
            objects[element.attrib["id"]] = element
        elif name == "build":
            build = element
    if build is None:
        raise ConversionError("3MF unit/transform verification failed: missing build section")

    triangles = []
    component_count = 0
    transform_count = 0

    def emit_object(object_id: str, transform: tuple[float, ...], stack: tuple[str, ...]) -> None:
        nonlocal component_count, transform_count
        if object_id in stack:
            raise ConversionError(f"cyclic 3MF component reference: {' -> '.join(stack + (object_id,))}")
        obj = objects.get(object_id)
        if obj is None:
            raise ConversionError(f"3MF component references unknown object id {object_id}")
        meshes = [e for e in obj if _local_name(e.tag) == "mesh"]
        components = [e for e in obj if _local_name(e.tag) == "components"]
        if len(meshes) + len(components) != 1:
            raise ConversionError(f"object {object_id} must contain exactly one mesh or components element")
        if meshes:
            mesh = meshes[0]
            vertices_element = next((e for e in mesh if _local_name(e.tag) == "vertices"), None)
            triangles_element = next((e for e in mesh if _local_name(e.tag) == "triangles"), None)
            if vertices_element is None or triangles_element is None:
                raise ConversionError(f"object {object_id} mesh is missing vertices/triangles")
            vertices = []
            for vertex in vertices_element:
                try:
                    p = (float(vertex.attrib["x"]), float(vertex.attrib["y"]), float(vertex.attrib["z"]))
                except (KeyError, ValueError) as exc:
                    raise ConversionError(f"object {object_id} has invalid vertex") from exc
                world = apply_transform(p, transform)
                vertices.append(tuple(value * scale for value in world))
            for triangle in triangles_element:
                try:
                    indices = [int(triangle.attrib[key]) for key in ("v1", "v2", "v3")]
                    triangles.append(tuple(vertices[index] for index in indices))
                except (KeyError, ValueError, IndexError) as exc:
                    raise ConversionError(f"object {object_id} has invalid triangle") from exc
        else:
            for component in components[0]:
                component_count += 1
                local = parse_transform(component.attrib.get("transform"))
                if local != IDENTITY:
                    transform_count += 1
                emit_object(component.attrib.get("objectid", ""), compose(local, transform), stack + (object_id,))

    build_items = [e for e in build if _local_name(e.tag) == "item"]
    if not build_items:
        raise ConversionError("3MF unit/transform verification failed: build contains no items")
    for item in build_items:
        item_transform = parse_transform(item.attrib.get("transform"))
        if item_transform != IDENTITY:
            transform_count += 1
        emit_object(item.attrib.get("objectid", ""), item_transform, ())
    if not triangles:
        raise ConversionError("3MF contains no triangles after resolving build/components")
    if not all(math.isfinite(value) for tri in triangles for point in tri for value in point):
        raise ConversionError("3MF produced NaN/Infinity after applying transforms")

    xs = [p[0] for tri in triangles for p in tri]
    ys = [p[1] for tri in triangles for p in tri]
    zs = [p[2] for tri in triangles for p in tri]
    metadata = {
        "unit": unit,
        "unit_scale_to_mm": scale,
        "build_items": len(build_items),
        "component_references": component_count,
        "non_identity_transforms": transform_count,
        "triangles": len(triangles),
        "bbox_mm": {"min": [min(xs), min(ys), min(zs)], "max": [max(xs), max(ys), max(zs)]},
    }
    return triangles, metadata


def _normal(triangle):
    a, b, c = triangle
    u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    v = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
    length = math.sqrt(sum(x * x for x in n))
    return (0.0, 0.0, 0.0) if length == 0 else tuple(x / length for x in n)


def write_binary_stl(path: Path, triangles, source_name: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ("FIBR3DEmul Rep5x converted: " + source_name).encode("ascii", "replace")[:80].ljust(80, b"\0")
    with path.open("wb") as stream:
        stream.write(header)
        stream.write(struct.pack("<I", len(triangles)))
        for triangle in triangles:
            stream.write(struct.pack("<12fH", *(_normal(triangle) + triangle[0] + triangle[1] + triangle[2]), 0))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def convert(source: Path, destination: Path) -> dict:
    triangles, metadata = read_model(source)
    write_binary_stl(destination, triangles, source.name)
    return {
        "source": source.as_posix(),
        "source_sha256": sha256(source),
        "output": destination.as_posix(),
        "output_sha256": sha256(destination),
        **metadata,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert Rep5x 3MF assets to binary STL while preserving 3MF transforms")
    parser.add_argument("--rep5x-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args()
    folders = (
        args.rep5x_root / "build-guide/universal-parts/3d-printed-parts/current/3mf",
        args.rep5x_root / "build-guide/printer-specific/ender-3-v3-se/3d-printed-parts/current/3mf",
    )
    sources = sorted(path for folder in folders for path in folder.glob("*.3mf"))
    if len(sources) != 10:
        raise ConversionError(f"expected 10 audited Rep5x 3MF files, found {len(sources)}")
    report = []
    for source in sources:
        destination = args.output / (source.stem + ".stl")
        item = convert(source.resolve(), destination.resolve())
        try:
            item["source"] = source.resolve().relative_to(args.rep5x_root.resolve()).as_posix()
            item["output"] = destination.resolve().relative_to(Path.cwd().resolve()).as_posix()
        except ValueError:
            pass
        report.append(item)
        print(f"{source.name}: {item['triangles']} triangles, unit={item['unit']}, transforms={item['non_identity_transforms']}")
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps({"converter": "tools/convert_3mf_assets.py", "assets": report}, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
