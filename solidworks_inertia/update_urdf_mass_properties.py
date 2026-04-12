#!/usr/bin/env python3
"""Update URDF inertial properties from SolidWorks mass-property text exports.

The script matches `mass/<link_name>.txt` to `<link name="...">` in the URDF.
By default it writes a sibling `*.updated.urdf` file instead of overwriting the
source URDF. Use `--inplace` only when you are ready to replace the original.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET


MM_TO_M = 1e-3
MM2_TO_M2 = 1e-6


NUMBER_RE = r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?"


@dataclass
class MassProperties:
    link_name: str
    mass_kg: float
    com_m: tuple[float, float, float]
    principal_moments_kg_m2: tuple[float, float, float]
    principal_axes: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    aligned_tensor_kg_m2: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    rpy_rad: tuple[float, float, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Use SolidWorks mass-property TXT files to update URDF inertial blocks.",
    )
    parser.add_argument(
        "urdf",
        type=Path,
        help="URDF file to read.",
    )
    parser.add_argument(
        "--mass-dir",
        type=Path,
        default=Path("mass"),
        help="Directory containing SolidWorks TXT exports. Default: ./mass",
    )
    parser.add_argument(
        "--mode",
        choices=("principal", "aligned"),
        default="principal",
        help=(
            "How inertial data is written. "
            "'principal' keeps principal-axis rotation and diagonal inertia; "
            "'aligned' writes the full inertia tensor in the link frame with rpy=0."
        ),
    )
    parser.add_argument(
        "--links",
        nargs="*",
        help="Optional subset of link names to update. Defaults to every TXT file in --mass-dir.",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Fail if a TXT file has no matching link or if a requested link is missing.",
    )
    output_group = parser.add_mutually_exclusive_group()
    output_group.add_argument(
        "--output",
        type=Path,
        help="Write the updated URDF to this path.",
    )
    output_group.add_argument(
        "--inplace",
        action="store_true",
        help="Overwrite the input URDF file.",
    )
    return parser.parse_args()


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def require_match(pattern: str, text: str, label: str) -> re.Match[str]:
    match = re.search(pattern, text, re.MULTILINE | re.DOTALL)
    if match is None:
        raise ValueError(f"Could not parse {label}.")
    return match


def parse_mass_properties(txt_path: Path) -> MassProperties:
    text = read_text(txt_path)

    mass_match = require_match(rf"质量\s*=\s*({NUMBER_RE})\s*千克", text, f"mass from {txt_path}")
    mass_kg = float(mass_match.group(1))

    com_match = require_match(
        rf"重心\s*:\s*\(\s*毫米\s*\)\s*.*?X\s*=\s*({NUMBER_RE})\s*.*?Y\s*=\s*({NUMBER_RE})\s*.*?Z\s*=\s*({NUMBER_RE})",
        text,
        f"center of mass from {txt_path}",
    )
    com_m = tuple(float(com_match.group(i)) * MM_TO_M for i in range(1, 4))

    axes: dict[str, tuple[float, float, float]] = {}
    moments_mm2: dict[str, float] = {}
    for axis, x_val, y_val, z_val, moment_axis, moment_val in re.findall(
        rf"I([xyz])\s*=\s*\(\s*({NUMBER_RE})\s*,\s*({NUMBER_RE})\s*,\s*({NUMBER_RE})\s*\)\s*P([xyz])\s*=\s*({NUMBER_RE})",
        text,
        re.IGNORECASE,
    ):
        axis_key = axis.lower()
        moment_key = moment_axis.lower()
        axes[axis_key] = (float(x_val), float(y_val), float(z_val))
        moments_mm2[moment_key] = float(moment_val)

    if set(axes) != {"x", "y", "z"} or set(moments_mm2) != {"x", "y", "z"}:
        raise ValueError(f"Could not parse principal axes and moments from {txt_path}.")

    tensor_terms = {
        name: float(value)
        for name, value in re.findall(
            rf"\b(Lxx|Lxy|Lxz|Lyx|Lyy|Lyz|Lzx|Lzy|Lzz)\s*=\s*({NUMBER_RE})",
            text,
        )
    }
    required_tensor_terms = {"Lxx", "Lxy", "Lxz", "Lyx", "Lyy", "Lyz", "Lzx", "Lzy", "Lzz"}
    if set(tensor_terms) != required_tensor_terms:
        raise ValueError(f"Could not parse inertia tensor aligned to output frame from {txt_path}.")

    principal_axes = (axes["x"], axes["y"], axes["z"])
    principal_moments_kg_m2 = tuple(moments_mm2[key] * MM2_TO_M2 for key in ("x", "y", "z"))
    aligned_tensor_kg_m2 = (
        (
            tensor_terms["Lxx"] * MM2_TO_M2,
            tensor_terms["Lxy"] * MM2_TO_M2,
            tensor_terms["Lxz"] * MM2_TO_M2,
        ),
        (
            tensor_terms["Lyx"] * MM2_TO_M2,
            tensor_terms["Lyy"] * MM2_TO_M2,
            tensor_terms["Lyz"] * MM2_TO_M2,
        ),
        (
            tensor_terms["Lzx"] * MM2_TO_M2,
            tensor_terms["Lzy"] * MM2_TO_M2,
            tensor_terms["Lzz"] * MM2_TO_M2,
        ),
    )

    rotation = orthonormalize_principal_axes(principal_axes)
    validate_principal_axes(txt_path, rotation, principal_moments_kg_m2, aligned_tensor_kg_m2)
    rpy_rad = rotation_matrix_to_rpy(rotation)

    return MassProperties(
        link_name=txt_path.stem,
        mass_kg=mass_kg,
        com_m=com_m,
        principal_moments_kg_m2=principal_moments_kg_m2,
        principal_axes=principal_axes,
        aligned_tensor_kg_m2=aligned_tensor_kg_m2,
        rpy_rad=rpy_rad,
    )


def dot(a: Iterable[float], b: Iterable[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def norm(vector: Iterable[float]) -> float:
    return math.sqrt(dot(vector, vector))


def normalize(vector: Iterable[float]) -> tuple[float, float, float]:
    values = tuple(vector)
    magnitude = norm(values)
    if magnitude <= 0.0:
        raise ValueError("Encountered a zero-length vector while normalizing principal axes.")
    return tuple(component / magnitude for component in values)


def subtract(a: Iterable[float], b: Iterable[float]) -> tuple[float, float, float]:
    return tuple(x - y for x, y in zip(a, b))


def scale(vector: Iterable[float], scalar: float) -> tuple[float, float, float]:
    return tuple(component * scalar for component in vector)


def cross(a: Iterable[float], b: Iterable[float]) -> tuple[float, float, float]:
    ax, ay, az = a
    bx, by, bz = b
    return (
        ay * bz - az * by,
        az * bx - ax * bz,
        ax * by - ay * bx,
    )


def orthonormalize_principal_axes(
    principal_axes: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    x_axis = normalize(principal_axes[0])
    y_guess = principal_axes[1]
    y_axis = subtract(y_guess, scale(x_axis, dot(y_guess, x_axis)))
    y_axis = normalize(y_axis)
    z_axis = normalize(cross(x_axis, y_axis))
    z_guess = normalize(principal_axes[2])

    if dot(z_axis, z_guess) < 0.0:
        y_axis = scale(y_axis, -1.0)
        z_axis = scale(z_axis, -1.0)

    y_axis = cross(z_axis, x_axis)
    return (x_axis, y_axis, z_axis)


def matmul_3x3(a: tuple[tuple[float, float, float], ...], b: tuple[tuple[float, float, float], ...]) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )


def transpose_3x3(matrix: tuple[tuple[float, float, float], ...]) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(matrix[row][col] for row in range(3)) for col in range(3))


def principal_rotation_matrix(
    principal_axes: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    x_axis, y_axis, z_axis = principal_axes
    return (
        (x_axis[0], y_axis[0], z_axis[0]),
        (x_axis[1], y_axis[1], z_axis[1]),
        (x_axis[2], y_axis[2], z_axis[2]),
    )


def reconstruct_aligned_tensor(
    rotation: tuple[tuple[float, float, float], ...],
    principal_moments_kg_m2: tuple[float, float, float],
) -> tuple[tuple[float, float, float], ...]:
    diagonal = (
        (principal_moments_kg_m2[0], 0.0, 0.0),
        (0.0, principal_moments_kg_m2[1], 0.0),
        (0.0, 0.0, principal_moments_kg_m2[2]),
    )
    return matmul_3x3(matmul_3x3(rotation, diagonal), transpose_3x3(rotation))


def validate_principal_axes(
    txt_path: Path,
    principal_axes: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    principal_moments_kg_m2: tuple[float, float, float],
    aligned_tensor_kg_m2: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
) -> None:
    rotation = principal_rotation_matrix(principal_axes)
    reconstructed = reconstruct_aligned_tensor(rotation, principal_moments_kg_m2)
    max_error = max(
        abs(reconstructed[row][col] - aligned_tensor_kg_m2[row][col])
        for row in range(3)
        for col in range(3)
    )
    if max_error > 5e-8:
        raise ValueError(
            f"Principal axes in {txt_path} do not reconstruct the reported inertia tensor "
            f"(max error {max_error:.3e} kg*m^2)."
        )


def rotation_matrix_to_rpy(rotation: tuple[tuple[float, float, float], ...]) -> tuple[float, float, float]:
    r20 = rotation[2][0]
    if abs(r20) < 1.0 - 1e-12:
        pitch = math.asin(-r20)
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        pitch = math.copysign(math.pi / 2.0, -r20)
        roll = math.atan2(-rotation[0][1], rotation[1][1])
        yaw = 0.0
    return (roll, pitch, yaw)


def ensure_child(parent: ET.Element, tag: str) -> ET.Element:
    child = parent.find(tag)
    if child is None:
        child = ET.SubElement(parent, tag)
    return child


def format_scalar(value: float, decimals: int = 9) -> str:
    if abs(value) < 5e-13:
        return "0"
    text = f"{value:.{decimals}f}".rstrip("0").rstrip(".")
    if text in {"-0", "-0.0"}:
        return "0"
    return text


def format_inertia(value: float) -> str:
    if abs(value) < 5e-13:
        return "0"
    return f"{value:.8e}"


def set_inertial_from_properties(link: ET.Element, props: MassProperties, mode: str) -> None:
    inertial = ensure_child(link, "inertial")
    origin = ensure_child(inertial, "origin")
    mass = ensure_child(inertial, "mass")
    inertia = ensure_child(inertial, "inertia")

    origin.set("xyz", " ".join(format_scalar(value, decimals=9) for value in props.com_m))
    if mode == "principal":
        origin.set("rpy", " ".join(format_scalar(value, decimals=9) for value in props.rpy_rad))
        inertia_matrix = (
            (props.principal_moments_kg_m2[0], 0.0, 0.0),
            (0.0, props.principal_moments_kg_m2[1], 0.0),
            (0.0, 0.0, props.principal_moments_kg_m2[2]),
        )
    else:
        origin.set("rpy", "0 0 0")
        inertia_matrix = props.aligned_tensor_kg_m2

    mass.set("value", format_scalar(props.mass_kg, decimals=9))
    inertia.set("ixx", format_inertia(inertia_matrix[0][0]))
    inertia.set("ixy", format_inertia(inertia_matrix[0][1]))
    inertia.set("ixz", format_inertia(inertia_matrix[0][2]))
    inertia.set("iyy", format_inertia(inertia_matrix[1][1]))
    inertia.set("iyz", format_inertia(inertia_matrix[1][2]))
    inertia.set("izz", format_inertia(inertia_matrix[2][2]))


def default_output_path(urdf_path: Path) -> Path:
    return urdf_path.with_name(f"{urdf_path.stem}.updated{urdf_path.suffix}")


def load_properties(mass_dir: Path, requested_links: set[str] | None) -> dict[str, MassProperties]:
    if not mass_dir.is_dir():
        raise FileNotFoundError(f"Mass-property directory does not exist: {mass_dir}")

    txt_files = sorted(mass_dir.glob("*.txt"))
    if requested_links is not None:
        txt_files = [path for path in txt_files if path.stem in requested_links]

    if not txt_files:
        requested_note = "" if requested_links is None else f" for links {sorted(requested_links)}"
        raise FileNotFoundError(f"No SolidWorks TXT files were found in {mass_dir}{requested_note}.")

    return {path.stem: parse_mass_properties(path) for path in txt_files}


def update_urdf(
    urdf_path: Path,
    properties_by_link: dict[str, MassProperties],
    mode: str,
    strict: bool,
) -> tuple[ET.ElementTree, list[str], list[str]]:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    updated_links: list[str] = []

    for link in root.findall("link"):
        link_name = link.get("name")
        if link_name is None:
            continue
        props = properties_by_link.get(link_name)
        if props is None:
            continue
        set_inertial_from_properties(link, props, mode)
        updated_links.append(link_name)

    missing_in_urdf = sorted(set(properties_by_link) - set(updated_links))
    if strict and missing_in_urdf:
        raise ValueError(f"TXT files without matching URDF links: {', '.join(missing_in_urdf)}")

    return tree, updated_links, missing_in_urdf


def indent_tree(tree: ET.ElementTree) -> None:
    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ")


def main() -> int:
    args = parse_args()

    urdf_path = args.urdf.resolve()
    mass_dir = args.mass_dir.resolve()
    requested_links = set(args.links) if args.links else None

    try:
        properties_by_link = load_properties(mass_dir, requested_links)
        if requested_links is not None:
            missing_txt = sorted(requested_links - set(properties_by_link))
            if missing_txt and args.strict:
                raise FileNotFoundError(f"No TXT file found for requested links: {', '.join(missing_txt)}")

        tree, updated_links, missing_in_urdf = update_urdf(
            urdf_path=urdf_path,
            properties_by_link=properties_by_link,
            mode=args.mode,
            strict=args.strict,
        )
    except (ET.ParseError, FileNotFoundError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    output_path = urdf_path if args.inplace else (args.output.resolve() if args.output else default_output_path(urdf_path))
    output_path.parent.mkdir(parents=True, exist_ok=True)

    indent_tree(tree)
    tree.write(output_path, encoding="utf-8", xml_declaration=True, short_empty_elements=True)

    print(f"Updated {len(updated_links)} link(s) in {output_path}")
    if updated_links:
        print("Links:", ", ".join(updated_links))
    if missing_in_urdf:
        print("TXT files without matching URDF link:", ", ".join(missing_in_urdf), file=sys.stderr)

    if requested_links is not None:
        missing_txt = sorted(requested_links - set(properties_by_link))
        if missing_txt:
            print("Requested links without TXT file:", ", ".join(missing_txt), file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
