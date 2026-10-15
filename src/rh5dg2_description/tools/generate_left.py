#!/usr/bin/env python3
"""Generate a geometric left reference by reflecting the vendor right model.

Only the Python standard library is required. All local frames are transformed
with S=diag(1,-1,1), so the assembled hand is reflected about the base XZ plane.
This is a derived reference, not manufacturer left-hand data or calibration.
Unknown source structure is deliberately rejected instead of silently copied.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import sys
import xml.etree.ElementTree as ET


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SOURCE = Path("urdf/rh5dg2_right.xacro")
TARGET = Path("urdf/rh5dg2_left.xacro")
MESH_DIR = Path("meshes/left_mirrored")
MESH_PREFIX = "package://rh5dg2_description/meshes/"
RECORD = struct.Struct("<12fH")

# tag: (required attributes, optional attributes, allowed child tags)
SCHEMA = {
    "robot": ({"name"}, set(), {"link", "joint"}),
    "link": ({"name"}, set(), {"inertial", "visual", "collision"}),
    "joint": ({"name", "type"}, set(),
              {"origin", "parent", "child", "axis", "limit", "mimic"}),
    "inertial": (set(), set(), {"origin", "mass", "inertia"}),
    "visual": (set(), set(), {"origin", "geometry", "material"}),
    "collision": (set(), set(), {"origin", "geometry", "surface"}),
    "origin": ({"xyz", "rpy"}, set(), set()),
    "mass": ({"value"}, set(), set()),
    "inertia": ({"ixx", "ixy", "ixz", "iyy", "iyz", "izz"}, set(), set()),
    "geometry": (set(), set(), {"mesh"}),
    "mesh": ({"filename"}, set(), set()),
    "material": ({"name"}, set(), {"color"}),
    "color": ({"rgba"}, set(), set()),
    "parent": ({"link"}, set(), set()),
    "child": ({"link"}, set(), set()),
    "axis": ({"xyz"}, set(), set()),
    "limit": ({"lower", "upper", "effort", "velocity"}, set(), set()),
    "mimic": ({"joint", "multiplier", "offset"}, set(), set()),
    "surface": (set(), set(), {"friction"}),
    "friction": (set(), set(), {"ode"}),
    "ode": (set(), set(), {"mu", "mu2"}),
    "mu": (set(), set(), set()),
    "mu2": (set(), set(), set()),
}


def finite_number(value):
    try:
        number = float(value)
    except ValueError as exc:
        raise ValueError(f"Expected a literal number, got {value!r}") from exc
    if not math.isfinite(number):
        raise ValueError(f"Non-finite number: {value!r}")
    return number


def negate(value):
    """Flip a decimal sign without rounding the manufacturer's precision."""
    if finite_number(value) == 0:
        return "0"
    if value.startswith("-"):
        return value[1:]
    return "-" + value.lstrip("+")


def reflect_vector(value, signs):
    components = value.split()
    if len(components) != len(signs):
        raise ValueError(f"Expected {len(signs)} components, got {value!r}")
    for component in components:
        finite_number(component)
    return " ".join(negate(component) if sign < 0 else component
                    for component, sign in zip(components, signs))


def left_name(value):
    if not value.startswith("right_"):
        raise ValueError(f"Expected a right_ name, got {value!r}")
    return "left_" + value[len("right_"):]


def validate_tree(root):
    if root.tag != "robot":
        raise ValueError("Expected a plain URDF robot root")
    for element in root.iter():
        if element.tag is ET.Comment:
            continue
        if element.tag not in SCHEMA:
            raise ValueError(f"Unsupported element {element.tag!r}")
        required, optional, children = SCHEMA[element.tag]
        attributes = set(element.attrib)
        if not required <= attributes or attributes - required - optional:
            raise ValueError(f"Unsupported attributes on {element.tag}: {attributes}")
        if any(child.tag is not ET.Comment and child.tag not in children
               for child in element):
            raise ValueError(f"Unsupported child of {element.tag}")
        if element.text and element.text.strip():
            if element.tag not in {"mu", "mu2"}:
                raise ValueError(f"Unexpected text in {element.tag}")
            finite_number(element.text)
        if element.tail and element.tail.strip():
            raise ValueError(f"Unexpected trailing text after {element.tag}")
        if element.tag in {"mass", "inertia", "limit"}:
            for value in element.attrib.values():
                finite_number(value)
        if element.tag == "mimic":
            finite_number(element.get("multiplier"))
            finite_number(element.get("offset"))
        if element.tag == "color":
            reflect_vector(element.get("rgba"), (1, 1, 1, 1))
    links = [element.get("name") for element in root.findall("link")]
    joints = [element.get("name") for element in root.findall("joint")]
    if len(links) != len(set(links)) or len(joints) != len(set(joints)):
        raise ValueError("Duplicate link or joint names")
    for joint in root.findall("joint"):
        if joint.get("type") not in {"fixed", "revolute", "continuous", "prismatic"}:
            raise ValueError(f"Unsupported joint type: {joint.get('type')}")
        for tag in ("parent", "child"):
            references = joint.findall(tag)
            if len(references) != 1 or references[0].get("link") not in links:
                raise ValueError(f"Invalid {tag} of joint {joint.get('name')}")
        if joint.get("type") != "fixed" and len(joint.findall("axis")) != 1:
            raise ValueError("Movable joints require exactly one explicit axis")
        for mimic in joint.findall("mimic"):
            if mimic.get("joint") not in joints:
                raise ValueError(f"Unknown mimic joint {mimic.get('joint')}")


def mirror_stl(source_bytes):
    if len(source_bytes) < 84:
        raise ValueError("STL is shorter than its binary header")
    count = struct.unpack_from("<I", source_bytes, 80)[0]
    if len(source_bytes) != 84 + RECORD.size * count:
        raise ValueError("Only exact-length binary STL files are supported")
    mirrored = bytearray(source_bytes)
    for offset in range(84, len(source_bytes), RECORD.size):
        values = list(RECORD.unpack_from(source_bytes, offset))
        if not all(math.isfinite(value) for value in values[:12]):
            raise ValueError("STL has a non-finite normal or vertex")
        for index in (1, 4, 7, 10):
            values[index] = -values[index]
        # Reflection reverses orientation: exchange vertices 1 and 2, leaving
        # vertex 0 fixed. Stored normals then agree with the new winding.
        values[6:9], values[9:12] = values[9:12], values[6:9]
        RECORD.pack_into(mirrored, offset, *values)
    return bytes(mirrored), count


def generate(package_dir):
    source_bytes = (package_dir / SOURCE).read_bytes()
    source_text = source_bytes.decode("utf-8")
    if "<!DOCTYPE" in source_text or "<!ENTITY" in source_text:
        raise ValueError("DTD/entity declarations are not supported")
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.fromstring(source_text, parser=parser)
    validate_tree(root)
    if root.get("name").count("_R_") != 1:
        raise ValueError("Expected exactly one _R_ in the source robot name")
    root.set("name", root.get("name").replace("_R_", "_L_"))
    # The source has this unused namespace; retain it for xacro consumers.
    root.set("xmlns:xacro", "http://www.ros.org/wiki/xacro")
    outputs = {}
    manifest_meshes = []
    mesh_paths = {}
    for element in root.iter():
        if element.tag in {"link", "joint"}:
            element.set("name", left_name(element.get("name")))
        elif element.tag in {"parent", "child"}:
            element.set("link", left_name(element.get("link")))
        elif element.tag == "mimic":
            element.set("joint", left_name(element.get("joint")))
        elif element.tag == "origin":
            element.set("xyz", reflect_vector(element.get("xyz"), (1, -1, 1)))
            element.set("rpy", reflect_vector(element.get("rpy"), (-1, 1, -1)))
        elif element.tag == "inertia":
            for key in ("ixy", "iyz"):
                element.set(key, negate(element.get(key)))
        elif element.tag == "mesh":
            filename = element.get("filename")
            if not filename.startswith(MESH_PREFIX):
                raise ValueError(f"Unexpected mesh URI: {filename}")
            basename = filename[len(MESH_PREFIX):]
            if not re.fullmatch(r"right_[A-Za-z0-9_]+\.STL", basename):
                raise ValueError(f"Unsupported mesh filename: {basename}")
            source_path = Path("meshes") / basename
            output_path = MESH_DIR / left_name(basename)
            mesh_paths[source_path] = output_path
            element.set("filename", "package://rh5dg2_description/" + output_path.as_posix())
    for joint in root.findall("joint"):
        # Rotation axes are axial vectors: det(S)*S*a; translation axes are
        # polar vectors S*a. Keeping q and limits unchanged mirrors all poses.
        signs = (1, -1, 1) if joint.get("type") == "prismatic" else (-1, 1, -1)
        for axis in joint.findall("axis"):
            axis.set("xyz", reflect_vector(axis.get("xyz"), signs))
    for source_path, output_path in sorted(mesh_paths.items()):
        mesh_bytes = (package_dir / source_path).read_bytes()
        mirrored_bytes, triangle_count = mirror_stl(mesh_bytes)
        outputs[output_path] = mirrored_bytes
        manifest_meshes.append({
            "source": source_path.as_posix(), "output": output_path.as_posix(),
            "triangles": triangle_count,
            "source_sha256": hashlib.sha256(mesh_bytes).hexdigest(),
            "output_sha256": hashlib.sha256(mirrored_bytes).hexdigest(),
        })
    prefix = source_text[:source_text.index("<robot")]
    prefix = re.sub(r"^\s*<\?xml[^>]*\?>\s*", "", prefix, count=1)
    derived_comment = (
        "<!-- GENERATED by tools/generate_left.py from rh5dg2_right.xacro.\n"
        "     Geometric left reference: S=diag(1,-1,1), base/flange XZ plane.\n"
        "     Not manufacturer left-hand data; not calibrated on hardware.\n"
        "     Edit the source or generator, then regenerate this file. -->\n")
    ET.indent(root, space="  ")
    xml_text = ('<?xml version="1.0" encoding="utf-8"?>\n' + derived_comment +
                prefix + ET.tostring(root, encoding="unicode") + "\n")
    outputs[TARGET] = xml_text.encode("utf-8")
    manifest = {
        "generator": "tools/generate_left.py",
        "source_urdf": SOURCE.as_posix(),
        "source_urdf_sha256": hashlib.sha256(source_bytes).hexdigest(),
        "output_urdf": TARGET.as_posix(),
        "output_urdf_sha256": hashlib.sha256(outputs[TARGET]).hexdigest(),
        "reflection": {"matrix_diagonal": [1, -1, 1], "plane": "base/flange XZ"},
        "joint_coordinate_convention": "same q, limits and mimic relations as right reference",
        "status": "derived geometric reference; hardware left-hand calibration unverified",
        "meshes": manifest_meshes,
    }
    outputs[MESH_DIR / "manifest.json"] = (json.dumps(manifest, indent=2) + "\n").encode()
    return outputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true",
                        help="check generated artifacts are current without writing")
    args = parser.parse_args()
    try:
        outputs = generate(PACKAGE_DIR)
        existing_meshes = set((PACKAGE_DIR / MESH_DIR).glob("*.STL"))
        expected_meshes = {PACKAGE_DIR / path for path in outputs if path.suffix == ".STL"}
        extras = existing_meshes - expected_meshes
        if extras:
            raise ValueError("Unexpected generated meshes: " + ", ".join(map(str, sorted(extras))))
        stale = []
        for path, expected in outputs.items():
            destination = PACKAGE_DIR / path
            if args.check:
                if not destination.is_file() or destination.read_bytes() != expected:
                    stale.append(str(path))
            else:
                destination.parent.mkdir(parents=True, exist_ok=True)
                temporary = destination.with_suffix(destination.suffix + ".tmp")
                temporary.write_bytes(expected)
                temporary.replace(destination)
        if stale:
            print("Missing or stale generated files:\n" + "\n".join(stale), file=sys.stderr)
            return 1
        print(f"{'Verified' if args.check else 'Generated'} {len(outputs)} files "
              f"({len(expected_meshes)} meshes, {sum(map(len, outputs.values())):,} bytes).")
        return 0
    except (OSError, ValueError, ET.ParseError) as exc:
        print(f"Generation failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
