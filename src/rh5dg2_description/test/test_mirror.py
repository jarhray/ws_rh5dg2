#!/usr/bin/env python3
"""Independent whole-model checks for the derived left RH5DG2 model.

Run without ROS: /usr/bin/python3 test/test_mirror.py
Uses its own URDF forward kinematics; it does not import the generator.
"""

from pathlib import Path
import struct
import unittest
import xml.etree.ElementTree as ET

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]
REFLECTION = np.diag([1.0, -1.0, 1.0])  # XZ plane in the flange frame.
STL_TRIANGLE = np.dtype([
    ("normal", "<f4", (3,)),
    ("vertices", "<f4", (3, 3)),
    ("attribute", "<u2"),
])


def vector(text, default):
    return np.array([float(x) for x in (text or default).split()])


def origin(element):
    """URDF fixed-axis roll, pitch, yaw; translation is in parent frame."""
    result = np.eye(4)
    if element is None:
        return result
    roll, pitch, yaw = vector(element.get("rpy"), "0 0 0")
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    result[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    result[:3, 3] = vector(element.get("xyz"), "0 0 0")
    return result


def joint_motion(joint, q):
    transform = np.eye(4)
    if joint.get("type") == "fixed":
        return transform
    axis = vector(joint.find("axis").get("xyz"), "1 0 0")
    axis /= np.linalg.norm(axis)
    if joint.get("type") == "prismatic":
        transform[:3, 3] = q * axis
    elif joint.get("type") in ("revolute", "continuous"):
        x, y, z = axis
        cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        transform[:3, :3] = (
            np.eye(3) + np.sin(q) * cross + (1 - np.cos(q)) * (cross @ cross)
        )
    else:
        raise AssertionError("Unsupported joint type: " + str(joint.get("type")))
    return transform


class Model:
    def __init__(self, side):
        self.side = side
        self.xml = ET.parse(PACKAGE / "urdf" / ("rh5dg2_" + side + ".xacro"))
        self.links = {x.get("name"): x for x in self.xml.findall("link")}
        self.joints = {x.get("name"): x for x in self.xml.findall("joint")}
        self.incoming = {
            x.find("child").get("link"): x for x in self.joints.values()
        }
        roots = set(self.links) - set(self.incoming)
        if len(roots) != 1 or len(self.incoming) != len(self.joints):
            raise AssertionError("The model must be a connected tree with one root")
        self.root = roots.pop()

    def fk(self, independent_q):
        poses = {self.root: np.eye(4)}
        angles = {}

        def angle(name):
            if name not in angles:
                joint = self.joints[name]
                mimic = joint.find("mimic")
                if joint.get("type") == "fixed":
                    angles[name] = 0.0
                elif mimic is not None:
                    angles[name] = (
                        float(mimic.get("multiplier", "1")) * angle(mimic.get("joint"))
                        + float(mimic.get("offset", "0"))
                    )
                else:
                    angles[name] = independent_q[name.removeprefix(self.side + "_")]
            return angles[name]

        def pose(name):
            if name not in poses:
                joint = self.incoming[name]
                poses[name] = (
                    pose(joint.find("parent").get("link"))
                    @ origin(joint.find("origin"))
                    @ joint_motion(joint, angle(joint.get("name")))
                )
            return poses[name]

        for name in self.links:
            pose(name)
        return poses


def mesh_path(mesh):
    prefix = "package://rh5dg2_description/"
    uri = mesh.get("filename")
    if not uri.startswith(prefix):
        raise AssertionError("Unexpected mesh URI: " + uri)
    return PACKAGE / uri[len(prefix):]


def read_stl(path):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * count or not count:
        raise AssertionError("Invalid binary STL: " + str(path))
    return np.frombuffer(data, dtype=STL_TRIANGLE, count=count, offset=84)


def body_in_flange(link, link_pose):
    inertial = link.find("inertial")
    pose = link_pose @ origin(inertial.find("origin"))
    mass = float(inertial.find("mass").get("value"))
    attributes = inertial.find("inertia").attrib
    a = {key: float(value) for key, value in attributes.items()}
    tensor = np.array([
        [a["ixx"], a["ixy"], a["ixz"]],
        [a["ixy"], a["iyy"], a["iyz"]],
        [a["ixz"], a["iyz"], a["izz"]],
    ])
    rotation, center = pose[:3, :3], pose[:3, 3]
    # Inertia about the flange origin, including the parallel-axis term.
    about_flange = rotation @ tensor @ rotation.T + mass * (
        np.dot(center, center) * np.eye(3) - np.outer(center, center)
    )
    return mass, center, about_flange


class MirrorModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.right = Model("right")
        cls.left = Model("left")
        cls.active = {
            name.removeprefix("right_"): joint
            for name, joint in cls.right.joints.items()
            if joint.get("type") != "fixed" and joint.find("mimic") is None
        }
        rng = np.random.default_rng(20261012)
        cls.configurations = []
        for sample in range(23):
            q = {}
            for name, joint in cls.active.items():
                lower = float(joint.find("limit").get("lower"))
                upper = float(joint.find("limit").get("upper"))
                fraction = [0.0, 0.5, 1.0][sample] if sample < 3 else rng.random()
                q[name] = lower + fraction * (upper - lower)
            cls.configurations.append(q)

    def assert_close(self, actual, expected, atol=2e-12):
        np.testing.assert_allclose(actual, expected, rtol=1e-10, atol=atol)

    def test_names_tree_mimic_and_asset_references(self):
        self.assertEqual(len(self.right.links), 27)
        self.assertEqual(len(self.right.joints), 26)
        self.assertEqual(len(self.active), 13)
        self.assertEqual(sum(x.find("mimic") is not None
                             for x in self.right.joints.values()), 5)
        self.assertEqual(set(self.left.links),
                         {x.replace("right_", "left_", 1) for x in self.right.links})
        self.assertEqual(set(self.left.joints),
                         {x.replace("right_", "left_", 1) for x in self.right.joints})
        self.assertEqual(self.left.root, "left_hand_flange")
        paths = set()
        for rname, right in self.right.joints.items():
            left = self.left.joints[rname.replace("right_", "left_", 1)]
            with self.subTest(joint=rname):
                self.assertEqual(left.get("type"), right.get("type"))
                for tag in ("parent", "child"):
                    self.assertEqual(left.find(tag).get("link"),
                                     right.find(tag).get("link").replace("right_", "left_", 1))
                for tag in ("limit", "dynamics", "safety_controller"):
                    r, l = right.find(tag), left.find(tag)
                    self.assertEqual(None if l is None else l.attrib,
                                     None if r is None else r.attrib)
                r, l = right.find("mimic"), left.find("mimic")
                self.assertEqual(r is None, l is None)
                if r is not None:
                    self.assertEqual(l.get("joint"), r.get("joint").replace("right_", "left_", 1))
                    self.assertIn(l.get("joint"), self.left.joints)
                    for key, default in (("multiplier", "1"), ("offset", "0")):
                        self.assertEqual(float(l.get(key, default)), float(r.get(key, default)))
        for link in self.left.links.values():
            for tag in ("visual", "collision"):
                elements = link.findall(tag)
                self.assertTrue(elements)
                for element in elements:
                    mesh = element.find("geometry/mesh")
                    self.assertIsNotNone(mesh)
                    self.assertNotIn("right_", mesh.get("filename"))
                    path = mesh_path(mesh)
                    self.assertTrue(path.is_file(), str(path))
                    self.assertEqual(path.parent.name, "left_mirrored")
                    self.assertTrue(path.name.startswith("left_"))
                    self.assertTrue(np.all(vector(mesh.get("scale"), "1 1 1") > 0))
                    paths.add(path)
        self.assertEqual(len(paths), 27)

    def test_all_links_forward_kinematics_at_same_joint_coordinates(self):
        for index, q in enumerate(self.configurations):
            right_poses, left_poses = self.right.fk(q), self.left.fk(q)
            for name, right in right_poses.items():
                with self.subTest(configuration=index, link=name):
                    left = left_poses[name.replace("right_", "left_", 1)]
                    self.assert_close(left[:3, 3], REFLECTION @ right[:3, 3])
                    self.assert_close(left[:3, :3], REFLECTION @ right[:3, :3] @ REFLECTION)
                    self.assert_close(left[:3, :3].T @ left[:3, :3], np.eye(3))
                    self.assertAlmostEqual(np.linalg.det(left[:3, :3]), 1.0, places=11)

    def test_inertia_and_center_of_mass_in_full_chain(self):
        for index, q in enumerate(self.configurations[::4]):
            right_poses, left_poses = self.right.fk(q), self.left.fk(q)
            mass_r = mass_l = 0.0
            center_r, center_l = np.zeros(3), np.zeros(3)
            inertia_r, inertia_l = np.zeros((3, 3)), np.zeros((3, 3))
            for name, right_link in self.right.links.items():
                lname = name.replace("right_", "left_", 1)
                rm, rc, ri = body_in_flange(right_link, right_poses[name])
                lm, lc, li = body_in_flange(self.left.links[lname], left_poses[lname])
                with self.subTest(configuration=index, link=name):
                    self.assertEqual(lm, rm)
                    self.assertGreater(lm, 0)
                    self.assert_close(lc, REFLECTION @ rc)
                    self.assert_close(li, REFLECTION @ ri @ REFLECTION)
                mass_r += rm
                mass_l += lm
                center_r += rm * rc
                center_l += lm * lc
                inertia_r += ri
                inertia_l += li
            self.assertEqual(mass_l, mass_r)
            self.assert_close(center_l / mass_l, REFLECTION @ (center_r / mass_r))
            self.assert_close(inertia_l, REFLECTION @ inertia_r @ REFLECTION)

    def test_every_binary_stl_triangle_and_outward_winding(self):
        originals = sorted((PACKAGE / "meshes").glob("right_*.STL"))
        self.assertEqual(len(originals), 27)
        for path in originals:
            with self.subTest(mesh=path.name):
                mirrored = path.parent / "left_mirrored" / path.name.replace("right_", "left_", 1)
                right, left = read_stl(path), read_stl(mirrored)
                self.assertEqual(len(right), len(left))
                self.assertTrue(np.all(np.isfinite(left["vertices"])))
                # Reverse winding after spatial reflection to preserve outward faces.
                self.assert_close(left["vertices"], right["vertices"][:, [0, 2, 1], :] @ REFLECTION,
                                  atol=1e-10)
                self.assert_close(left["normal"], right["normal"] @ REFLECTION, atol=1e-10)
                np.testing.assert_array_equal(left["attribute"], right["attribute"])
                cross = np.cross(left["vertices"][:, 1] - left["vertices"][:, 0],
                                 left["vertices"][:, 2] - left["vertices"][:, 0])
                # Some vendor normals are not exact face normals, but must point
                # into the same hemisphere. Do not conceal pre-existing facets.
                nondegenerate = np.linalg.norm(cross, axis=1) > 1e-13
                self.assertTrue(np.all(np.einsum("ij,ij->i", cross[nondegenerate],
                                                left["normal"][nondegenerate]) > 0))

    def test_articulated_visual_and_collision_surface_points(self):
        # Includes transforms of real off-axis mesh points, not only link origins.
        q = self.configurations[-1]
        right_poses, left_poses = self.right.fk(q), self.left.fk(q)
        for name, right_link in self.right.links.items():
            lname = name.replace("right_", "left_", 1)
            left_link = self.left.links[lname]
            for tag in ("visual", "collision"):
                right_items, left_items = right_link.findall(tag), left_link.findall(tag)
                self.assertEqual(len(right_items), len(left_items))
                for right, left in zip(right_items, left_items):
                    with self.subTest(link=name, geometry=tag):
                        rmesh, lmesh = right.find("geometry/mesh"), left.find("geometry/mesh")
                        rtri, ltri = read_stl(mesh_path(rmesh)), read_stl(mesh_path(lmesh))
                        indices = np.linspace(0, len(rtri) - 1, min(40, len(rtri)), dtype=int)
                        rv = rtri["vertices"][indices].reshape(-1, 3)
                        lv = ltri["vertices"][indices][:, [0, 2, 1], :].reshape(-1, 3)
                        rv = rv * vector(rmesh.get("scale"), "1 1 1")
                        lv = lv * vector(lmesh.get("scale"), "1 1 1")
                        rt = right_poses[name] @ origin(right.find("origin"))
                        lt = left_poses[lname] @ origin(left.find("origin"))
                        rw = rv @ rt[:3, :3].T + rt[:3, 3]
                        lw = lv @ lt[:3, :3].T + lt[:3, 3]
                        self.assert_close(lw, rw @ REFLECTION, atol=2e-10)


if __name__ == "__main__":
    unittest.main(verbosity=2)
