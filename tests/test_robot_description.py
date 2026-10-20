"""URDF edge cases and independent checks of rigid-transform conventions."""

import math
from pathlib import Path
import sys
import unittest

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))

from robot_description import RobotDescriptionError, parse_robot_description
from transforms import (
    IDENTITY_QUATERNION,
    compose_transforms,
    matrix_column_major,
    quaternion_from_axis_angle,
    quaternion_from_rpy,
    rotate_vector,
)


def arm(joint_attributes="type='fixed'", joint_content="", link_content=""):
    return (
        "<robot><link name='tip'>" + link_content + "</link>"
        "<joint name='joint' " + joint_attributes + ">"
        "<parent link='base'/><child link='tip'/>" + joint_content + "</joint>"
        "<link name='base'/></robot>"
    )


class RobotDescriptionTests(unittest.TestCase):
    def test_defaults_and_order_independent_root(self):
        robot = parse_robot_description(arm())
        self.assertEqual(robot.name, "robot")
        self.assertEqual(robot.root_link, "base")
        joint = robot.joints[0]
        self.assertEqual(joint.origin_xyz, (0.0, 0.0, 0.0))
        self.assertEqual(joint.origin_quaternion, IDENTITY_QUATERNION)
        self.assertEqual(joint.axis, (1.0, 0.0, 0.0))
        self.assertEqual(joint.joint_transform(), ((0.0, 0.0, 0.0), IDENTITY_QUATERNION))

    def test_real_xml_variation_and_ignored_nested_elements(self):
        robot = parse_robot_description(""" \t\n<?xml version='1.0' encoding='UTF-8'?>
            <?course robot?>
            <robot xmlns='urn:example' xmlns:xacro='urn:xacro' name='A&amp;B'>
              <!-- <joint name='fake'/> -->
              <gazebo><joint/><link/><![CDATA[<joint/>]]></gazebo>
              <joint name='a&amp;b' type='fixed'>
                <parent link='b&#97;se'/><child link='tip'/>
                <origin xyz=' +1&#9;.25&#10;-2E+0 ' rpy='0 0 0'></origin>
                <axis xyz='0 0 0'/><limit lower='bad'/>
              </joint>
              <link name='tip'/><link name='base'/>
            </robot>""")
        self.assertEqual(robot.name, "A&B")
        self.assertEqual(robot.joints[0].name, "a&b")
        self.assertEqual(robot.joints[0].origin_xyz, (1.0, 0.25, -2.0))
        self.assertEqual(set(robot.links), {"base", "tip"})

    def test_visual_shapes_and_later_visual_ignored(self):
        shapes = [
            ("<box size='1 .2 3.'/>", "box"),
            ("<cylinder radius='1e-400' length='+2'/>", "cylinder"),
            ("<sphere radius='.5'/>", "sphere"),
            ("<mesh filename='package://a&amp;b.dae' scale='2 3 4'/>", "mesh"),
        ]
        for geometry, shape in shapes:
            with self.subTest(shape=shape):
                content = "<visual><origin xyz='1 2 3'/><geometry>" + geometry + "</geometry></visual><visual/>"
                robot = parse_robot_description(arm(link_content=content))
                self.assertEqual(robot.links["tip"].visual.shape, shape)
                self.assertEqual(robot.links["tip"].visual.origin_xyz, (1.0, 2.0, 3.0))

    def test_rejected_descriptions(self):
        candidates = [
            None, {}, 3, "", " \t", "<robot>", "<world/>", "<robot/>",
            "<robot><link/></robot>",
            "<robot><gazebo><link name='base'/></gazebo></robot>",
            "<robot><link name='one'/><link name='two'/></robot>",
            arm(joint_attributes="type='floating'"),
            arm(joint_attributes="type='Revolute'"),
            arm(joint_attributes=""),
            arm(joint_attributes="type='revolute'"),
            arm(joint_attributes="type='prismatic'", joint_content="<limit upper='two'/>"),
            arm(joint_content="<origin xyz='1 2'/>"),
            arm(joint_content="<origin xyz='1 2 3 4'/>"),
            arm(joint_content="<origin xyz='0,0,1'/>"),
            arm(joint_content="<origin rpy=''/>"),
            arm(joint_content="<axis xyz='1_000 0 0'/>"),
            arm(joint_content="<axis xyz='two 0 0'/>"),
            arm(link_content="<visual/>"),
            arm(link_content="<visual><geometry/></visual>"),
            arm(link_content="<visual><geometry><capsule/></geometry></visual>"),
            arm(link_content="<visual><geometry><sphere/></geometry></visual>"),
            arm(link_content="<visual><geometry><box size='1 2'/></geometry></visual>"),
            arm(link_content="<visual><geometry><mesh/></geometry></visual>"),
            arm(link_content="<visual><geometry><mesh filename='x' scale='1 2 nope'/></geometry></visual>"),
            arm(link_content="<visual><origin rpy='0 x 0'/><geometry><sphere radius='1'/></geometry></visual>"),
        ]
        for value in candidates:
            with self.subTest(description=value):
                with self.assertRaises(RobotDescriptionError):
                    parse_robot_description(value)

    def test_limits_defaults_and_continuous_limit_ignored(self):
        robot = parse_robot_description(arm("type='revolute'", "<limit/>"))
        self.assertEqual((robot.joints[0].lower, robot.joints[0].upper), (0.0, 0.0))
        parse_robot_description(arm("type='continuous'", "<limit lower='bad'/>"))

    def test_ambiguous_graphs_rejected_without_looping(self):
        cycle = """<robot><link name='root'/><link name='a'/><link name='b'/>
          <joint name='ab' type='fixed'><parent link='a'/><child link='b'/></joint>
          <joint name='ba' type='fixed'><parent link='b'/><child link='a'/></joint></robot>"""
        with self.assertRaises(RobotDescriptionError):
            parse_robot_description(cycle)
        with self.assertRaises(RobotDescriptionError):
            parse_robot_description(arm().replace("<link name='base'/>", "<link name='base'/><link name='base'/>"))
        with self.assertRaises(RobotDescriptionError):
            parse_robot_description(arm().replace("parent link='base'", "parent link='missing'"))

    def test_large_deep_robot_and_supplied_fetch(self):
        count = 1800
        links = [f"<link name='l{index}'/>" for index in range(count)]
        joints = [
            f"<joint name='j{index}' type='fixed'><parent link='l{index}'/>"
            f"<child link='l{index + 1}'/></joint>"
            for index in range(count - 1)
        ]
        robot = parse_robot_description("<robot>" + "".join(links + joints) + "</robot>")
        self.assertEqual(robot.root_link, "l0")
        self.assertEqual(len(robot.links), count)
        robot = parse_robot_description((PROJECT / "robots/fetch/fetch.urdf").read_text(encoding="utf-8"))
        self.assertEqual(robot.root_link, "base_link")
        self.assertEqual(len(robot.links), len(robot.joints) + 1)


class TransformTests(unittest.TestCase):
    def assert_vector_close(self, actual, expected):
        for value, reference in zip(actual, expected):
            self.assertAlmostEqual(value, reference, places=12)

    def test_noncommuting_fixed_axis_rpy(self):
        rotation = quaternion_from_rpy(math.pi / 2, math.pi / 2, 0.0)
        self.assert_vector_close(rotate_vector(rotation, (1.0, 0.0, 0.0)), (0.0, 0.0, -1.0))
        self.assert_vector_close(rotate_vector(rotation, (0.0, 1.0, 0.0)), (1.0, 0.0, 0.0))
        self.assert_vector_close(rotate_vector(rotation, (0.0, 0.0, 1.0)), (0.0, -1.0, 0.0))

    def test_composition_and_column_major_storage(self):
        translation, rotation = compose_transforms(
            (1.0, 2.0, 3.0), quaternion_from_rpy(0.0, 0.0, math.pi / 2),
            (2.0, 0.0, 0.0), quaternion_from_rpy(math.pi / 2, 0.0, 0.0),
        )
        self.assert_vector_close(translation, (1.0, 4.0, 3.0))
        self.assert_vector_close(rotate_vector(rotation, (0.0, 0.0, 1.0)), (1.0, 0.0, 0.0))
        matrix = matrix_column_major(translation, rotation)
        self.assertEqual(len(matrix), 16)
        self.assert_vector_close(matrix[12:15], (1.0, 4.0, 3.0))
        self.assert_vector_close([matrix[index] for index in (3, 7, 11, 15)], (0.0, 0.0, 0.0, 1.0))
        self.assert_vector_close(matrix[8:11], (1.0, 0.0, 0.0))

    def test_joint_origin_precedes_motion_and_limits_do_not_clamp(self):
        joint = parse_robot_description(arm("type='prismatic'", """
            <origin xyz='1 2 3' rpy='0 0 1.5707963267948966'/>
            <axis xyz='2 0 0'/><limit lower='0' upper='.1'/>""")).joints[0]
        self.assert_vector_close(joint.joint_transform(.7)[0], (1.0, 3.4, 3.0))
        joint = parse_robot_description(arm("type='continuous'", "<axis xyz='0 0 2'/>" )).joints[0]
        self.assert_vector_close(rotate_vector(joint.joint_transform(math.pi / 2)[1], (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))
        self.assert_vector_close(rotate_vector(quaternion_from_axis_angle((1.0, 1.0, 0.0), math.pi), (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))


if __name__ == "__main__":
    unittest.main()
