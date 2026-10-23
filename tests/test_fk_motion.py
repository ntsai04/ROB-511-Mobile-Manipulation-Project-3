"""Final FK behavior checked against analytically solvable 3-D configurations."""

import math
from pathlib import Path
import sys
import threading
import unittest

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))

from param_server import ParameterServer
from robot_state_publisher import DESCRIPTION_STATUS, RobotStatePublisher
from robot_world_state_publisher import RobotWorldStatePublisher


ROBOT = """<robot name="spatial_arm">
  <link name="tip"/><link name="slider"/><link name="shoulder_link"/>
  <link name="branch"/><link name="base"/>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="shoulder_link"/>
    <origin xyz="1 0 0" rpy="0 1.5707963267948966 0"/>
    <axis xyz="0 0 2"/><limit lower="-4" upper="4"/></joint>
  <joint name="slide" type="prismatic"><parent link="shoulder_link"/><child link="slider"/>
    <origin xyz="0 0 1" rpy="0 0 1.5707963267948966"/>
    <axis xyz="2 0 0"/><limit lower="0" upper="1"/></joint>
  <joint name="tool" type="fixed"><parent link="slider"/><child link="tip"/>
    <origin xyz="1 0 0"/></joint>
  <joint name="branch_joint" type="continuous"><parent link="base"/><child link="branch"/>
    <origin xyz="0 2 0"/><axis xyz="1 1 0"/></joint>
</robot>"""


def state_message(names, positions, sec=31, nanosec=987654321):
    return {"name": names, "position": positions,
            "header": {"stamp": {"sec": sec, "nanosec": nanosec}}}


def pose_message(position=(10.0, -2.0, 1.0)):
    return {"position": dict(zip(("x", "y", "z"), position)),
            "orientation": {"x": math.sqrt(.5), "y": 0.0, "z": 0.0, "w": math.sqrt(.5)},
            "seq": "ignored extra field"}


class ForwardKinematicsTests(unittest.TestCase):
    def setUp(self):
        self.parameters = ParameterServer()
        self.messages = []
        self.world = RobotWorldStatePublisher(self.publish)
        self.state = RobotStatePublisher(self.call, self.publish)

    def call(self, service, args):
        return self.parameters.service_handlers()[service](args)

    def publish(self, topic, message):
        self.messages.append((topic, message))
        if topic == "/tf":
            self.world.accept_tf(message)

    def load(self, description=ROBOT):
        reply = self.parameters.set_param({"name": "robot_description", "value": description})
        self.state.update()
        return reply[1]["version"]

    def latest(self, topic):
        return next(message for name, message in reversed(self.messages) if name == topic)

    def transforms(self, topic):
        return {entry["child_frame_id"]: entry for entry in self.latest(topic)["transforms"]}

    def assert_vector(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for value, reference in zip(actual, expected):
            self.assertAlmostEqual(value, reference, places=11)

    def move(self):
        # Fixed and unknown names affect neither robot motion nor entry count.
        self.state.accept_joint_states(state_message(
            ["slide", "tool", "branch_joint", "not_a_joint", "shoulder"],
            [.7, 8, math.pi, 99, math.pi / 2],
        ))
        self.state.update()
        self.world.update()

    def test_motion_origin_order_nonunit_axes_and_all_stamps(self):
        self.load()
        self.move()
        edges = self.transforms("/tf")
        self.assertEqual(set(edges), {"shoulder_link", "slider", "tip", "branch"})
        self.assert_vector(list(edges["shoulder_link"]["transform"]["translation"].values()), (1, 0, 0))
        self.assert_vector(list(edges["slider"]["transform"]["translation"].values()), (0, 1.4, 1))
        self.assert_vector(list(edges["tip"]["transform"]["translation"].values()), (1, 0, 0))
        world = self.transforms("/xform_world")
        self.assertEqual(set(world), {"base", "shoulder_link", "slider", "tip", "branch"})
        self.assert_vector(world["slider"]["matrix"][12:15], (2, 0, 1.4))
        self.assert_vector(world["tip"]["matrix"][12:15], (2, 0, 2.4))
        self.assert_vector(world["shoulder_link"]["matrix"][:3], (0, 1, 0))
        self.assert_vector(world["shoulder_link"]["matrix"][4:7], (0, 0, 1))
        self.assert_vector(world["branch"]["matrix"][:3], (0, 1, 0))
        for entry in list(edges.values()) + list(world.values()):
            self.assertEqual(entry["header"]["stamp"], {"sec": 31, "nanosec": 987654321})

    def test_each_message_replaces_state_and_empty_resets_every_joint(self):
        self.load()
        self.move()
        self.state.accept_joint_states(state_message(["slide"], [.2], sec=44, nanosec=12))
        self.state.update()
        self.world.update()
        world = self.transforms("/xform_world")
        self.assert_vector(world["tip"]["matrix"][12:15], (2, 1.4, 0))
        self.assert_vector(world["branch"]["matrix"][:3], (1, 0, 0))
        for entry in self.latest("/tf")["transforms"]:
            self.assertEqual(entry["header"]["stamp"], {"sec": 44, "nanosec": 12})
        self.state.accept_joint_states(state_message([], [], sec=45, nanosec=13))
        self.state.update()
        self.world.update()
        self.assert_vector(self.transforms("/xform_world")["tip"]["matrix"][12:15], (2, 1, 0))

    def test_global_pose_persists_and_updates_without_a_new_tf(self):
        self.load()
        self.move()
        self.world.accept_global_pose(pose_message())
        self.world.update()
        world = self.transforms("/xform_world")
        self.assert_vector(world["base"]["matrix"][12:15], (10, -2, 1))
        self.assert_vector(world["tip"]["matrix"][12:15], (12, -4.4, 1))
        self.assert_vector(world["tip"]["matrix"][:3], (0, -1, 0))
        self.assert_vector(world["tip"]["matrix"][4:7], (0, 0, -1))
        self.assert_vector(world["tip"]["matrix"][8:11], (1, 0, 0))
        self.assert_vector(world["branch"]["matrix"][12:15], (10, -2, 3))
        self.world.accept_global_pose(pose_message(position=(-1, 4, 2)))
        self.world.update()
        world = self.transforms("/xform_world")
        self.assert_vector(world["tip"]["matrix"][12:15], (1, 1.6, 2))
        self.assertEqual(world["tip"]["header"]["stamp"], {"sec": 31, "nanosec": 987654321})

    def test_pose_before_tf_and_reload_replace_tree_without_resetting_pose(self):
        self.world.accept_global_pose(pose_message())
        self.world.update()
        self.assertEqual(self.messages, [])
        self.load()
        self.move()
        self.load("""<robot><link name='newtip'/><link name='newbase'/>
          <joint name='newjoint' type='fixed'><parent link='newbase'/><child link='newtip'/>
          <origin xyz='0 0 2'/></joint></robot>""")
        self.world.update()
        world = self.transforms("/xform_world")
        self.assertEqual(set(world), {"newbase", "newtip"})
        self.assert_vector(world["newbase"]["matrix"][12:15], (10, -2, 1))
        self.assert_vector(world["newtip"]["matrix"][12:15], (10, -4, 1))

    def test_invalid_description_preserves_current_motion(self):
        loaded = self.load()
        self.move()
        previous = self.latest("/tf")
        rejected = self.load("<robot><link/></robot>")
        self.assertEqual(self.latest("/tf"), previous)
        status = self.parameters.get_param({"name": DESCRIPTION_STATUS})[1]["value"]
        self.assertEqual(status["version"], rejected)
        self.assertFalse(status["accepted"])
        self.assertEqual(status["loaded_version"], loaded)
        self.assertEqual(status["root_link"], "base")

    def test_repeated_and_malformed_states_leave_consistent_snapshots(self):
        self.load()
        self.move()
        previous = self.latest("/tf")
        for bad in (None, {}, state_message(["shoulder"], []), state_message(["shoulder"], [float("nan")])):
            self.state.accept_joint_states(bad)
            self.state.update()
            self.assertEqual(self.latest("/tf"), previous)
        self.move()
        self.assertEqual(self.latest("/tf"), previous)

    def test_positions_and_stamps_are_atomic_between_threads(self):
        self.load("""<robot><link name='root'/><link name='tip'/>
          <joint name='slide' type='prismatic'><parent link='root'/><child link='tip'/>
          <limit lower='0' upper='10'/></joint></robot>""")
        started = threading.Event()

        def receive():
            started.set()
            for number in range(1, 301):
                self.state.accept_joint_states(state_message(["slide"], [number / 100], sec=number, nanosec=0))

        receiver = threading.Thread(target=receive)
        receiver.start()
        started.wait()
        for _ in range(100):
            self.state.update()
        receiver.join()
        self.state.update()
        for topic, message in self.messages:
            if topic == "/tf":
                entry = message["transforms"][0]
                self.assertAlmostEqual(entry["transform"]["translation"]["x"], entry["header"]["stamp"]["sec"] / 100)


if __name__ == "__main__":
    unittest.main()
