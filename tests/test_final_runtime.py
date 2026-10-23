"""Final-project checks through real TCP clients, with independent matrix math."""

import json
import math
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest

from tests.test_checkpoint import WireClient, ADDRESS, GET, SET, STATUS


ROOT = Path(__file__).resolve().parents[1]
STAMP = {"sec": 157, "nanosec": 987654321}
IDENTITY = [[int(r == c) for c in range(4)] for r in range(4)]


def multiply(left, right):
    return [[sum(left[r][k] * right[k][c] for k in range(4))
             for c in range(4)] for r in range(4)]


def pose(xyz=(0, 0, 0), axis=(1, 0, 0), angle=0):
    """Rodrigues rotation supplies a reference independent of quaternions."""
    norm = math.sqrt(sum(v*v for v in axis))
    x, y, z = [v / norm for v in axis]
    c, s, d = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [[c+x*x*d, x*y*d-z*s, x*z*d+y*s, xyz[0]],
            [y*x*d+z*s, c+y*y*d, y*z*d-x*s, xyz[1]],
            [z*x*d-y*s, z*y*d+x*s, c+z*z*d, xyz[2]], [0, 0, 0, 1]]


def wire_matrix(entry):
    if "matrix" in entry:
        return [[entry["matrix"][4*c+r] for c in range(4)] for r in range(4)]
    transform = entry["transform"]
    q = transform["rotation"]
    x, y, z, w = [q[k] for k in ("x", "y", "z", "w")]
    t = transform["translation"]
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), t["x"]],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), t["y"]],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), t["z"]], [0, 0, 0, 1]]


ROBOT = '''<robot name="final_test">
  <link name="root"/><link name="turn"/><link name="slide"/>
  <link name="tool"/><link name="branch"/>
  <joint name="rotation" type="revolute"><parent link="root"/><child link="turn"/>
    <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/>
    <axis xyz="1 1 0"/><limit lower="-7" upper="7"/></joint>
  <joint name="translation" type="prismatic"><parent link="turn"/><child link="slide"/>
    <origin xyz=".5 0 0" rpy="1.5707963267948966 0 0"/>
    <axis xyz="0 2 0"/><limit lower="-2" upper="2"/></joint>
  <joint name="mount" type="fixed"><parent link="slide"/><child link="tool"/>
    <origin xyz="0 0 .25"/></joint>
  <joint name="spin" type="continuous"><parent link="root"/><child link="branch"/>
    <origin xyz="-1 0 1"/><axis xyz="0 0 2"/></joint>
</robot>'''


class FinalRuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with socket.socket() as probe:
            if probe.connect_ex(ADDRESS) == 0:
                raise RuntimeError("TCP port 9095 is in use; stop the runtime first")
        cls.log = tempfile.TemporaryFile(mode="w+b")
        cls.process = subprocess.Popen([sys.executable, str(ROOT / "src/main.py")],
                                       cwd=ROOT, stdout=cls.log, stderr=cls.log)
        cls.addClassCleanup(cls.stop_runtime)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if cls.process.poll() is not None:
                break
            try:
                client = WireClient()
                client.call(GET, {"name": "startup"}, timeout=.5)
                client.close()
                return
            except OSError:
                time.sleep(.05)
        cls.log.seek(0)
        raise RuntimeError(cls.log.read().decode(errors="replace"))

    @classmethod
    def stop_runtime(cls):
        if cls.process.poll() is None:
            cls.process.terminate()
            cls.process.wait(timeout=5)
        cls.log.close()

    def setUp(self):
        self.client = WireClient()
        self.addCleanup(self.client.close)
        self.client.send({"op": "advertise", "topic": "/joint_states", "type": "sensor_msgs/JointState"})
        self.client.send({"op": "advertise", "topic": "/global_pose", "type": "geometry_msgs/Pose"})
        self.load(ROBOT)
        self.global_pose((0, 0, 0), (0, 0, 0, 1))

    def load(self, description, accepted=True):
        result = self.client.call(SET, {"name": "robot_description", "value": description})
        version = result["values"]["version"]
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            reply = self.client.call(GET, {"name": STATUS})
            status = reply["values"]["value"]
            if status and status["version"] >= version:
                self.assertIs(status["accepted"], accepted, status)
                return status
            time.sleep(.02)
        self.fail("description status missed pickup bound")

    def joints(self, names, positions, stamp=STAMP):
        self.client.send({"op": "publish", "topic": "/joint_states", "msg": {
            "header": {"stamp": stamp, "frame_id": "ignored"},
            "name": names, "position": positions, "extra": "ignored"}})

    def global_pose(self, xyz, quaternion):
        self.client.send({"op": "publish", "topic": "/global_pose", "msg": {
            "position": dict(zip(("x", "y", "z"), xyz)),
            "orientation": dict(zip(("x", "y", "z", "w"), quaternion))}})

    def snapshot(self, topic, stamp=STAMP, client=None):
        client = client or self.client
        client.subscribe(topic, "tf/tfMessage" if topic == "/tf" else "xform_world/MatrixTransformArray")
        message = client.topic(topic, lambda msg: msg["transforms"] and
                               all(e["header"]["stamp"] == stamp for e in msg["transforms"]))
        return {e["child_frame_id"]: e for e in message["transforms"]}

    def assert_pose(self, entry, expected):
        actual = wire_matrix(entry)
        for r in range(4):
            for c in range(4):
                self.assertAlmostEqual(actual[r][c], expected[r][c], delta=1e-5)
        if "transform" in entry:
            q = entry["transform"]["rotation"]
            self.assertAlmostEqual(sum(v*v for v in q.values()), 1, delta=1e-5)

    def expected_edges(self, rotation=0, translation=0, spin=0):
        return {
            "turn": multiply(pose((1, 2, 3), (0, 0, 1), math.pi/2),
                             pose(axis=(1, 1, 0), angle=rotation)),
            "slide": multiply(pose((.5, 0, 0), (1, 0, 0), math.pi/2),
                              pose((0, 2*translation, 0))),
            "tool": pose((0, 0, .25)),
            "branch": pose((-1, 0, 1), (0, 0, 1), spin),
        }

    def test_00_nonunit_axes_origin_motion_order_and_full_world_chain(self):
        for rotation in (math.pi, -.73):
            with self.subTest(rotation=rotation):
                stamp = {"sec": int(rotation*100), "nanosec": 1234567}
                self.joints(["rotation", "translation", "spin"], [rotation, .7, 1.7*math.pi], stamp)
                edges = self.expected_edges(rotation, .7, 1.7*math.pi)
                tf = self.snapshot("/tf", stamp)
                self.assertEqual(set(tf), set(edges))
                for name in edges:
                    self.assert_pose(tf[name], edges[name])
                world = self.snapshot("/xform_world", stamp)
                self.assertEqual(set(world), {"root", *edges})
                self.assert_pose(world["root"], IDENTITY)
                self.assert_pose(world["slide"], multiply(edges["turn"], edges["slide"]))
                self.assert_pose(world["tool"], multiply(multiply(edges["turn"], edges["slide"]), edges["tool"]))

    def test_01_latest_message_replaces_missing_and_ignores_fixed_unknown(self):
        self.joints(["rotation", "translation", "spin"], [.7, .9, -.8])
        self.snapshot("/tf")
        next_stamp = {"sec": 22, "nanosec": 8}
        self.joints(["spin", "mount", "not_a_joint"], [.3, 91, 92], next_stamp)
        tf = self.snapshot("/tf", next_stamp)
        expected = self.expected_edges(spin=.3)
        for name in expected:
            self.assert_pose(tf[name], expected[name])
        self.joints([], [], {"sec": 23, "nanosec": 9})
        tf = self.snapshot("/tf", {"sec": 23, "nanosec": 9})
        for name, matrix in self.expected_edges().items():
            self.assert_pose(tf[name], matrix)

    def test_02_rejected_reload_preserves_moving_configuration_and_stamp(self):
        self.joints(["rotation"], [.5])
        before = self.snapshot("/tf")
        status = self.load("<robot><link/></robot>", accepted=False)
        self.assertEqual(status["root_link"], "root")
        after = self.snapshot("/tf")
        self.assertEqual(before, after)

    def test_03_world_pose_persists_and_composes_into_every_link(self):
        self.joints(["rotation", "translation"], [.2, -.4])
        self.snapshot("/tf")
        self.global_pose((4, -2, 1), (0, 0, math.sqrt(.5), math.sqrt(.5)))
        global_matrix = pose((4, -2, 1), (0, 0, 1), math.pi/2)
        deadline = time.monotonic() + 2
        while True:
            world = self.snapshot("/xform_world")
            if abs(world["root"]["matrix"][12] - 4) < 1e-6:
                break
            self.assertLess(time.monotonic(), deadline)
        edges = self.expected_edges(.2, -.4)
        root_turn = multiply(global_matrix, edges["turn"])
        expected = {"root": global_matrix, "turn": root_turn,
                    "slide": multiply(root_turn, edges["slide"]),
                    "tool": multiply(multiply(root_turn, edges["slide"]), edges["tool"]),
                    "branch": multiply(global_matrix, edges["branch"])}
        for name, matrix in expected.items():
            self.assert_pose(world[name], matrix)
        late = WireClient()
        self.addCleanup(late.close)
        for name, entry in self.snapshot("/xform_world", client=late).items():
            self.assert_pose(entry, expected[name])

    def test_04_publications_continue_and_late_clients_get_complete_state(self):
        self.joints(["rotation"], [.6])
        self.snapshot("/tf")
        for topic, names in (("/tf", {"turn", "slide", "tool", "branch"}),
                             ("/xform_world", {"root", "turn", "slide", "tool", "branch"})):
            with self.subTest(topic=topic):
                late = WireClient()
                self.addCleanup(late.close)
                start = time.monotonic()
                snapshot = self.snapshot(topic, client=late)
                self.assertLess(time.monotonic() - start, .4)
                self.assertEqual(set(snapshot), names)
                arrivals = [time.monotonic()]
                for _ in range(6):
                    late.topic(topic, timeout=.4)
                    arrivals.append(time.monotonic())
                self.assertGreaterEqual(6 / (arrivals[-1] - arrivals[0]), 5)
                self.assertLess(max(b-a for a, b in zip(arrivals, arrivals[1:])), .3)



if __name__ == "__main__":
    unittest.main()
