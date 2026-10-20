"""Run with ``python -m unittest discover -s tests -v`` from Project 3.

The runtime is a child process and every assertion uses its public TCP/JSON
interface. No implementation modules are imported. Port 9095 must be free.
Near-1 MiB transport coverage also guards a requirement graded in the final.
"""

from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
ADDRESS = ("127.0.0.1", 9095)
SET = "/param_server/set_param"
GET = "/param_server/get_param"
STATUS = "/robot_state_publisher/description_status"
IDENTITY = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
ZERO_STAMP = {"sec": 0, "nanosec": 0}


class WireClient:
    """Minimal newline-framed client that retains interleaved topic traffic."""

    def __init__(self):
        self.socket = socket.create_connection(ADDRESS, timeout=2)
        self.buffer = bytearray()
        self.publications = deque()
        self.next_id = 0

    def close(self):
        self.socket.close()

    def send(self, message):
        encoded = json.dumps(message, separators=(",", ":"), allow_nan=False)
        self.socket.sendall((encoded + "\n").encode("utf-8"))

    def receive(self, timeout):
        deadline = time.monotonic() + timeout
        while True:
            newline = self.buffer.find(b"\n")
            if newline >= 0:
                line = bytes(self.buffer[:newline])
                del self.buffer[:newline + 1]
                if line.strip():
                    return json.loads(line)
                continue
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("timed out waiting for a complete JSON line")
            self.socket.settimeout(remaining)
            try:
                data = self.socket.recv(65536)
            except socket.timeout as error:
                raise TimeoutError("timed out waiting for TCP traffic") from error
            if not data:
                raise ConnectionError("runtime closed the client connection")
            self.buffer.extend(data)

    def call(self, service, args, timeout=3, call_id=None):
        if call_id is None:
            self.next_id += 1
            call_id = "test-" + str(self.next_id)
        self.send({"op": "call_service", "service": service,
                   "args": args, "id": call_id})
        deadline = time.monotonic() + timeout
        while True:
            reply = self.receive(max(0, deadline - time.monotonic()))
            if reply.get("op") == "service_response" and reply.get("id") == call_id:
                if reply.get("service") != service:
                    raise AssertionError("service response named the wrong service")
                return reply
            if reply.get("op") == "publish":
                self.publications.append(reply)

    def subscribe(self, topic, type_name):
        self.send({"op": "subscribe", "topic": topic, "type": type_name})

    def topic(self, topic, predicate=lambda _msg: True, timeout=3):
        deadline = time.monotonic() + timeout
        while True:
            while self.publications:
                message = self.publications.popleft()
                if message.get("topic") == topic and predicate(message.get("msg")):
                    return message["msg"]
            message = self.receive(max(0, deadline - time.monotonic()))
            if message.get("op") == "publish":
                self.publications.append(message)


def simple_robot(root="base", child="tip", joint_type="fixed", extra=""):
    limit = "<limit/>" if joint_type in ("revolute", "prismatic") else ""
    return (f'<robot><link name="{root}"/><link name="{child}"/>'
            f'<joint name="mount" type="{joint_type}"><parent link="{root}"/>'
            f'<child link="{child}"/>{limit}{extra}</joint></robot>')


def quaternion_rotation(q):
    """Convert wire quaternions, independent of the runtime's FK implementation."""
    x, y, z, w = (q[key] for key in ("x", "y", "z", "w"))
    return ((1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)),
            (2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)),
            (2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)))


class CheckpointTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Refuse to test or terminate an unrelated runtime using the course port.
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            if probe.connect_ex(ADDRESS) == 0:
                raise RuntimeError("TCP port 9095 is in use; stop that runtime first")
        cls.log = tempfile.TemporaryFile(mode="w+b")
        cls.process = subprocess.Popen(
            [sys.executable, str(ROOT / "src" / "main.py")], cwd=ROOT,
            stdout=cls.log, stderr=cls.log,
        )
        cls.addClassCleanup(cls.stop_runtime)
        started = time.monotonic()
        last_error = None
        while time.monotonic() - started < 10:
            if cls.process.poll() is not None:
                break
            try:
                client = WireClient()
                try:
                    reply = client.call(GET, {"name": "__startup_probe__"}, timeout=0.5)
                    if reply.get("result") is False and reply.get("values") == {
                            "value": None, "version": 0}:
                        return
                finally:
                    client.close()
            except (OSError, ConnectionError, TimeoutError) as error:
                last_error = error
            time.sleep(0.05)
        cls.log.seek(0)
        output = cls.log.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"runtime was not ready within 10 seconds: {last_error}\n{output}")

    @classmethod
    def stop_runtime(cls):
        if cls.process.poll() is None:
            cls.process.terminate()
            try:
                cls.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                cls.process.kill()
                cls.process.wait(timeout=5)
        cls.log.close()

    def setUp(self):
        self.client = WireClient()
        self.addCleanup(self.client.close)

    def get(self, name):
        return self.client.call(GET, {"name": name})

    def set(self, name, value):
        reply = self.client.call(SET, {"name": name, "value": value})
        self.assertIs(reply.get("result"), True, reply)
        self.assertIsInstance(reply.get("status"), str)
        self.assertIs(type(reply["values"]["version"]), int)
        return reply["values"]["version"]

    def load(self, description, accepted=True, previous=None, root="base"):
        version = self.set("robot_description", description)
        deadline = time.monotonic() + 2
        latest = None
        while time.monotonic() < deadline:
            response = self.get(STATUS)
            if response.get("result") is True:
                latest = response["values"]["value"]
                if latest.get("version", 0) >= version:
                    break
            time.sleep(0.02)
        else:
            self.fail(f"description version {version} had no status within 2s: {latest}")
        self.assertEqual(latest["version"], version)
        self.assertIs(latest["accepted"], accepted, latest)
        if accepted:
            self.assertEqual(latest["error"], "")
            self.assertEqual(latest["loaded_version"], version)
        else:
            self.assertIsInstance(latest["error"], str)
            self.assertTrue(latest["error"])
            self.assertEqual(latest["loaded_version"], previous)
        self.assertEqual(latest["root_link"], root)
        return version

    def subscribe_fk(self):
        self.client.subscribe("/tf", "tf/tfMessage")
        self.client.subscribe("/xform_world", "xform_world/MatrixTransformArray")

    def transforms(self, topic, children):
        expected = set(children)
        def matches(message):
            if not isinstance(message, dict):
                return False
            names = {entry.get("child_frame_id") for entry in message.get("transforms", [])}
            if topic == "/xform_world":
                names.discard("global_frame")
            return names == expected
        message = self.client.topic(topic, matches)
        entries = message["transforms"]
        result = {entry["child_frame_id"]: entry for entry in entries}
        self.assertEqual(len(result), len(entries), "duplicate transform entries")
        if "global_frame" in result:
            self.assertEqual(topic, "/xform_world")
            self.assert_world(result.pop("global_frame"), (0, 0, 0), IDENTITY)
        self.assertEqual(set(result), expected)
        return result

    def assert_rotation(self, actual, expected):
        for row in actual:
            for value in row:
                self.assert_number(value)
        # Geodesic rotation error is insensitive to quaternion sign.
        trace = sum(actual[r][c] * expected[r][c] for r in range(3) for c in range(3))
        angle = math.acos(max(-1.0, min(1.0, (trace - 1) / 2)))
        self.assertLessEqual(angle, 1e-4)
        for r in range(3):
            for c in range(3):
                dot = sum(actual[k][r] * actual[k][c] for k in range(3))
                self.assertAlmostEqual(dot, int(r == c), delta=1e-4)
        a = actual
        determinant = (a[0][0]*(a[1][1]*a[2][2] - a[1][2]*a[2][1])
                       - a[0][1]*(a[1][0]*a[2][2] - a[1][2]*a[2][0])
                       + a[0][2]*(a[1][0]*a[2][1] - a[1][1]*a[2][0]))
        self.assertAlmostEqual(determinant, 1, delta=1e-4)

    def assert_number(self, value):
        self.assertIn(type(value), (int, float))
        self.assertTrue(math.isfinite(value))

    def assert_tf(self, entry, parent, xyz, rotation=IDENTITY):
        self.assertEqual(entry["header"]["frame_id"], parent)
        self.assertEqual(entry["header"]["stamp"], ZERO_STAMP)
        transform = entry["transform"]
        for key, value in zip(("x", "y", "z"), xyz):
            self.assert_number(transform["translation"][key])
            self.assertAlmostEqual(transform["translation"][key], value, delta=1e-4)
        q = transform["rotation"]
        for key in ("x", "y", "z", "w"):
            self.assert_number(q[key])
        self.assertAlmostEqual(math.sqrt(sum(q[key]**2 for key in ("x", "y", "z", "w"))),
                               1, delta=1e-4)
        self.assert_rotation(quaternion_rotation(q), rotation)

    def assert_world(self, entry, xyz, rotation=IDENTITY):
        self.assertEqual(entry["header"]["frame_id"], "global_frame")
        self.assertEqual(entry["header"]["stamp"], ZERO_STAMP)
        matrix = entry["matrix"]
        self.assertEqual(len(matrix), 16)
        for value in matrix:
            self.assert_number(value)
        for index, value in zip((12, 13, 14), xyz):
            self.assertAlmostEqual(matrix[index], value, delta=1e-4)
        for index, value in zip((3, 7, 11, 15), (0, 0, 0, 1)):
            self.assertAlmostEqual(matrix[index], value, delta=1e-4)
        self.assert_rotation(tuple(tuple(matrix[4*c + r] for c in range(3))
                                   for r in range(3)), rotation)

    def test_00_startup_has_no_robot_or_input_publishers(self):
        self.assertIs(self.get("robot_description")["result"], False)
        self.client.subscribe("/tf", "tf/tfMessage")
        self.client.subscribe("/joint_states", "sensor_msgs/JointState")
        self.client.subscribe("/joint_trajectory", "trajectory_msgs/JointTrajectory")
        self.client.subscribe("/global_pose", "geometry_msgs/Pose")
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            try:
                message = self.client.receive(deadline - time.monotonic())
            except TimeoutError:
                break
            self.assertNotEqual(message.get("op"), "publish", message)

    def test_01_parameter_values_versions_and_opaque_ids(self):
        name = "test/checkpoint/one"
        missing = self.get(name)
        self.assertIs(missing["result"], False)
        self.assertTrue(missing["status"])
        self.assertEqual(missing["values"], {"value": None, "version": 0})
        value = {"nested": [None, True, 3.5, "a\nb", {"quote": "\"&"}]}
        self.assertEqual(self.set(name, value), 1)
        self.assertEqual(self.set("test/checkpoint/other", [1, 2]), 1)
        reply = self.client.call(GET, {"name": name}, call_id={"opaque": [7, "x"]})
        self.assertIs(reply["result"], True)
        self.assertEqual(reply["values"], {"value": value, "version": 1})
        self.assertEqual(self.set(name, value), 2, "even identical values increment the version")
        reply = self.client.call(SET, {"name": name})
        self.assertIs(reply["result"], True)
        self.assertEqual(reply["values"]["version"], 3)
        self.assertEqual(self.get(name)["values"], {"value": None, "version": 3})
        # This is valid wire JSON, but recursive Python object copying can
        # exceed its recursion budget before the JSON codec does.
        nested = "leaf"
        for _ in range(600):
            nested = [nested]
        self.set("test/checkpoint/deep", nested)
        reply = self.get("test/checkpoint/deep")
        self.assertIs(reply["result"], True)
        value = reply["values"]["value"]
        for _ in range(600):
            self.assertEqual(len(value), 1)
            value = value[0]
        self.assertEqual(value, "leaf")

    def test_02_parameter_names_are_required_strings(self):
        self.set("test/checkpoint/preserved", "keep")
        for request in ({}, {"name": None}, {"name": 42}, {"name": []}, {"name": {}}):
            with self.subTest(request=request):
                reply = self.client.call(SET, dict(request, value="bad"))
                self.assertIs(reply["result"], False)
                self.assertIsInstance(reply["status"], str)
                self.assertTrue(reply["status"])
        self.assertEqual(self.get("test/checkpoint/preserved")["values"],
                         {"value": "keep", "version": 1})

    def test_03_first_rejection_reports_no_loaded_robot(self):
        self.load("<robot/>", accepted=False, previous=0, root="")

    def test_04_defaults_and_each_joint_type_at_zero(self):
        for kind in ("fixed", "continuous", "revolute", "prismatic"):
            with self.subTest(kind=kind):
                self.subscribe_fk()
                # Limits exclude zero deliberately: FK must not clamp.
                extra = ('<axis/>' if kind in ("revolute", "prismatic") else
                         '<axis xyz="0 0 0"/>' if kind == "fixed" else "")
                description = simple_robot(joint_type=kind, extra=extra)
                description = description.replace("<limit/>", '<limit lower="1" upper="2"/>')
                self.load(description)
                self.assert_tf(self.transforms("/tf", {"tip"})["tip"], "base", (0, 0, 0))
                world = self.transforms("/xform_world", {"base", "tip"})
                self.assert_world(world["base"], (0, 0, 0))
                self.assert_world(world["tip"], (0, 0, 0))

    def test_05_branch_chain_root_discovery_and_rpy_order(self):
        # Expected poses are hand-derived for 90-degree turns and a three-axis origin.
        description = '''<robot>
          <link name="leaf"/><link name="a"/><link name="branch"/>
          <link name="base"/><link name="b"/>
          <joint name="end" type="fixed"><parent link="b"/><child link="leaf"/>
            <origin xyz="0 1 0" rpy="0 1.5707963267948966 0"/></joint>
          <joint name="start" type="revolute"><parent link="base"/><child link="a"/>
            <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/><limit/></joint>
          <joint name="side" type="continuous"><parent link="base"/><child link="branch"/>
            <origin xyz="-2 3 4" rpy="1.5707963267948966 0.5235987755982988 1.0471975511965976"/>
            <axis xyz="1 1 0"/></joint>
          <joint name="middle" type="prismatic"><parent link="a"/><child link="b"/>
            <origin xyz="2 0 0" rpy="1.5707963267948966 0 0"/><limit/><axis xyz="2 0 0"/>
          </joint></robot>'''
        self.subscribe_fk()
        self.load(description)
        rz = ((0, -1, 0), (1, 0, 0), (0, 0, 1))
        rx = ((1, 0, 0), (0, 0, -1), (0, 1, 0))
        ry = ((0, 0, 1), (0, 1, 0), (-1, 0, 0))
        combined = ((math.sqrt(3)/4, 0.25, math.sqrt(3)/2),
                    (0.75, math.sqrt(3)/4, -0.5), (-0.5, math.sqrt(3)/2, 0))
        tf = self.transforms("/tf", {"a", "b", "leaf", "branch"})
        self.assert_tf(tf["a"], "base", (1, 2, 3), rz)
        self.assert_tf(tf["b"], "a", (2, 0, 0), rx)
        self.assert_tf(tf["leaf"], "b", (0, 1, 0), ry)
        self.assert_tf(tf["branch"], "base", (-2, 3, 4), combined)
        world = self.transforms("/xform_world", {"base", "a", "b", "leaf", "branch"})
        self.assert_world(world["base"], (0, 0, 0))
        self.assert_world(world["a"], (1, 2, 3), rz)
        self.assert_world(world["b"], (1, 4, 3), ((0, 0, 1), (1, 0, 0), (0, 1, 0)))
        self.assert_world(world["leaf"], (1, 4, 4), ((-1, 0, 0), (0, 0, 1), (0, 1, 0)))
        self.assert_world(world["branch"], (-2, 3, 4), combined)

    def test_06_real_xml_entities_comments_namespaces_and_ignored_elements(self):
        self.subscribe_fk()
        description = '''<?xml version="1.0" encoding="UTF-8"?>
<?course ignored?>
<robot xmlns="urn:robot" xmlns:xacro="http://example.com/xacro" unknown="ok">
<!-- <joint name="fake" type="floating"><link/> -->
<joint name='joint&amp;&quot;' type='fixed'>
  <parent link='b&#x61;se'/><child link='tip&amp;&quot;&lt;&gt;&apos;'/>
  <origin xyz=' +1\t.25&#10;-2E+0 ' rpy='0 0 0'></origin>
  <axis xyz='0 0 0'/><limit lower='not a number'/>
  <mimic/><dynamics/><safety_controller/>
</joint>
<link name='tip&amp;&quot;&lt;&gt;&apos;'><visual><origin xyz='8 9 10'/>
  <geometry><box size='1 2 3.'/></geometry></visual>
  <collision><origin xyz='bad'/><geometry><capsule/></geometry></collision>
  <inertial><origin xyz='bad'/><mass value='bad'/></inertial>
</link><link name='base'/>
<gazebo><joint name='hollow'/><link/><![CDATA[<unclosed> text & not XML]]></gazebo>
<transmission><joint name='hollow'/></transmission>
</robot>'''.replace("\n", "\r\n")
        self.load(description)
        child = 'tip&"<>\''
        self.assert_tf(self.transforms("/tf", {child})[child], "base", (1, 0.25, -2))
        world = self.transforms("/xform_world", {"base", child})
        self.assert_world(world[child], (1, 0.25, -2))

    def test_07_valid_geometry_defaults_number_spellings_and_leading_space(self):
        shapes = ('<box size="+1 .25 3."/>', '<cylinder radius="1.5e-1" length="-2E+0"/>',
                  '<sphere radius="1e-400"/>', '<mesh filename="package://mesh"/>',
                  '<mesh filename="mesh" scale="1\t2&#13;3"/>')
        for shape in shapes:
            with self.subTest(shape=shape):
                robot = simple_robot(extra='<origin xyz="1e-400 +.25 3."/><axis xyz="0 0 0"/>')
                robot = robot.replace('<link name="tip"/>', '<link name="tip"><visual>'
                                      '<origin rpy="0 -0.5e-400 0"/><geometry>' + shape +
                                      '</geometry></visual></link>')
                self.load(" \r\n\t" + robot)

    def test_08_every_structural_rejection_preserves_previous_robot(self):
        self.subscribe_fk()
        previous = self.load(simple_robot(extra='<origin xyz="4 5 6"/>'))
        bad = [None, 42, [], {}, "", " \r\n\t", "<robot>", "<robot/><robot/>",
               "<robot>&unknown;</robot>", "<robot><link name='base'></robot>",
               "<not_robot><link name='base'/></not_robot>", "<robot/>",
               "<robot><gazebo><link name='base'/></gazebo></robot>",
               "<robot><link/></robot>",
               "<robot><link name='base'/><link name='other'/></robot>"]
        good = simple_robot()
        bad += [good.replace('name="mount" ', ''), good.replace('type="fixed"', ''),
                good.replace('<parent link="base"/>', ''),
                good.replace('<child link="tip"/>', ''),
                good.replace('<parent link="base"/>', '<parent/>'),
                good.replace('<child link="tip"/>', '<child/>'),
                good.replace('link="base"', 'link="missing"'),
                good.replace('link="tip"', 'link="missing"')]
        bad += [simple_robot(joint_type=kind) for kind in ("floating", "planar", "Fixed", "unknown")]
        bad += [simple_robot(joint_type=kind).replace("<limit/>", "")
                for kind in ("revolute", "prismatic")]
        # A cycle has no root here, which is explicitly invalid (not the
        # unspecified single-root-plus-disconnected-cycle case).
        bad.append(good.replace("</robot>", '<joint name="back" type="fixed">'
                               '<parent link="tip"/><child link="base"/></joint></robot>'))
        for description in bad:
            with self.subTest(description=description):
                self.load(description, accepted=False, previous=previous)
        self.assert_tf(self.transforms("/tf", {"tip"})["tip"], "base", (4, 5, 6))
        self.assert_world(self.transforms("/xform_world", {"base", "tip"})["tip"], (4, 5, 6))
        # The parameter server stores malformed URDF unchanged despite rejection.
        self.assertEqual(self.get("robot_description")["values"]["value"], bad[-1])

    def test_09_malformed_numbers_and_first_visual_are_rejected(self):
        previous = self.load(simple_robot())
        extras = [f'<origin {attribute}="{value}"/>'
                  for attribute in ("xyz", "rpy")
                  for value in ("", "1 2", "1 2 3 4", "0,0,1", "1_000 0 0", "x 0 0")]
        extras += ['<axis xyz="1 2"/>', '<axis xyz="0 two 1"/>']
        for extra in extras:
            with self.subTest(extra=extra):
                self.load(simple_robot(extra=extra), accepted=False, previous=previous)
        for kind in ("revolute", "prismatic"):
            for attr in ("lower", "upper"):
                with self.subTest(kind=kind, attr=attr):
                    self.load(simple_robot(joint_type=kind).replace(
                        "<limit/>", f'<limit {attr}="two"/>'), accepted=False, previous=previous)
        visuals = ["", "<geometry/>", "<geometry><capsule/></geometry>",
                   '<geometry><box/></geometry>', '<geometry><box size="1 2"/></geometry>',
                   '<geometry><sphere/></geometry>', '<geometry><sphere radius="x"/></geometry>',
                   '<geometry><cylinder radius="1"/></geometry>',
                   '<geometry><cylinder length="1"/></geometry>',
                   '<geometry><mesh/></geometry>', '<geometry><mesh filename="x" scale="1 2"/></geometry>',
                   '<origin xyz="0 1"/><geometry><sphere radius="1"/></geometry>']
        for visual in visuals:
            with self.subTest(visual=visual):
                description = simple_robot().replace('<link name="tip"/>',
                    '<link name="tip"><visual>' + visual + '</visual></link>')
                self.load(description, accepted=False, previous=previous)

    def test_10_reload_replaces_complete_tree(self):
        self.subscribe_fk()
        self.load(simple_robot(root="old_root", child="old_tip"), root="old_root")
        self.transforms("/xform_world", {"old_root", "old_tip"})
        self.load(simple_robot(root="new_root", child="new_tip",
                               extra='<origin xyz="-3 2 1"/>'), root="new_root")
        self.assert_tf(self.transforms("/tf", {"new_tip"})["new_tip"], "new_root", (-3, 2, 1))
        world = self.transforms("/xform_world", {"new_root", "new_tip"})
        self.assert_world(world["new_root"], (0, 0, 0))
        self.assert_world(world["new_tip"], (-3, 2, 1))

    def test_11_single_link_without_joints_is_valid(self):
        self.client.subscribe("/tf", "tf/tfMessage")
        self.load('<robot><link name="alone"/></robot>', root="alone")
        self.assertEqual(self.transforms("/tf", set()), {})
        # /xform_world behavior for an empty /tf is explicitly unspecified.

    def test_12_unmodified_fetch_robot(self):
        description = (ROOT / "robots" / "fetch" / "fetch.urdf").read_text(encoding="utf-8")
        xml = ET.fromstring(description)
        links = {element.attrib["name"] for element in xml.findall("link")}
        children = {element.find("child").attrib["link"] for element in xml.findall("joint")}
        self.subscribe_fk()
        self.load(description, root="base_link")
        tf = self.transforms("/tf", children)
        world = self.transforms("/xform_world", links)
        self.assert_tf(tf["r_wheel_link"], "base_link", (0.0012914, -0.18738, 0.055325))
        self.assert_world(world["base_link"], (0, 0, 0))
        self.assert_world(world["torso_lift_link"], (-0.086875, 0, 0.37743))
        self.assert_world(world["head_pan_link"], (-0.03375, 0, 0.980431417713939))

    def test_13_large_description_survives_every_hop(self):
        # Fetch in this kit is only 22 KiB. Ignored CDATA makes a legal URDF
        # exercise both the historical 64 KiB pitfall and the 1 MiB contract.
        base = simple_robot(extra='<origin xyz="7 -8 9"/>')
        for payload_size in (100 * 1024, 1024 * 1024 - 4096):
            with self.subTest(payload_size=payload_size):
                description = base.replace("</robot>", "<gazebo><![CDATA[" +
                                           "x" * payload_size + "]]></gazebo></robot>")
                request = {"op": "call_service", "service": SET, "id": "size",
                           "args": {"name": "robot_description", "value": description}}
                self.assertLess(len(json.dumps(request).encode("utf-8")) + 1, 1024 * 1024)
                self.assertGreater(len(description), 65536)
                self.subscribe_fk()
                version = self.load(description)
                self.assertEqual(self.get("robot_description")["values"],
                                 {"value": description, "version": version})
                self.assert_tf(self.transforms("/tf", {"tip"})["tip"], "base", (7, -8, 9))
                self.assert_world(self.transforms("/xform_world", {"base", "tip"})["tip"], (7, -8, 9))


if __name__ == "__main__":
    unittest.main()
