"""Exercise the optional servo, choreography services, and offline example."""

from __future__ import annotations

import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from finite_state_machine import FiniteStateMachine
from joint_state_publisher import JointStatePublisher
from param_server import ParameterServer
from robot_description import parse_robot_description


ROOT = Path(__file__).resolve().parents[1]


def robot(upper=1.0):
    return f'''<robot name="servo"><link name="base"/><link name="turn"/>
    <link name="slide"/><link name="tip"/>
    <joint name="turn" type="continuous"><parent link="base"/><child link="turn"/></joint>
    <joint name="slide" type="prismatic"><parent link="turn"/><child link="slide"/>
    <limit lower="0" upper="{upper}"/></joint>
    <joint name="fixed" type="fixed"><parent link="slide"/><child link="tip"/></joint>
    </robot>'''


def logic():
    return {"initial_state": "one", "states": [
        {"name": "one", "target": {"joint_names": ["turn"], "positions": [1.0]},
         "epsilon": 0.05, "hold_seconds": 0.2, "on_arrival": "advance",
         "transitions": [{"name": "advance", "next_state": "two"}]},
        {"name": "two", "target": {"joint_names": ["turn"], "positions": [-1.0]},
         "epsilon": 0.05, "transitions": [{"name": "back", "next_state": "one"}]},
    ]}


class ServoTests(unittest.TestCase):
    def setUp(self):
        self.parameters = ParameterServer()
        self.publications = []
        handlers = self.parameters.service_handlers()
        self.servo = JointStatePublisher(lambda name, args: handlers[name](args),
                                         lambda topic, msg: self.publications.append((topic, msg)))

    def load(self, description):
        self.parameters.set_param({"name": "robot_description", "value": description})
        self.servo.update(0)

    def trajectory(self, names, positions):
        return {"joint_names": names, "points": [{"positions": [99] * len(names)},
                                                 {"positions": positions}]}

    def test_preload_targets_last_point_limits_and_invalid_reload(self):
        self.servo.accept_trajectory(self.trajectory(["turn", "slide"], [7.0, 5.0]))
        self.servo.update(0.1)
        self.assertEqual(self.publications, [])
        self.load(robot())
        self.servo.update(10.0)
        msg = self.publications[-1][1]
        self.assertEqual(msg["name"], ["turn", "slide"])
        self.assertAlmostEqual(msg["position"][0], 7.0, places=8)
        self.assertAlmostEqual(msg["position"][1], 1.0, places=8)
        self.load("<broken>")
        self.assertEqual(self.servo.robot.name, "servo")
        self.load(robot(0.25))
        self.assertEqual(self.publications[-1][1]["position"][1], 0.25)
        self.assertAlmostEqual(self.publications[-1][1]["position"][0], 7.0, places=8)

    def test_exponential_servo_reset_and_malformed_commands(self):
        self.assertFalse(self.servo.reset({})[0])
        self.load(robot())
        self.assertFalse(self.servo.accept_trajectory(self.trajectory(["turn"], [float("nan")])))
        self.assertFalse(self.servo.accept_trajectory({"joint_names": ["turn"], "points": []}))
        self.servo.accept_trajectory(self.trajectory(["turn", "slide"], [1.0, 1.0]))
        self.servo.update(0.1)
        position = self.publications[-1][1]["position"][0]
        self.assertGreater(position, 0.1)
        self.assertLess(position, 0.3)
        self.assertTrue(self.servo.reset({})[0])
        self.servo.update(0.1)
        self.assertEqual(self.publications[-1][1]["position"], [0.0, 0.5])


class FSMTests(unittest.TestCase):
    def setUp(self):
        # Keep artifacts inside the writable project under Windows sandboxing.
        self.temp = tempfile.TemporaryDirectory(prefix="demo-node-", dir=ROOT / "tests")
        self.addCleanup(self.temp.cleanup)
        self.publications = []
        self.fsm = FiniteStateMachine(lambda *_: None,
                                      lambda topic, msg: self.publications.append((topic, msg)),
                                      self.temp.name, clock=lambda: 0.0)

    def state(self):
        return self.publications[-1][1]["current_state"]

    def test_arrival_hold_manual_transition_and_pause(self):
        self.assertTrue(self.fsm.update_logic(logic())[0])
        self.assertEqual(self.publications[-1][0], "/joint_trajectory")
        self.fsm.update(0.0)
        self.assertEqual(self.state(), "one")  # Unknown joints cannot count as arrived.
        self.fsm.accept_joint_states({"name": ["turn"], "position": [1.01]})
        self.fsm.update(0.1)
        self.fsm.update(0.29)
        self.assertEqual(self.state(), "one")
        self.fsm.update(0.31)
        self.assertEqual(self.state(), "two")
        self.assertEqual(self.publications[-1][1]["last_transition"], "advance")
        self.assertTrue(self.fsm.set_running({"data": False})[0])
        self.assertFalse(self.fsm.fire_transition({"transition": "back"})[0])
        self.assertEqual(self.fsm.set_running({})[1], {"running": False})
        self.fsm.set_running({"data": True})
        self.assertTrue(self.fsm.fire_transition({"transition": "back"})[0])
        self.fsm.update(1.0)
        self.assertEqual(self.state(), "one")

    def test_invalid_logic_is_atomic_and_get_returns_independent_copy(self):
        document = logic()
        self.fsm.update_logic(document)
        invalid = []
        for edit in (lambda d: d.update(initial_state="missing"),
                     lambda d: d["states"].append(copy.deepcopy(d["states"][0])),
                     lambda d: d["states"][0].update(epsilon=-1),
                     lambda d: d["states"][0].update(on_arrival="missing"),
                     lambda d: d["states"][0]["transitions"][0].update(next_state="missing"),
                     lambda d: d["states"][0]["target"].update(positions=[True])):
            candidate = copy.deepcopy(document)
            edit(candidate)
            invalid.append(candidate)
        for candidate in invalid:
            self.assertFalse(self.fsm.update_logic(candidate)[0])
            self.assertEqual(self.fsm.get_logic({})[1], document)
        returned = self.fsm.get_logic({})[1]
        returned["states"].clear()
        self.assertEqual(len(self.fsm.get_logic({})[1]["states"]), 2)

    def test_draft_file_lifecycle_is_safe_and_does_not_deploy(self):
        self.fsm.update_logic(logic())
        for identifier in ("../escape", "", "x/y", "a" * 65, "C:\\escape"):
            self.assertFalse(self.fsm.save_to_file({"identifier": identifier})[0])
            self.assertFalse(self.fsm.load_from_file({"identifier": identifier})[0])
        self.assertFalse(self.fsm.load_from_file({"identifier": "missing"})[0])
        for name in ["zeta", "alpha"]:
            result, _, _ = self.fsm.save_to_file({"identifier": name, "initial_state": "draft", "states": []})
            self.assertTrue(result)
        self.assertEqual(self.fsm.list_saved({})[1]["identifiers"], ["alpha", "zeta"])
        self.assertEqual(self.fsm.load_from_file({"identifier": "alpha"})[1],
                         {"initial_state": "draft", "states": []})
        self.assertEqual(self.fsm.get_logic({})[1], logic())


class ShowcaseTests(unittest.TestCase):
    def test_example_choreography_completes_a_smooth_cycle(self):
        description = (ROOT / "examples" / "showcase.urdf").read_text(encoding="utf-8")
        model = parse_robot_description(description)
        self.assertGreaterEqual(len(model.joints), 10)
        self.assertTrue(all(link.visual is not None for link in model.links.values()))
        document = json.loads((ROOT / "examples" / "choreography.json").read_text())
        parameters = ParameterServer()
        parameters.set_param({"name": "robot_description", "value": description})
        handlers = parameters.service_handlers()
        now = [0.0]
        visited = []
        latest = {}

        def publish(topic, message):
            if topic == "/joint_states":
                fsm.accept_joint_states(message)
                latest.update(zip(message["name"], message["position"]))
            elif topic == "/joint_trajectory":
                servo.accept_trajectory(message)
            elif topic == "/fsm/status":
                state = message["current_state"]
                if not visited or state != visited[-1][1]:
                    visited.append((now[0], state))

        call = lambda name, args: handlers[name](args)
        servo = JointStatePublisher(call, publish)
        fsm = FiniteStateMachine(call, publish, clock=lambda: now[0])
        self.assertTrue(fsm.update_logic(document)[0])
        for index in range(1, 251):
            now[0] = index * 0.1
            servo.update(0.1)
            fsm.update(now[0])
        self.assertGreaterEqual(len(visited), 9)
        self.assertEqual(visited[0][1], visited[8][1])
        self.assertGreater(visited[8][0], 14)
        self.assertLess(visited[8][0], 23)
        for joint in model.joints:
            if joint.type not in ("fixed", "continuous"):
                self.assertGreaterEqual(latest[joint.name], joint.lower)
                self.assertLessEqual(latest[joint.name], joint.upper)


if __name__ == "__main__":
    unittest.main()
