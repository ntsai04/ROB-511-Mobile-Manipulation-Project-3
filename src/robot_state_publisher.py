"""Robot state publisher: URDF + joint positions -> one parent/child edge per joint.

HOW TO READ THIS FILE
---------------------
The node has the same callback/update split as Project 2's simulation nodes:

    accept_joint_states()   validate and remember the newest input
    update()                read a description, compute /tf, publish a snapshot

Only update() publishes /tf. It runs on main.py's single periodic thread.
Socket-reader threads may call accept_joint_states() at the same time, so
positions and their stamp are always read or replaced under one lock.

Follow one configuration through the file:
1. _process_description() polls the parameter service and validates a candidate.
2. _replace_robot() installs a fully validated model in one critical section.
3. joint_configuration() turns every joint's current q into T_origin*T_motion.
4. update() publishes the new /tf BEFORE reporting acceptance in the parameters.

The two version counters mean different things. _processed_version includes
rejected descriptions; _loaded_version names the last accepted description.
A failed load changes the former counter but preserves the robot and q state.
"""

from __future__ import annotations

import math
import threading
from typing import Callable

from robot_description import Robot, RobotDescriptionError, parse_robot_description


DESCRIPTION_STATUS = "/robot_state_publisher/description_status"


def joint_configuration(robot: Robot, positions: dict[str, float], stamp: dict) -> dict:
    """Build a COMPLETE /tf snapshot; these are local edges, not world poses."""
    transforms = []
    for joint in robot.joints:
        # A JointState message replaces the whole state. Consequently a name
        # absent from this dictionary means q=0, not 'keep its previous q'.
        position = positions.get(joint.name, 0.0)
        translation, quaternion = joint.joint_transform(position)
        x, y, z = translation
        qx, qy, qz, qw = quaternion

        # Include fixed joints too. Their motion is identity, but their origin
        # may still place a child frame somewhere different from its parent.
        transforms.append({
            "header": {"stamp": dict(stamp), "frame_id": joint.parent},
            "child_frame_id": joint.child,
            "transform": {
                "translation": {"x": x, "y": y, "z": z},
                "rotation": {"x": qx, "y": qy, "z": qz, "w": qw},
            },
        })
    return {"transforms": transforms}


def zero_configuration(robot: Robot) -> dict:
    """At the checkpoint q=0, so every joint transform is just its origin."""
    return joint_configuration(robot, {}, {"sec": 0, "nanosec": 0})


class RobotStatePublisher:
    def __init__(self, call_service: Callable, publish: Callable) -> None:
        # These are ordinary functions supplied by main.py. As in Project 2,
        # the node does domain work while RosbridgeServer handles transport.
        self._call_service = call_service
        self._publish = publish
        self.robot: Robot | None = None
        self._processed_version = 0
        self._loaded_version = 0
        self._positions: dict[str, float] = {}
        self._stamp = {"sec": 0, "nanosec": 0}
        self._movable_names: set[str] = set()
        self._lock = threading.Lock()

    def accept_joint_states(self, message: object) -> None:
        """Replace the latest positions and their original stamp as ONE snapshot."""
        try:
            names = message["name"]
            positions = message["position"]
            received_stamp = message["header"]["stamp"]
            stamp = {"sec": received_stamp["sec"], "nanosec": received_stamp["nanosec"]}

            if not isinstance(names, list) or not isinstance(positions, list):
                raise ValueError("joint names and positions must be arrays")
            if len(names) != len(positions):
                raise ValueError("joint names and positions differ in length")
            for name in names:
                if not isinstance(name, str):
                    raise ValueError("joint names must be strings")
            for position in positions:
                if isinstance(position, bool) or not isinstance(position, (int, float)):
                    raise ValueError("joint positions must be numbers")
                if not math.isfinite(position):
                    raise ValueError("joint positions must be finite")
            for component in stamp.values():
                if isinstance(component, bool) or not isinstance(component, int):
                    raise ValueError("stamp components must be integers")
        except (KeyError, TypeError, ValueError, OverflowError):
            # Bad input must not partially overwrite a valid joint state.
            return

        with self._lock:
            replacement = {}
            for name, position in zip(names, positions):
                # Ignore fixed and unknown names, but apply the rest of the
                # message. This is filtering, not a reason to reject it all.
                if name in self._movable_names:
                    replacement[name] = position
            self._positions = replacement
            self._stamp = stamp
            # No wrapping, limit clamping or wall-clock substitution belongs
            # in FK. The optional joint servo performs its own limit clamping.

    def _replace_robot(self, robot: Robot, version: int) -> None:
        """Install a model only after the complete candidate has passed validation."""
        movable_names = set()
        for joint in robot.joints:
            if joint.type != "fixed":
                movable_names.add(joint.name)

        with self._lock:
            # Reload carryover is unspecified by the course. We keep positions
            # for shared movable names and let newly introduced names use 0.
            retained_positions = {}
            for name, position in self._positions.items():
                if name in movable_names:
                    retained_positions[name] = position
            self.robot = robot
            self._movable_names = movable_names
            self._positions = retained_positions
            self._loaded_version = version

    def _process_description(self) -> dict | None:
        """Process a new parameter version, returning the status to report later."""
        result, values, status = self._call_service(
            "/param_server/get_param", {"name": "robot_description"},
        )
        if not result:
            return None
        version = values["version"]
        if version <= self._processed_version:
            return None

        error = ""
        try:
            candidate = parse_robot_description(values["value"])
        except RobotDescriptionError as failure:
            error = str(failure)
        else:
            self._replace_robot(candidate, version)

        # A rejected version is still processed. Otherwise each tick would
        # parse the same broken XML again instead of waiting for a new set.
        self._processed_version = version
        root_link = ""
        if self.robot is not None:
            root_link = self.robot.root_link
        return {
            "version": version,
            "accepted": error == "",
            "error": error,
            "loaded_version": self._loaded_version,
            "root_link": root_link,
        }

    def update(self) -> None:
        """Process description -> snapshot state -> publish edges -> report status."""
        description_status = self._process_description()

        # Build under the lock so a message cannot combine an old q array
        # with a new stamp. Release it before calling another node or gateway.
        with self._lock:
            message = None
            if self.robot is not None:
                message = joint_configuration(self.robot, self._positions, self._stamp)

        if message is not None:
            self._publish("/tf", message)

        # Acceptance is observable only after /tf has the new robot. The local
        # world subscriber runs synchronously during publish(), so its cached
        # output is also ready. There is no second thread publishing old /tf.
        if description_status is not None:
            self._call_service("/param_server/set_param", {
                "name": DESCRIPTION_STATUS,
                "value": description_status,
            })
