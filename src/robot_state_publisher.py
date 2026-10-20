"""Load descriptions transactionally and publish their zero-configuration edges.

The checkpoint intentionally has no joint-state input. Every edge is T_origin;
the unit-quaternion math and robot model can also represent later joint motion.
"""

from __future__ import annotations

from typing import Callable

from robot_description import Robot, RobotDescriptionError, parse_robot_description


DESCRIPTION_STATUS = "/robot_state_publisher/description_status"


def zero_configuration(robot: Robot) -> dict:
    entries = []
    for joint in robot.joints:
        x, y, z = joint.origin_xyz
        qx, qy, qz, qw = joint.origin_quaternion
        entries.append({
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": joint.parent},
            "child_frame_id": joint.child,
            "transform": {
                "translation": {"x": x, "y": y, "z": z},
                "rotation": {"x": qx, "y": qy, "z": qz, "w": qw},
            },
        })
    return {"transforms": entries}


class RobotStatePublisher:
    def __init__(self, call_service: Callable, publish: Callable) -> None:
        self._call_service = call_service
        self._publish = publish
        self.robot: Robot | None = None
        self._processed_version = 0
        self._loaded_version = 0
        self._message: dict | None = None

    def update(self) -> None:
        """Poll through get_param; validate before replacing any live state.

        The runtime calls this and publishes on a single thread. There can be
        no old /tf publication racing a successful description status.
        """
        result, values, _ = self._call_service(
            "/param_server/get_param", {"name": "robot_description"},
        )
        if result and values["version"] > self._processed_version:
            version = values["version"]
            error = ""
            try:
                candidate = parse_robot_description(values["value"])
                candidate_message = zero_configuration(candidate)
            except RobotDescriptionError as failure:
                error = str(failure)
            else:
                self.robot = candidate
                self._message = candidate_message
                self._loaded_version = version
                # Synchronous local subscribers (including the world node)
                # see the replacement before its acceptance is reported.
                self._publish("/tf", self._message)
            self._processed_version = version
            self._call_service("/param_server/set_param", {
                "name": DESCRIPTION_STATUS,
                "value": {
                    "version": version,
                    "accepted": not error,
                    "error": error,
                    "loaded_version": self._loaded_version,
                    "root_link": self.robot.root_link if self.robot else "",
                },
            })
        if self._message is not None:
            self._publish("/tf", self._message)
