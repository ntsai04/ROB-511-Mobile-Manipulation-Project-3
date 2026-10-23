"""Optional demo node: move named joints toward held trajectory targets.

HOW TO READ THIS FILE
---------------------
This node fills the control role of Project 2's ArmSimNode/PIDController,
using a position servo instead of simulated arm dynamics:

    /joint_trajectory -> accept_trajectory() -> _targets
    robot_description -> update() -> robot and _positions
    update(dt) -> advance movable joints -> publish /joint_states

_positions contains only movable joints in the current robot. _targets
remembers commands by name, even before those joints appear in a robot.
This allows a trajectory to arrive before its description and lets shared
joints keep their commands across reloads.

This node applies servo limits. The FK node still uses external joint states
exactly as given. reset() changes both positions and targets so an old target
cannot pull reset joints back on the next tick.

THREAD OWNERSHIP
----------------
One periodic thread calls update(). TCP callbacks may call accept_trajectory()
or reset() concurrently. The lock protects the loaded robot, positions,
targets, and simulated time. Parameter polling/XML parsing happen outside the
lock. Publication uses a copied snapshot after releasing the lock.
"""

from __future__ import annotations

import math
import threading
from typing import Callable

from robot_description import Joint, Robot, RobotDescriptionError, parse_robot_description


def _number(value: object) -> bool:
    """Accept finite JSON numbers, excluding Python's integer-like Booleans."""
    if isinstance(value, bool):
        return False
    if not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _clamp(joint: Joint, value: float) -> float:
    """Continuous joints have no bounds; other movable joints use URDF limits."""
    if joint.type == "continuous":
        return value
    return max(joint.lower, min(joint.upper, value))


class JointStatePublisher:
    """Hold servo state and publish one complete JointState per update call."""

    def __init__(self, call_service: Callable, publish: Callable, gain: float = 2.5) -> None:
        if not _number(gain) or gain <= 0:
            raise ValueError("servo gain must be finite and positive")
        self._call_service = call_service
        self._publish = publish
        self._gain = float(gain)
        self._lock = threading.RLock()
        self.robot: Robot | None = None
        self._processed_version = 0
        self._positions: dict[str, float] = {}
        self._targets: dict[str, float] = {}
        self._time = 0.0

    def service_handlers(self) -> dict:
        """Keep each public service and its method visible, as in Project 2."""
        return {
            "/joint_state_publisher/reset": self.reset,
        }

    def accept_trajectory(self, message: object) -> bool:
        """Validate the whole command, then retain only its final setpoint."""
        if not isinstance(message, dict):
            return False
        names = message.get("joint_names")
        if not isinstance(names, list):
            return False
        for name in names:
            if not isinstance(name, str):
                return False

        points = message.get("points")
        if not isinstance(points, list) or not points:
            return False
        final_point = points[-1]
        if not isinstance(final_point, dict):
            return False
        requested_positions = final_point.get("positions")
        if not isinstance(requested_positions, list):
            return False
        if len(names) != len(requested_positions):
            return False
        for position in requested_positions:
            if not _number(position):
                return False

        # Unnamed joints retain their targets; duplicate names use the last
        # supplied value. Names absent from the current robot are remembered.
        with self._lock:
            for index in range(len(names)):
                joint_name = names[index]
                self._targets[joint_name] = float(requested_positions[index])
        return True

    def reset(self, _args: object) -> tuple[bool, dict, str]:
        """Put each current movable joint at its default position immediately."""
        with self._lock:
            if self.robot is None:
                return False, {}, "no robot description is loaded"
            for joint in self.robot.joints:
                if joint.type == "fixed":
                    continue
                if joint.type == "continuous":
                    reset_position = 0.0
                else:
                    # Divide before adding to avoid overflow for large bounds.
                    reset_position = joint.lower / 2 + joint.upper / 2
                self._positions[joint.name] = reset_position
                self._targets[joint.name] = reset_position
            return True, {"positions": dict(self._positions)}, ""

    def update(self, dt: float = 0.1) -> None:
        """Poll the robot, advance by dt seconds, and publish a joint snapshot."""
        if not _number(dt) or dt < 0:
            raise ValueError("dt must be finite and nonnegative")

        # 1. Poll/parse outside the lock. A malformed new version leaves the
        # loaded robot and servo state intact. Only update() owns version reads.
        found, values, _status = self._call_service(
            "/param_server/get_param", {"name": "robot_description"},
        )
        version = values.get("version", 0) if found else 0
        candidate_robot = None
        if version > self._processed_version:
            try:
                candidate_robot = parse_robot_description(values.get("value"))
            except RobotDescriptionError:
                pass

        with self._lock:
            # 2. Install a complete valid robot. Rejected versions are marked
            # processed too, so the same bad XML is not parsed every tick.
            if version > self._processed_version:
                self._processed_version = version
                if candidate_robot is not None:
                    retained_positions = {}
                    for joint in candidate_robot.joints:
                        if joint.type == "fixed":
                            continue
                        previous_position = self._positions.get(joint.name, 0.0)
                        retained_positions[joint.name] = _clamp(joint, previous_position)
                    self.robot = candidate_robot
                    self._positions = retained_positions

            # Simulated time continues across reloads, even before a robot
            # loads. dt=0 can load/publish without advancing any joint.
            self._time += dt
            if self.robot is None:
                return

            # 3. Solve dq/dt = gain * (target - q) over this timestep:
            # q_new = q_old + (1 - exp(-gain*dt)) * (target - q_old).
            # The fraction stays in [0, 1], avoiding overshoot on a delayed
            # tick. expm1(x) accurately computes exp(x)-1 for very small dt.
            moved_fraction = -math.expm1(-self._gain * dt)
            names = []
            positions = []
            velocities = []
            for joint in self.robot.joints:
                if joint.type == "fixed":
                    continue
                previous_position = self._positions[joint.name]
                requested_target = self._targets.get(joint.name, previous_position)
                bounded_target = _clamp(joint, requested_target)
                position_error = bounded_target - previous_position
                next_position = _clamp(joint, previous_position + moved_fraction * position_error)
                self._positions[joint.name] = next_position
                if dt > 0:
                    average_velocity = (next_position - previous_position) / dt
                else:
                    average_velocity = 0.0
                names.append(joint.name)
                positions.append(next_position)
                velocities.append(average_velocity)

            # 4. Copy a complete message with one simulated timestamp. Later
            # callbacks edit dictionaries, never these outgoing message lists.
            seconds = int(self._time)
            nanoseconds = int((self._time - seconds) * 1_000_000_000)
            stamp = {"sec": seconds, "nanosec": nanoseconds}
            message = {
                "header": {"stamp": stamp, "frame_id": ""},
                "name": names,
                "position": positions,
                "velocity": velocities,
                "effort": [],
            }

        # Local publication may synchronously invoke other nodes. Release our
        # state lock first so their callbacks cannot hold up servo commands.
        self._publish("/joint_states", message)
