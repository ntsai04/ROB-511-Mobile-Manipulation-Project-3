"""Optional choreography node: enter a pose, wait for arrival, take a transition.

HOW TO READ THIS FILE
---------------------
validate_logic() checks a complete candidate before live state changes.
update_logic() installs the copied table, starts it, and enters initial_state.
_enter() publishes that state's /joint_trajectory target. Joint feedback arrives
through accept_joint_states(); update() checks arrival, applies an optional
hold, advances at most one transition, and publishes /fsm/status.

State names identify poses. Transition names identify outgoing edges within
one state. on_arrival names an edge, not its destination. _transition() resolves
that edge and enters next_state. fire_transition() uses the same manual path.

DATA OWNERSHIP AND THREADS
-------------------------
_logic is our private copied document. _states indexes the same state objects
by name. _current is a state name. _positions contains only the latest feedback
message. _entered_at and _arrival_since are absolute monotonic times, whereas
hold_seconds is a duration. get_logic() returns a deep copy, so callers cannot
edit the active choreography through a returned dictionary.

The periodic runtime thread and TCP service/topic callbacks share an RLock.
Entry publications happen under that lock, keeping a new state and its command
together. Local gateway callbacks may run synchronously; a reentrant lock lets
a same-thread callback safely inspect this node. Status is published after the
lock is released. Pause stops transitions, not the independent joint servo.

FILES ARE DRAFTS
----------------
Saving/loading a JSON file does not deploy a choreography. Only update_logic()
validates and installs one. Identifiers are restricted to a filename stem in
the configured fsm directory. Saving writes a temporary file before replacing
the final file so a failed write does not truncate an existing draft.
"""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import re
import threading
import time
from typing import Callable


_IDENTIFIER = re.compile(r"[A-Za-z0-9_-]{1,64}")


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


def _name(value: object) -> bool:
    return isinstance(value, str) and bool(value)


def validate_logic(args: object) -> tuple[dict, dict[str, dict]]:
    """Return a copied document and name index only after every rule passes."""
    if not isinstance(args, dict):
        raise ValueError("logic requires at least one state")
    supplied_states = args.get("states")
    if not isinstance(supplied_states, list) or not supplied_states:
        raise ValueError("logic requires at least one state")

    # Keep optional fields as supplied instead of inserting defaults into the
    # document returned by get_logic(). The FSM owns this complete deep copy.
    document = copy.deepcopy({
        "initial_state": args.get("initial_state"),
        "states": supplied_states,
    })
    states_by_name = {}

    # Pass 1 validates local shapes and collects all state names. Transition
    # destinations may refer to later states, so resolve them in pass 2.
    for state in document["states"]:
        if not isinstance(state, dict):
            raise ValueError("state names must be nonempty and unique")
        state_name = state.get("name")
        if not _name(state_name) or state_name in states_by_name:
            raise ValueError("state names must be nonempty and unique")

        target = state.get("target")
        if not isinstance(target, dict):
            raise ValueError("each state requires a joint target")
        joint_names = target.get("joint_names")
        target_positions = target.get("positions")
        target_error = "target requires equally sized joint_names and finite positions"
        if not isinstance(joint_names, list):
            raise ValueError(target_error)
        seen_joint_names = set()
        for joint_name in joint_names:
            if not _name(joint_name) or joint_name in seen_joint_names:
                raise ValueError(target_error)
            seen_joint_names.add(joint_name)
        if not isinstance(target_positions, list):
            raise ValueError(target_error)
        if len(joint_names) != len(target_positions):
            raise ValueError(target_error)
        for position in target_positions:
            if not _number(position):
                raise ValueError(target_error)

        epsilon = state.get("epsilon", 0.02)
        hold_seconds = state.get("hold_seconds", 0.0)
        if not _number(epsilon) or epsilon < 0:
            raise ValueError("epsilon and hold_seconds must be finite and nonnegative")
        if not _number(hold_seconds) or hold_seconds < 0:
            raise ValueError("epsilon and hold_seconds must be finite and nonnegative")

        transitions = state.get("transitions", [])
        if not isinstance(transitions, list):
            raise ValueError("transitions must be a list")
        transition_names = set()
        for transition in transitions:
            transition_error = "transition names must be nonempty and unique within a state"
            if not isinstance(transition, dict):
                raise ValueError(transition_error)
            transition_name = transition.get("name")
            next_state = transition.get("next_state")
            if not _name(transition_name) or transition_name in transition_names:
                raise ValueError(transition_error)
            if not _name(next_state):
                raise ValueError(transition_error)
            transition_names.add(transition_name)

        if "on_arrival" in state and state["on_arrival"] not in transition_names:
            raise ValueError("on_arrival must name a transition of its state")
        states_by_name[state_name] = state

    # Pass 2 resolves the initial state and every destination globally.
    initial_state = document["initial_state"]
    if not _name(initial_state) or initial_state not in states_by_name:
        raise ValueError("initial_state must name a state")
    for state in states_by_name.values():
        for transition in state.get("transitions", []):
            if transition["next_state"] not in states_by_name:
                raise ValueError("every transition must name an existing next_state")
    return document, states_by_name


class FiniteStateMachine:
    """Own one choreography and advance it from joint feedback and time."""

    def __init__(self, call_service: Callable, publish: Callable,
                 storage_directory: str | Path | None = None,
                 clock: Callable = time.monotonic) -> None:
        # call_service preserves the construction interface of the other nodes;
        # this node only consumes topic feedback and does not need to call it.
        self._publish = publish
        self._clock = clock
        if storage_directory is None:
            self._directory = Path(__file__).resolve().parents[1] / "fsm"
        else:
            self._directory = Path(storage_directory)
        self._lock = threading.RLock()
        self._logic = {"initial_state": None, "states": []}
        self._states: dict[str, dict] = {}
        self._current: str | None = None
        self._positions: dict[str, float] = {}
        self._running = False
        self._entered_at = self._clock()
        self._arrival_since: float | None = None
        self._last_transition: str | None = None

    def service_handlers(self) -> dict:
        """Make every public service and its handler visible in one place."""
        return {
            "/fsm/update_logic": self.update_logic,
            "/fsm/get_logic": self.get_logic,
            "/fsm/set_running": self.set_running,
            "/fsm/fire_transition": self.fire_transition,
            "/fsm/save_to_file": self.save_to_file,
            "/fsm/load_from_file": self.load_from_file,
            "/fsm/list_saved": self.list_saved,
        }

    def accept_joint_states(self, message: object) -> bool:
        """Replace feedback only after the entire incoming message is valid."""
        if not isinstance(message, dict):
            return False
        names = message.get("name")
        positions = message.get("position")
        if not isinstance(names, list) or not isinstance(positions, list):
            return False
        if len(names) != len(positions):
            return False
        for name in names:
            if not isinstance(name, str):
                return False
        for position in positions:
            if not _number(position):
                return False

        latest_positions = {}
        for index in range(len(names)):
            latest_positions[names[index]] = positions[index]
        with self._lock:
            # A joint absent from this message is unknown to the arrival check;
            # it must not retain feedback from an older message.
            self._positions = latest_positions
        return True

    def _enter(self, name: str, now: float) -> None:
        """Enter a validated state and send its target; caller holds the lock."""
        self._current = name
        self._entered_at = now
        self._arrival_since = None
        target = self._states[name]["target"]
        trajectory = {
            "joint_names": list(target["joint_names"]),
            "points": [{
                "positions": list(target["positions"]),
                "velocities": [],
                "accelerations": [],
                "time_from_start": {"sec": 0, "nanosec": 0},
            }],
        }
        self._publish("/joint_trajectory", trajectory)

    def update_logic(self, args: object) -> tuple[bool, dict, str]:
        """Validate before swapping the live table, then start its initial state."""
        try:
            document, states_by_name = validate_logic(args)
        except (ValueError, TypeError, RecursionError) as error:
            return False, {}, str(error)
        with self._lock:
            self._logic = document
            self._states = states_by_name
            self._running = True
            self._last_transition = None
            self._enter(document["initial_state"], self._clock())
            return True, {"current_state": self._current}, ""

    def get_logic(self, _args: object) -> tuple[bool, dict, str]:
        with self._lock:
            return True, copy.deepcopy(self._logic), ""

    def set_running(self, args: object) -> tuple[bool, dict, str]:
        """Query with {}, pause with false, or re-enter the same state on resume."""
        if not isinstance(args, dict):
            return False, {}, "data must be a Boolean, or omitted to query"
        if "data" in args and not isinstance(args["data"], bool):
            return False, {}, "data must be a Boolean, or omitted to query"
        with self._lock:
            if "data" in args:
                requested_running = args["data"]
                if requested_running and self._current is None:
                    return False, {"running": False}, "no choreography is loaded"
                was_running = self._running
                self._running = requested_running
                if requested_running and not was_running:
                    # Resume republishes the current target and restarts entry
                    # and hold timers; it does not jump back to initial_state.
                    self._enter(self._current, self._clock())
            # Pausing stops transitions, not the independent joint servo.
            return True, {"running": self._running}, ""

    def _transition(self, name: str, now: float) -> bool:
        """Find a current state's outgoing edge and enter its destination."""
        current_state = self._states[self._current]
        selected_transition = None
        for transition in current_state.get("transitions", []):
            if transition["name"] == name:
                selected_transition = transition
                break
        if selected_transition is None:
            return False
        self._last_transition = name
        self._enter(selected_transition["next_state"], now)
        return True

    def fire_transition(self, args: object) -> tuple[bool, dict, str]:
        """Take a named current-state transition immediately while running."""
        if not isinstance(args, dict) or not _name(args.get("transition")):
            return False, {}, "transition must be a name"
        with self._lock:
            if not self._running or self._current is None:
                return False, {}, "the choreography is stopped"
            transitioned = self._transition(args["transition"], self._clock())
            if not transitioned:
                return False, {}, "transition does not exist in the current state"
            return True, {"current_state": self._current}, ""

    def _target_reached(self, target: dict, epsilon: float) -> bool:
        """Require feedback within epsilon for every target joint; lock is held."""
        joint_names = target["joint_names"]
        target_positions = target["positions"]
        for index in range(len(joint_names)):
            joint_name = joint_names[index]
            if joint_name not in self._positions:
                return False
            position_error = abs(self._positions[joint_name] - target_positions[index])
            if position_error > epsilon:
                return False
        # An empty target has no joints left to reach, so it is arrived.
        return True

    def update(self, now: float | None = None) -> None:
        """Advance at most one automatic transition, then publish status."""
        current_time = self._clock() if now is None else now
        with self._lock:
            if self._running and self._current is not None:
                current_state = self._states[self._current]
                target = current_state["target"]
                epsilon = current_state.get("epsilon", 0.02)
                arrived = self._target_reached(target, epsilon)
                if not arrived:
                    # Leaving tolerance breaks the hold. It must be continuous
                    # time at the target, as in Project 2's IK action node.
                    self._arrival_since = None
                elif "on_arrival" in current_state:
                    if self._arrival_since is None:
                        self._arrival_since = current_time
                    arrived_duration = current_time - self._arrival_since
                    required_hold = current_state.get("hold_seconds", 0.0)
                    if arrived_duration >= required_hold:
                        self._transition(current_state["on_arrival"], current_time)

            if self._current is None:
                time_in_state = 0.0
            else:
                # Displayed elapsed time continues while paused. Resume enters
                # the state again and restarts the entry and hold clocks.
                time_in_state = max(0.0, current_time - self._entered_at)
            status = {
                "current_state": self._current,
                "time_in_state": time_in_state,
                "last_transition": self._last_transition,
                "running": self._running,
            }
        self._publish("/fsm/status", status)

    def _path(self, args: object) -> Path:
        """Build a draft path only from a plain, bounded identifier."""
        identifier = args.get("identifier") if isinstance(args, dict) else None
        if not isinstance(identifier, str) or _IDENTIFIER.fullmatch(identifier) is None:
            raise ValueError("identifier must contain 1-64 letters, digits, underscores or hyphens")
        return self._directory / (identifier + ".json")

    def save_to_file(self, args: object) -> tuple[bool, dict, str]:
        """Save an editable draft without changing the active choreography."""
        try:
            path = self._path(args)
            # Drafts may be incomplete, so skip validate_logic(). Serialization
            # still rejects non-JSON numbers such as NaN and Infinity.
            document = {
                "initial_state": args.get("initial_state"),
                "states": args.get("states", []),
            }
            contents = json.dumps(document, indent=2, allow_nan=False) + "\n"
            with self._lock:
                self._directory.mkdir(parents=True, exist_ok=True)
                temporary_path = path.with_suffix(".json.tmp")
                temporary_path.write_text(contents, encoding="utf-8")
                temporary_path.replace(path)
            return True, {"path": str(path)}, ""
        except (OSError, ValueError, TypeError, RecursionError) as error:
            return False, {}, str(error)

    def load_from_file(self, args: object) -> tuple[bool, dict, str]:
        """Read a saved draft; deploy it separately with update_logic()."""
        try:
            path = self._path(args)
            with self._lock:
                document = json.loads(path.read_text(encoding="utf-8"))
            return True, document, ""
        except (OSError, ValueError, TypeError, RecursionError) as error:
            return False, {}, str(error)

    def list_saved(self, _args: object) -> tuple[bool, dict, str]:
        """Return legal JSON draft identifiers in alphabetical order."""
        try:
            with self._lock:
                identifiers = []
                for path in self._directory.glob("*.json"):
                    if not path.is_file():
                        continue
                    if _IDENTIFIER.fullmatch(path.stem) is None:
                        continue
                    identifiers.append(path.stem)
                identifiers.sort()
            return True, {"identifiers": identifiers}, ""
        except OSError as error:
            return False, {}, str(error)
