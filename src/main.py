"""Compose the Project 3 nodes using the same local-node pattern as Project 2.

HOW TO READ THIS FILE
---------------------
Compare this file with Project 2's src/main.py:

    construct RosbridgeServer and domain nodes
    register each node's service table and topic callbacks
    run one periodic update thread
    accept TCP clients, each with a reader thread and a writer thread

The gateway knows JSON envelopes and delivery. The node classes know robot
parameters, joint states, transforms and choreography. No node owns a socket.
The injected call_service/publish functions are ordinary Python callbacks,
not a second network protocol or a hidden middleware implementation.

The default mode starts only the three graded nodes. --demo also starts the
joint servo and FSM and loads the sample files. Keep those two paths separate:
grading supplies its own descriptions, joint states and global poses.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import sys
import threading
import time

from param_server import ParameterServer
from robot_state_publisher import RobotStatePublisher
from robot_world_state_publisher import RobotWorldStatePublisher
from rosbridge_server import HOST, PORT, RosbridgeServer, serve_client


PUBLISH_PERIOD_SECONDS = 0.1  # 10 Hz; the wire contract requires at least 5 Hz.


def _register_services(server: RosbridgeServer, node: object) -> None:
    """Expose a node's service table, exactly as Project 2's main.py does."""
    for name, handler in node.service_handlers().items():
        # Nodes accept args and return (result, values, status). The gateway's
        # handler interface also supplies the service name, which we ignore.
        # handler=handler captures THIS iteration's method. Without that
        # default argument, every lambda would use the final loop variable.
        server.register_local_service(name, lambda _name, args, handler=handler: handler(args))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true", help="run the bundled robot choreography")
    options = parser.parse_args()

    server = RosbridgeServer()
    parameters = ParameterServer()
    _register_services(server, parameters)

    def call_service(name: str, args: dict) -> tuple:
        # Local calls and external TCP calls reach the same registered method.
        # In particular RSP reads descriptions through get_param, rather than
        # reaching into ParameterServer's private dictionary.
        handler = server.local_services[name]
        return handler(name, args)

    robot_state = RobotStatePublisher(call_service, server.publish_local)
    world_state = RobotWorldStatePublisher(server.publish_local)
    server.register_topic_listener("/joint_states", robot_state.accept_joint_states)
    server.register_topic_listener("/tf", world_state.accept_tf)
    server.register_topic_listener("/global_pose", world_state.accept_global_pose)

    joint_servo = None
    state_machine = None
    if options.demo:
        from finite_state_machine import FiniteStateMachine
        from joint_state_publisher import JointStatePublisher

        joint_servo = JointStatePublisher(call_service, server.publish_local)
        state_machine = FiniteStateMachine(call_service, server.publish_local)
        _register_services(server, joint_servo)
        _register_services(server, state_machine)
        server.register_topic_listener("/joint_trajectory", joint_servo.accept_trajectory)
        server.register_topic_listener("/joint_states", state_machine.accept_joint_states)

        project_root = Path(__file__).resolve().parents[1]
        description_path = project_root / "examples/showcase.urdf"
        choreography_path = project_root / "examples/choreography.json"
        description = description_path.read_text(encoding="utf-8")
        choreography = json.loads(choreography_path.read_text(encoding="utf-8"))

        # All listeners are installed before the FSM enters its first state.
        # Its first target can precede the servo loading the robot; the servo
        # remembers setpoints by name and applies them once that model arrives.
        call_service("/param_server/set_param", {"name": "robot_description", "value": description})
        result, values, status = call_service("/fsm/update_logic", choreography)
        if not result:
            print(f"unable to load demo choreography: {status}", file=sys.stderr)
            return 1

    stopped = threading.Event()

    def publish_loop() -> None:
        """Update domain state in pipeline order, then wait for the next tick."""
        # Give the very first servo step the same dt as an ordinary tick.
        previous_update = time.monotonic() - PUBLISH_PERIOD_SECONDS
        while not stopped.is_set():
            update_started = time.monotonic()
            elapsed_seconds = update_started - previous_update

            # Local publish() delivers callbacks synchronously. Therefore:
            # servo -> joint_states -> RSP input
            # RSP   -> tf           -> world publisher input
            # world -> xform_world  -> external viewer
            # FSM   -> trajectory   -> next servo target
            if joint_servo is not None:
                joint_servo.update(elapsed_seconds)
            robot_state.update()
            world_state.update()
            if state_machine is not None:
                state_machine.update(update_started)

            previous_update = update_started
            update_duration = time.monotonic() - update_started
            remaining_seconds = max(0.0, PUBLISH_PERIOD_SECONDS - update_duration)
            # Event.wait yields like sleep, but wakes immediately on shutdown.
            # Subtracting computation time avoids adding it to every period.
            stopped.wait(remaining_seconds)

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            # This accept loop and per-client serve_client thread are the same
            # design as Projects 1/2. SO_REUSEADDR permits quick restarts.
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((HOST, PORT))
            listener.listen()
            threading.Thread(target=publish_loop, daemon=True).start()
            threading.Thread(target=server.expire_calls_forever, daemon=True).start()
            mode = "demo" if options.demo else "runtime"
            print(f"Project 3 {mode} listening on {HOST}:{PORT}", file=sys.stderr, flush=True)
            while True:
                connection, address = listener.accept()
                threading.Thread(target=serve_client, args=(server, connection), daemon=True).start()
    except KeyboardInterrupt:
        return 0
    except OSError as error:
        print(f"unable to start gateway: {error}", file=sys.stderr, flush=True)
        return 1
    finally:
        stopped.set()


if __name__ == "__main__":
    raise SystemExit(main())
