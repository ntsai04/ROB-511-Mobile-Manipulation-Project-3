"""Project 3 checkpoint: the reused gateway and three cooperating local nodes."""

from __future__ import annotations

import socket
import sys
import threading
import time

from param_server import ParameterServer
from robot_state_publisher import RobotStatePublisher
from robot_world_state_publisher import RobotWorldStatePublisher
from rosbridge_server import HOST, PORT, RosbridgeServer, serve_client


def main() -> int:
    server = RosbridgeServer()
    parameters = ParameterServer()
    for name, handler in parameters.service_handlers().items():
        server.register_local_service(name, lambda _name, args, handler=handler: handler(args))

    def call_service(name: str, args: dict) -> tuple:
        # In-process calls use exactly the same service handlers as TCP calls,
        # following Project 2's local-node transport interface.
        return server.local_services[name](name, args)

    state = RobotStatePublisher(call_service, server.publish_local)
    world = RobotWorldStatePublisher(server.publish_local)
    server.register_topic_listener("/tf", world.accept_tf)
    stopped = threading.Event()

    def publish_loop() -> None:
        while not stopped.is_set():
            started = time.monotonic()
            state.update()
            world.update()
            stopped.wait(max(0.0, 0.1 - (time.monotonic() - started)))

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((HOST, PORT))
            listener.listen()
            threading.Thread(target=publish_loop, daemon=True).start()
            threading.Thread(target=server.expire_calls_forever, daemon=True).start()
            print(f"Project 3 checkpoint listening on {HOST}:{PORT}", file=sys.stderr, flush=True)
            while True:
                connection, _address = listener.accept()
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
