"""A small client for the course rosbridge protocol, for your Python tests.

    from rosbridge_client import Rosbridge

    with Rosbridge() as ros:                      # connects to 127.0.0.1:9095
        reply = ros.call_service("/heap_sort", {"numbers": [3, 1, 2]})
        assert reply["result"] is True, reply
        assert reply["values"]["sorted"] == [1, 2, 3], reply

Messages are plain dicts, one JSON object per line, exactly as in
ROSBRIDGE_PROTOCOL.md. Every wait has a timeout and raises TimeoutError
instead of hanging. Standard library only. This file is course-supplied;
you may change it.
"""

from __future__ import annotations

import itertools
import json
import socket
import time
from typing import Any, Callable

_ids = itertools.count(1)


class Rosbridge:
    def __init__(self, host: str = "127.0.0.1", port: int = 9095, timeout: float = 5.0) -> None:
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.buffer = b""
        # Messages read while waiting for something else, kept for later waits.
        self.pending: list[dict] = []

    def __enter__(self) -> "Rosbridge":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def close(self) -> None:
        self.sock.close()

    def send(self, message: dict) -> None:
        self.sock.sendall(json.dumps(message).encode("utf-8") + b"\n")

    def _read(self, deadline: float) -> dict:
        """The next message from the gateway, waiting at most until deadline."""
        while b"\n" not in self.buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("no message from the gateway in time")
            self.sock.settimeout(remaining)
            try:
                chunk = self.sock.recv(65536)
            except socket.timeout:
                raise TimeoutError("no message from the gateway in time") from None
            if not chunk:
                raise ConnectionError("the gateway closed the connection")
            self.buffer += chunk
        line, self.buffer = self.buffer.split(b"\n", 1)
        return json.loads(line)

    def wait_for(self, match: Callable[[dict], bool], timeout: float = 5.0) -> dict:
        """The first message (already received or arriving within timeout) for which match is true."""
        for index, message in enumerate(self.pending):
            if match(message):
                return self.pending.pop(index)
        deadline = time.monotonic() + timeout
        while True:
            message = self._read(deadline)
            if match(message):
                return message
            self.pending.append(message)
            del self.pending[:-1000]  # busy topics: keep only the newest

    def call_service(self, service: str, args: Any, timeout: float = 10.0) -> dict:
        """Call a service and return its whole service_response message."""
        call_id = f"call-{next(_ids)}"
        self.send({"op": "call_service", "service": service, "id": call_id, "args": args})
        return self.wait_for(
            lambda m: m.get("op") == "service_response" and m.get("id") == call_id, timeout)

    def advertise(self, topic: str, msg_type: str) -> None:
        self.send({"op": "advertise", "topic": topic, "type": msg_type})

    def publish(self, topic: str, msg: Any) -> None:
        self.send({"op": "publish", "topic": topic, "msg": msg})

    def subscribe(self, topic: str, msg_type: str, timeout: float = 5.0) -> None:
        """Subscribe and wait for the gateway's status saying it is registered."""
        sub_id = f"sub-{next(_ids)}"
        self.send({"op": "subscribe", "topic": topic, "type": msg_type, "id": sub_id})
        self.wait_for(lambda m: m.get("op") == "status" and m.get("id") == sub_id, timeout)

    def next_message(self, topic: str, timeout: float = 5.0,
                     match: Callable[[Any], bool] = lambda msg: True) -> Any:
        """The msg of the next publication on a subscribed topic for which match is true."""
        return self.wait_for(
            lambda m: m.get("op") == "publish" and m.get("topic") == topic and match(m.get("msg")),
            timeout)["msg"]
