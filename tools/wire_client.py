"""Small external TCP/JSON client shared by the viewer and recorder."""
from __future__ import annotations

from collections import deque
import itertools
import json
import socket
import time


class WireClient:
    def __init__(self, host="127.0.0.1", port=9095, timeout=3.0):
        self.socket = socket.create_connection((host, port), timeout=timeout)
        self.buffer = bytearray()
        self.publications = deque()
        self.ids = itertools.count()

    def close(self):
        self.socket.close()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()

    def send(self, message):
        self.socket.settimeout(3)
        data = json.dumps(message, allow_nan=False, separators=(",", ":"))
        self.socket.sendall((data + "\n").encode("utf-8"))

    def receive(self, timeout=3.0):
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
                raise TimeoutError("no complete rosbridge message before deadline")
            self.socket.settimeout(remaining)
            try:
                data = self.socket.recv(65536)
            except socket.timeout as error:
                raise TimeoutError("no rosbridge traffic before deadline") from error
            if not data:
                raise ConnectionError("rosbridge disconnected")
            self.buffer.extend(data)

    def call(self, service, args, timeout=3.0):
        call_id = f"viewer-{next(self.ids)}"
        self.send({"op": "call_service", "id": call_id, "service": service, "args": args})
        deadline = time.monotonic() + timeout
        while True:
            message = self.receive(max(0, deadline - time.monotonic()))
            if message.get("op") == "service_response" and message.get("id") == call_id:
                return message
            if message.get("op") == "publish":
                self.publications.append(message)

    def subscribe(self, topic, type_name):
        self.send({"op": "subscribe", "topic": topic, "type": type_name})

    def next_publication(self, timeout=1.0):
        deadline = time.monotonic() + timeout
        while True:
            if self.publications:
                return self.publications.popleft()
            message = self.receive(max(0, deadline - time.monotonic()))
            if message.get("op") == "publish":
                return message


TOPICS = {"/xform_world": "xform_world/MatrixTransformArray",
          "/joint_states": "sensor_msgs/JointState", "/fsm/status": "std_msgs/String"}
