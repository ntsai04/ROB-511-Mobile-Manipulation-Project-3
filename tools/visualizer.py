"""Offline HTTP bridge to the unchanged TCP runtime. No third-party packages."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import socket
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
LOCK = threading.Lock()
EXAMPLE = '''<robot name="Sample arm">
  <link name="base"/><link name="shoulder"/><link name="slider"/><link name="tip"/><link name="branch"/>
  <joint name="turn" type="continuous"><parent link="base"/><child link="shoulder"/>
    <origin xyz="0 0 .4" rpy=".2 -.1 .3"/><axis xyz="0 0 2"/></joint>
  <joint name="slide" type="prismatic"><parent link="shoulder"/><child link="slider"/>
    <origin xyz=".5 0 .2" rpy="0 0 .7"/><axis xyz="2 0 0"/><limit lower="-.2" upper=".8"/></joint>
  <joint name="tool" type="fixed"><parent link="slider"/><child link="tip"/><origin xyz=".35 0 0" rpy="0 .5 0"/></joint>
  <joint name="side" type="fixed"><parent link="base"/><child link="branch"/><origin xyz="0 -.5 .2" rpy=".4 .2 -.3"/></joint>
</robot>'''


class Client:
    def __init__(self):
        self.socket = socket.create_connection((OPTIONS.runtime_host, OPTIONS.runtime_port), timeout=3)
        self.buffer = b""
        self.pending = []
        self.counter = 0

    def close(self):
        self.socket.close()

    def send(self, message):
        self.socket.sendall(json.dumps(message, allow_nan=False).encode() + b"\n")

    def wait(self, match, timeout=4):
        for i, message in enumerate(self.pending):
            if match(message):
                return self.pending.pop(i)
        deadline = time.monotonic() + timeout
        while True:
            while b"\n" not in self.buffer:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("The robot pose is unavailable")
                self.socket.settimeout(remaining)
                data = self.socket.recv(65536)
                if not data:
                    raise ConnectionError("The runtime closed its connection")
                self.buffer += data
            line, self.buffer = self.buffer.split(b"\n", 1)
            message = json.loads(line)
            if match(message):
                return message
            self.pending.append(message)
            del self.pending[:-200]

    def call(self, service, args):
        self.counter += 1
        request_id = str(self.counter)
        self.send({"op": "call_service", "service": service, "id": request_id, "args": args})
        reply = self.wait(lambda m: m.get("op") == "service_response" and m.get("id") == request_id)
        if not reply.get("result"):
            raise ValueError(reply.get("status", "Runtime service rejected the request"))
        return reply["values"]

    def subscribe(self, topic):
        self.counter += 1
        request_id = "sub-" + str(self.counter)
        self.send({"op": "subscribe", "topic": topic, "type": "visualizer/Message", "id": request_id})
        self.wait(lambda m: m.get("op") == "status" and m.get("id") == request_id)


def evaluate(body):
    urdf = body["urdf"]
    joints = body.get("joints", {})
    xyz, rpy = body.get("xyz", [0, 0, 0]), body.get("rpy", [0, 0, 0])
    if not isinstance(urdf, str) or len(urdf) > 2_000_000 or not isinstance(joints, dict):
        raise ValueError("Supply URDF text and a joint-name to position object")
    if len(xyz) != 3 or len(rpy) != 3:
        raise ValueError("Global translation and RPY must have three numbers")
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in list(joints.values())+xyz+rpy):
        raise ValueError("Joint positions and global pose must contain finite numbers")
    r, p, y = [a/2 for a in rpy]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    quaternion = (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
    client = Client()
    try:
        client.subscribe("/tf"); client.subscribe("/xform_world")
        version = client.call("/param_server/set_param", {"name": "robot_description", "value": urdf})["version"]
        deadline = time.monotonic()+4
        while True:
            try:
                status = client.call("/param_server/get_param", {"name": "/robot_state_publisher/description_status"})["value"]
            except ValueError:
                status = None
            if status and status.get("version") == version:
                if not status.get("accepted"):
                    raise ValueError("The robot description could not be loaded")
                break
            if time.monotonic() > deadline:
                raise TimeoutError("The robot pose is unavailable")
            time.sleep(.04)
        # A fresh stamp distinguishes this request from queued periodic snapshots.
        tick = time.time_ns()
        stamp = {"sec": tick//1_000_000_000, "nanosec": tick%1_000_000_000}
        client.send({"op": "publish", "topic": "/global_pose", "msg": {"position": dict(zip("xyz", xyz)), "orientation": dict(zip("xyzw", quaternion))}})
        client.send({"op": "publish", "topic": "/joint_states", "msg": {"header": {"stamp": stamp}, "name": list(joints), "position": list(joints.values())}})
        def current(topic):
            return lambda m: m.get("op") == "publish" and m.get("topic") == topic and bool(m.get("msg", {}).get("transforms")) and m["msg"]["transforms"][0]["header"]["stamp"] == stamp
        # An empty /tf for a one-link robot cannot carry a source stamp.
        tf = client.wait(lambda m: m.get("op") == "publish" and m.get("topic") == "/tf" and (not m.get("msg", {}).get("transforms") or current("/tf")(m)))["msg"]
        world = client.wait(current("/xform_world"))["msg"] if tf.get("transforms") else {"transforms": []}
        return {"world": world}
    finally:
        client.close()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def json(self, value, status=200):
        data = json.dumps(value, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers(); self.wfile.write(data)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/example":
            return self.json({"urdf": EXAMPLE})
        if path == "/api/health":
            return self.json({"ok": True, "runtime": "%s:%s" % (OPTIONS.runtime_host, OPTIONS.runtime_port)})
        files = {"/": ("index.html", "text/html"), "/viewer.js": ("viewer.js", "text/javascript"), "/style.css": ("style.css", "text/css")}
        if path not in files:
            return self.send_error(404)
        name, mime = files[path]
        data = (ROOT/"viewer"/name).read_bytes()
        self.send_response(200); self.send_header("Content-Type", mime+"; charset=utf-8")
        self.send_header("Content-Length", str(len(data))); self.end_headers(); self.wfile.write(data)

    def do_POST(self):
        if self.path != "/api/evaluate":
            return self.send_error(404)
        try:
            length = int(self.headers.get("Content-Length", 0))
            if not 0 < length <= 2_100_000:
                raise ValueError("Request must contain at most 2 MB of JSON")
            body = json.loads(self.rfile.read(length))
            with LOCK:
                result = evaluate(body)
            self.json(result)
        except (OSError, TimeoutError):
            self.json({"error": "The robot service is unavailable. Start make run in another terminal."}, 503)
        except (KeyError, TypeError, ValueError):
            self.json({"error": "Unable to load this configuration. Check the robot description and input values."}, 400)


def main():
    global OPTIONS
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8003)
    parser.add_argument("--runtime-host", default="127.0.0.1")
    parser.add_argument("--runtime-port", type=int, default=9095)
    OPTIONS = parser.parse_args()
    server = ThreadingHTTPServer((OPTIONS.host, OPTIONS.port), Handler)
    print("Project 3 visualizer: http://localhost:%d (start make run separately)" % OPTIONS.port, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
