"""Offline HTTP bridge to the unchanged TCP runtime. No third-party packages."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import socket
import threading
import time
import uuid

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


def configuration(body):
    if not isinstance(body, dict):
        raise ValueError("Supply a configuration object")
    joints = body.get("joints", {})
    xyz, rpy = body.get("xyz", [0, 0, 0]), body.get("rpy", [0, 0, 0])
    if not isinstance(joints, dict) or any(not isinstance(name, str) for name in joints):
        raise ValueError("Supply named joint positions")
    if not isinstance(xyz, list) or not isinstance(rpy, list) or len(xyz) != 3 or len(rpy) != 3:
        raise ValueError("Global translation and RPY must contain three numbers")
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
               for v in list(joints.values()) + xyz + rpy):
        raise ValueError("Use finite joint positions and global pose values")
    return {"joints": dict(joints), "xyz": list(xyz), "rpy": list(rpy)}


def load_description(client, urdf):
    if not isinstance(urdf, str) or len(urdf) > 2_000_000:
        raise ValueError("Supply URDF text")
    version = client.call("/param_server/set_param", {"name": "robot_description", "value": urdf})["version"]
    deadline = time.monotonic() + 4
    while True:
        try:
            status = client.call("/param_server/get_param", {"name": "/robot_state_publisher/description_status"})["value"]
        except ValueError:
            status = None
        if status and status.get("version") == version:
            if not status.get("accepted"):
                raise ValueError("The robot description could not be loaded")
            return
        if time.monotonic() > deadline:
            raise TimeoutError("The robot pose is unavailable")
        time.sleep(.04)


def publish_pose(client, body):
    body = configuration(body)
    joints, xyz, rpy = body["joints"], body["xyz"], body["rpy"]
    r, p, y = [a / 2 for a in rpy]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    quaternion = (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
    tick = time.time_ns()
    stamp = {"sec": tick//1_000_000_000, "nanosec": tick%1_000_000_000}
    client.send({"op": "publish", "topic": "/global_pose", "msg": {"position": dict(zip("xyz", xyz)), "orientation": dict(zip("xyzw", quaternion))}})
    client.send({"op": "publish", "topic": "/joint_states", "msg": {"header": {"stamp": stamp}, "name": list(joints), "position": list(joints.values())}})
    def current(topic):
        return lambda m: m.get("op") == "publish" and m.get("topic") == topic and bool(m.get("msg", {}).get("transforms")) and m["msg"]["transforms"][0]["header"]["stamp"] == stamp
    tf = client.wait(lambda m: m.get("op") == "publish" and m.get("topic") == "/tf" and (not m.get("msg", {}).get("transforms") or current("/tf")(m)))["msg"]
    world = client.wait(current("/xform_world"))["msg"] if tf.get("transforms") else {"transforms": []}
    return {"world": world, "configuration": body}


def evaluate(body):
    if not isinstance(body, dict):
        raise ValueError("Supply a configuration object")
    configuration(body)
    client = Client()
    try:
        client.subscribe("/tf")
        client.subscribe("/xform_world")
        load_description(client, body["urdf"])
        return publish_pose(client, body)
    finally:
        client.close()


MOTION_LOCK = threading.Lock()
CURRENT_MOTION = None


class Motion:
    def __init__(self, body):
        if not isinstance(body, dict):
            raise ValueError("Supply motion inputs")
        self.urdf = body["urdf"]
        if not isinstance(self.urdf, str) or len(self.urdf) > 2_000_000:
            raise ValueError("Supply URDF text")
        self.start = configuration(body["start"])
        self.goal = configuration(body["goal"])
        self.duration = body.get("duration", 3)
        if isinstance(self.duration, bool) or not isinstance(self.duration, (int, float)) or not math.isfinite(self.duration) or not .2 <= self.duration <= 120:
            raise ValueError("Use a duration from 0.2 to 120 seconds")
        self.id = uuid.uuid4().hex
        self.condition = threading.Condition()
        self.paused = self.stopped = self.finished = self.claimed = False
        self.elapsed = 0.0
        self.client = None

    def command(self, action):
        close = None
        with self.condition:
            if action == "pause" and not self.finished:
                self.paused = True
            elif action == "resume" and not self.finished:
                self.paused = False
            elif action == "stop":
                self.stopped = True
                close = self.client
            self.condition.notify_all()
            result = {"state": "stopped" if self.stopped else "complete" if self.finished else "paused" if self.paused else "playing", "elapsed": self.elapsed, "duration": self.duration}
        if close is not None:
            try:
                close.socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            close.close()
        return result

    def input_at(self, elapsed):
        fraction = min(1, max(0, elapsed / self.duration))
        names = set(self.start["joints"]) | set(self.goal["joints"])
        return {
            "joints": {name: self.start["joints"].get(name, 0) + fraction * (self.goal["joints"].get(name, self.start["joints"].get(name, 0)) - self.start["joints"].get(name, 0)) for name in names},
            "xyz": [a + fraction * (b-a) for a, b in zip(self.start["xyz"], self.goal["xyz"])],
            "rpy": [a + fraction * (b-a) for a, b in zip(self.start["rpy"], self.goal["rpy"])],
        }


def motion_by_id(identity):
    with MOTION_LOCK:
        motion = CURRENT_MOTION
        if motion is None or motion.id != identity:
            raise ValueError("Playback is unavailable")
        return motion


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def json(self, value, status=200):
        data = json.dumps(value, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def start_events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Connection", "close")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        self.close_connection = True
        self.connection.settimeout(3)

    def event(self, data, name=None):
        prefix = "event: " + name + "\n" if name else ""
        self.wfile.write((prefix + "data: " + json.dumps(data, allow_nan=False) + "\n\n").encode())
        self.wfile.flush()

    def stream_error(self):
        try:
            self.event({"error": "Playback stopped. Start the project runtime and try again."}, "stream_error")
        except OSError:
            pass

    def live_events(self):
        self.start_events()
        client = None
        try:
            client = Client()
            client.subscribe("/xform_world")
            previous = None
            refresh = 0.0
            while True:
                now = time.monotonic()
                if now >= refresh:
                    description = client.call("/param_server/get_param", {"name": "robot_description"})["value"]
                    if not isinstance(description, str):
                        raise ValueError("Robot description unavailable")
                    if description != previous:
                        self.event({"urdf": description}, "robot")
                        previous = description
                    refresh = now + 1
                message = client.wait(lambda m: m.get("op") == "publish" and m.get("topic") == "/xform_world", timeout=2)
                queued = [m for m in client.pending if m.get("op") == "publish" and m.get("topic") == "/xform_world"]
                if queued:
                    message = queued[-1]
                    client.pending = [m for m in client.pending if m not in queued]
                self.event({"world": message["msg"]})
        except (BrokenPipeError, ConnectionResetError):
            pass
        except (OSError, TimeoutError, KeyError, TypeError, ValueError):
            self.stream_error()
        finally:
            if client is not None:
                client.close()

    def motion_events(self, motion):
        with motion.condition:
            if motion.claimed or motion.stopped:
                return self.json({"error": "Playback is unavailable"}, 409)
            motion.claimed = True
        self.start_events()
        client = None
        try:
            client = Client()
            with motion.condition:
                motion.client = client
                if motion.stopped:
                    return
            client.subscribe("/tf")
            client.subscribe("/xform_world")
            load_description(client, motion.urdf)
            data = publish_pose(client, motion.start)
            self.event(dict(data, elapsed=0, duration=motion.duration))
            previous = time.monotonic()
            while True:
                with motion.condition:
                    if motion.stopped:
                        break
                    if motion.paused:
                        previous = None
                        self.event({"state": "paused", "elapsed": motion.elapsed, "duration": motion.duration}, "playback")
                        motion.condition.wait(.25)
                        continue
                    now = time.monotonic()
                    elapsed = min(motion.duration, motion.elapsed + (now - previous if previous is not None else 0))
                    previous = now
                data = publish_pose(client, motion.input_at(elapsed))
                with motion.condition:
                    motion.elapsed = elapsed
                self.event(dict(data, elapsed=elapsed, duration=motion.duration))
                if elapsed >= motion.duration:
                    with motion.condition:
                        motion.finished = True
                    self.event({"state": "complete", "elapsed": elapsed, "duration": motion.duration}, "done")
                    return
                time.sleep(.02)
            self.event({"state": "stopped", "elapsed": motion.elapsed, "duration": motion.duration}, "done")
        except (BrokenPipeError, ConnectionResetError):
            pass
        except (OSError, TimeoutError, KeyError, TypeError, ValueError):
            if not motion.stopped:
                self.stream_error()
        finally:
            motion.command("stop")
            if client is not None:
                client.close()

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/example":
            return self.json({"urdf": EXAMPLE})
        if path == "/api/health":
            return self.json({"ok": True, "runtime": "%s:%s" % (OPTIONS.runtime_host, OPTIONS.runtime_port)})
        if path == "/api/stream":
            return self.live_events()
        if path.startswith("/api/motion/") and path.endswith("/events"):
            try:
                motion = motion_by_id(path.split("/")[3])
            except ValueError:
                return self.json({"error": "Playback is unavailable"}, 404)
            return self.motion_events(motion)
        files = {"/": ("index.html", "text/html"), "/viewer.js": ("viewer.js", "text/javascript"), "/style.css": ("style.css", "text/css")}
        if path not in files:
            return self.send_error(404)
        name, mime = files[path]
        data = (ROOT/"viewer"/name).read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", mime+"; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self):
        global CURRENT_MOTION
        try:
            length = int(self.headers.get("Content-Length", 0))
            if not 0 < length <= 2_100_000:
                raise ValueError("Supply JSON inputs")
            body = json.loads(self.rfile.read(length))
            if self.path == "/api/evaluate":
                with MOTION_LOCK:
                    if CURRENT_MOTION is not None:
                        CURRENT_MOTION.command("stop")
                with LOCK:
                    result = evaluate(body)
            elif self.path == "/api/motion":
                motion = Motion(body)
                with MOTION_LOCK:
                    if CURRENT_MOTION is not None:
                        CURRENT_MOTION.command("stop")
                    CURRENT_MOTION = motion
                result = {"id": motion.id}
            elif self.path.startswith("/api/motion/"):
                pieces = self.path.split("/")
                if len(pieces) != 5 or pieces[4] not in ("pause", "resume", "stop"):
                    return self.send_error(404)
                result = motion_by_id(pieces[3]).command(pieces[4])
            else:
                return self.send_error(404)
            self.json(result)
        except (BrokenPipeError, ConnectionResetError):
            pass
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
