"""Local static viewer plus an external rosbridge-to-SSE adapter. No dependencies."""
from __future__ import annotations

import argparse
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import queue
import threading
import time
from urllib.parse import urlsplit

from wire_client import TOPICS, WireClient
from webm_duration import finalize_webm


ROOT = Path(__file__).resolve().parents[1]


class Bridge:
    def __init__(self, host, port):
        self.host, self.port = host, port
        self.lock = threading.Lock()
        self.listeners = set()
        self.latest = {}
        self.stopped = threading.Event()

    def emit(self, topic, msg):
        event = {"topic": topic, "msg": msg}
        with self.lock:
            self.latest[topic] = event
            for listener in self.listeners:
                if listener.full():
                    try:
                        listener.get_nowait()
                    except queue.Empty:
                        pass
                listener.put_nowait(event)

    def run(self):
        while not self.stopped.is_set():
            try:
                with WireClient(self.host, self.port) as client:
                    for topic, type_name in TOPICS.items():
                        client.subscribe(topic, type_name)
                    self.emit("connection", {"connected": True})
                    last_description = 0.0
                    version = None
                    while not self.stopped.is_set():
                        if time.monotonic() - last_description >= 1:
                            response = client.call("/param_server/get_param", {"name": "robot_description"})
                            if response.get("result") and response["values"]["version"] != version:
                                version = response["values"]["version"]
                                self.emit("robot_description", response["values"]["value"])
                            last_description = time.monotonic()
                        try:
                            message = client.next_publication(0.25)
                            self.emit(message["topic"], message["msg"])
                        except TimeoutError:
                            pass
            except (OSError, ConnectionError, TimeoutError, ValueError) as error:
                self.emit("connection", {"connected": False, "error": str(error)})
                self.stopped.wait(1)


class ViewerHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def log_message(self, format, *args):
        if str(args[1] if len(args) > 1 else "") != "200":
            super().log_message(format, *args)

    def do_GET(self):
        if urlsplit(self.path).path != "/events":
            return super().do_GET()
        listener = queue.Queue(maxsize=100)
        with self.server.bridge.lock:
            self.server.bridge.listeners.add(listener)
            initial = list(self.server.bridge.latest.values())
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        try:
            for event in initial:
                self.wfile.write(("data: " + json.dumps(event) + "\n\n").encode())
            self.wfile.flush()
            while not self.server.bridge.stopped.is_set():
                try:
                    event = listener.get(timeout=5)
                    data = "data: " + json.dumps(event, allow_nan=False) + "\n\n"
                except queue.Empty:
                    data = ": keepalive\n\n"
                self.wfile.write(data.encode("utf-8"))
                self.wfile.flush()
        except (OSError, ConnectionError):
            pass
        finally:
            with self.server.bridge.lock:
                self.server.bridge.listeners.discard(listener)

    def do_POST(self):
        # Requests come only from this local viewer. Reject cross-origin writes.
        origin = self.headers.get("Origin")
        if origin and origin not in (f"http://127.0.0.1:{self.server.server_port}",
                                     f"http://localhost:{self.server.server_port}"):
            return self.send_error(403, "cross-origin writes are disabled")
        path = urlsplit(self.path).path
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            return self.send_error(400, "invalid Content-Length")
        if length <= 0:
            return self.send_error(400, "a nonempty request body is required")
        if path == "/api/poster":
            if length > 8 * 1024 * 1024:
                return self.send_error(413, "poster must be at most 8 MiB")
            data = self.rfile.read(length)
            if not data.startswith(b"\x89PNG\r\n\x1a\n"):
                return self.send_error(400, "poster must be a PNG image")
            output = ROOT / "portfolio" / "poster.png"
            output.parent.mkdir(exist_ok=True)
            output.write_bytes(data)
            return self.json_response({"saved": True, "path": "portfolio/poster.png"})
        if path == "/api/video":
            if not 0 < length <= 64 * 1024 * 1024:
                return self.send_error(413, "video must be at most 64 MiB")
            try:
                hint = float(self.headers["X-Video-Duration"]) if "X-Video-Duration" in self.headers else None
                video, seconds = finalize_webm(self.rfile.read(length), hint)
                output = ROOT / "portfolio" / "demo.webm"
                output.parent.mkdir(exist_ok=True)
                temporary = output.with_suffix(".webm.tmp")
                temporary.write_bytes(video)
                temporary.replace(output)
            except (ValueError, OverflowError, OSError) as error:
                return self.json_response({"saved": False, "status": str(error)}, status=400)
            return self.json_response({"saved": True, "path": "portfolio/demo.webm", "duration": seconds})
        if path not in ("/api/call", "/api/publish") or length > 2 * 1024 * 1024:
            return self.send_error(404)
        try:
            request = json.loads(self.rfile.read(length))
            with WireClient(self.server.bridge.host, self.server.bridge.port) as client:
                if path == "/api/call":
                    result = client.call(request["service"], request.get("args", {}))
                else:
                    client.send({"op": "advertise", "topic": request["topic"], "type": request["type"]})
                    client.send({"op": "publish", "topic": request["topic"], "msg": request["msg"]})
                    result = {"result": True, "status": ""}
            self.json_response(result)
        except (KeyError, ValueError, OSError, TimeoutError, ConnectionError) as error:
            self.json_response({"result": False, "status": str(error)}, status=400)

    def json_response(self, value, status=200):
        data = json.dumps(value, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--runtime-host", default="127.0.0.1")
    parser.add_argument("--runtime-port", type=int, default=9095)
    args = parser.parse_args()
    bridge = Bridge(args.runtime_host, args.runtime_port)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), ViewerHandler)
    server.daemon_threads = True
    server.bridge = bridge
    threading.Thread(target=bridge.run, daemon=True).start()
    print(f"Viewer: http://127.0.0.1:{args.port}/viewer/", flush=True)
    print(f"Portfolio: http://127.0.0.1:{args.port}/portfolio/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stopped.set()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
