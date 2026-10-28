"""Offline debugging UI server. Copied into each project's debug_visualizer/.

Run beside `make run`; the browser talks HTTP to this process, which talks
the course's newline-delimited TCP/JSON protocol to the unmodified runtime.
"""

from __future__ import annotations

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import socket
import time


MAX_BYTES = 2_000_000
MAX_MESSAGES = 5000


def finite_number(value, low, high):
    number = float(value)
    if not math.isfinite(number) or not low <= number <= high:
        raise ValueError(f"expected a number between {low} and {high}")
    return number


def exchange(host, port, payload):
    commands = payload.get("commands", [])
    if not isinstance(commands, list) or len(commands) > 200:
        raise ValueError("commands must be an array of at most 200 objects")
    if any(not isinstance(command, dict) or not isinstance(command.get("op"), str)
           for command in commands):
        raise ValueError("each command needs a string op")
    collect = finite_number(payload.get("collect", 0.2), 0.01, 10)
    timeout = finite_number(payload.get("timeout", 3), 0.05, 15)
    pending = [command.get("id") for command in commands
               if command["op"] == "call_service"]
    encoded = b"".join(json.dumps(command, allow_nan=False).encode("utf-8") + b"\n"
                       for command in commands)
    if len(encoded) > MAX_BYTES:
        raise ValueError("commands are too large")
    messages = []
    received = 0
    truncated = False
    with socket.create_connection((host, port), timeout=min(timeout, 3)) as peer:
        peer.sendall(encoded)
        started = time.monotonic()
        deadline = started + max(collect, timeout)
        buffer = b""
        while len(messages) < MAX_MESSAGES:
            now = time.monotonic()
            if now >= deadline or (not pending and now - started >= collect):
                break
            remaining = deadline - now if pending else started + collect - now
            peer.settimeout(max(0.001, remaining))
            try:
                chunk = peer.recv(65536)
            except socket.timeout:
                break
            if not chunk:
                break
            received += len(chunk)
            if received > 8 * MAX_BYTES:
                truncated = True
                break
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                message = json.loads(line)
                messages.append(message)
                if isinstance(message, dict) and message.get("op") in ("service_response", "status"):
                    request_id = message.get("id")
                    if request_id in pending and (message["op"] == "service_response"
                                                  or message.get("level") == "error"):
                        pending.remove(request_id)
                if len(messages) >= MAX_MESSAGES:
                    truncated = True
                    break
    return {"messages": messages, "timed_out": bool(pending),
            "pending_ids": pending, "truncated": truncated}


class Handler(SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def send_json(self, status, body):
        data = json.dumps(body, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/api/health":
            self.send_json(200, {"ok": True, "runtime_host": self.server.runtime_host,
                                 "runtime_port": self.server.runtime_port})
            return
        super().do_GET()

    def do_POST(self):
        if self.path != "/api/bridge":
            self.send_json(404, {"error": "unknown endpoint"})
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= MAX_BYTES:
                raise ValueError("request must contain at most 2 MB of JSON")
            payload = json.loads(self.rfile.read(size))
            if not isinstance(payload, dict):
                raise ValueError("expected a JSON object")
            result = exchange(self.server.runtime_host, self.server.runtime_port, payload)
            self.send_json(200, result)
        except (ValueError, TypeError, UnicodeError) as error:
            self.send_json(400, {"error": str(error)})
        except (OSError, ConnectionError) as error:
            self.send_json(502, {"error": f"Runtime connection failed: {error}. Start make run in this project."})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parent_name = Path(__file__).resolve().parents[1].name
    default_port = 8000 + int(parent_name[-1]) if parent_name[-1:] in ("1", "2", "3") else 8000
    parser.add_argument("--port", type=int, default=default_port)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--runtime-host", default="127.0.0.1")
    parser.add_argument("--runtime-port", type=int, default=9095)
    args = parser.parse_args()
    directory = Path(__file__).resolve().parent
    if directory.name != "debug_visualizer":
        parser.error("Use the server.py copied into a project's debug_visualizer folder")
    server = ThreadingHTTPServer((args.bind, args.port), partial(Handler, directory=str(directory)))
    server.runtime_host, server.runtime_port = args.runtime_host, args.runtime_port
    print(f"Visualizer: http://localhost:{args.port} (runtime {args.runtime_host}:{args.runtime_port})", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
