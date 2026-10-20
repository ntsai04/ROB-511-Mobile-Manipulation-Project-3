"""Contract checks for the reused gateway, independent of a listening port."""

from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rosbridge_server import MAX_LINE_BYTES, RosbridgeServer, serve_client


class Client:
    def __init__(self):
        self.subscriptions = set()
        self.publishers = set()
        self.services = set()
        self.messages = []
        self._closed = False

    def send(self, message):
        if not self._closed:
            self.messages.append(message)

    def close(self):
        self._closed = True


class FragmentedInput(io.RawIOBase):
    """Expose a stream in short reads, as TCP does across packet boundaries."""

    def __init__(self, data):
        self.data = memoryview(data)
        self.offset = 0

    def readable(self):
        return True

    def readinto(self, target):
        count = min(len(target), 7919, len(self.data) - self.offset)
        target[:count] = self.data[self.offset:self.offset + count]
        self.offset += count
        return count


class InputConnection:
    def __init__(self, data):
        self.data = data

    def makefile(self, _mode):
        return io.BufferedReader(FragmentedInput(self.data))

    def sendall(self, _data):
        pass

    def shutdown(self, _how):
        pass

    def close(self):
        pass


class ParsingServer(RosbridgeServer):
    def __init__(self):
        super().__init__()
        self.requests = []
        self.errors = []

    def handle_request(self, _client, request):
        self.requests.append(request)

    def send_status(self, _client, level, message, _request):
        self.errors.append((level, message))


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.server = RosbridgeServer()
        self.provider = Client()
        self.first = Client()
        self.second = Client()
        self.server.handle_request(self.provider, {
            "op": "advertise_service", "service": "/echo", "type": "test/Echo",
        })
        self.provider.messages.clear()

    def call(self, caller, request_id, args):
        self.server.handle_request(caller, {
            "op": "call_service", "service": "/echo", "id": request_id, "args": args,
        })
        return self.provider.messages[-1]

    def respond(self, provider_id, **fields):
        request = {"op": "service_response", "service": "/echo",
                   "id": provider_id, "values": None, "result": True}
        request.update(fields)
        self.server.handle_request(self.provider, request)

    def test_equal_opaque_caller_ids_are_correlated_by_connection(self):
        opaque = {"operation": [1, None, "opaque"]}
        first = self.call(self.first, opaque, "first")
        second = self.call(self.second, opaque, "second")
        self.assertNotEqual(first["id"], second["id"])
        self.respond(second["id"], values={"answer": "second"})
        self.respond(first["id"], values={"answer": "first"})
        self.assertEqual(self.first.messages[0]["values"], {"answer": "first"})
        self.assertEqual(self.second.messages[0]["values"], {"answer": "second"})
        self.assertEqual(self.first.messages[0]["id"], opaque)

    def test_spoofed_or_mismatched_response_does_not_consume_request(self):
        request = self.call(self.first, "caller", {})
        spoof = {"op": "service_response", "service": "/echo", "id": request["id"],
                 "values": "spoof", "result": True}
        self.server.handle_request(self.second, spoof)
        self.respond(request["id"], service="/wrong")
        self.assertEqual(self.first.messages, [])
        self.respond(request["id"])
        self.assertEqual(len(self.first.messages), 1)
        self.assertIsNone(self.first.messages[0]["values"])
        self.respond(request["id"], values="stale")
        self.assertEqual(len(self.first.messages), 1)

    def test_disconnect_and_timeout_preserve_original_id(self):
        self.call(self.first, ["disconnect", 1], {})
        self.server.disconnect_client(self.provider)
        self.assertEqual(self.first.messages[0]["id"], ["disconnect", 1])
        self.assertFalse(self.first.messages[0]["result"])
        self.assertTrue(self.first.messages[0]["status"])

        self.server.hub.advertise_service(self.second, "/echo")
        self.server.handle_request(self.first, {
            "op": "call_service", "service": "/echo", "id": None, "args": {},
        })
        pending = self.server.hub.expired_calls(float("inf"))
        self.assertEqual(len(pending), 1)
        self.assertIsNone(pending[0].caller_id)
        self.assertEqual(self.server.hub.expired_calls(float("inf")), [])

    def test_callbacks_and_local_services_fail_without_disabling_gateway(self):
        received = []

        def reject(_message):
            raise ValueError("bad node input")

        self.server.register_topic_listener("/input", reject)
        self.server.register_topic_listener("/input", received.append)
        self.server.hub.subscribe(self.second, "/input")
        with contextlib.redirect_stderr(io.StringIO()):
            self.server.handle_request(self.first, {
                "op": "publish", "topic": "/input", "msg": {"data": "value"},
            })
        self.assertEqual(received, [{"data": "value"}])
        self.assertEqual(self.second.messages[0]["msg"], received[0])

        self.server.register_local_service("/local", lambda _name, args: reject(args))
        self.server.handle_request(self.first, {
            "op": "call_service", "service": "/local", "id": 7, "args": {},
        })
        self.assertFalse(self.first.messages[-1]["result"])
        self.server.register_local_service("/local", lambda _name, args: (True, args, ""))
        self.server.handle_request(self.first, {
            "op": "call_service", "service": "/local", "id": 8, "args": [1, 2],
        })
        self.assertEqual(self.first.messages[-1]["values"], [1, 2])

    def test_required_fields_report_errors_with_opaque_id(self):
        for operation, field in [("advertise", "topic"), ("subscribe", "topic"),
                                 ("advertise_service", "service")]:
            self.server.handle_request(self.first, {
                "op": operation, field: "/name", "id": [operation, 1],
            })
            self.assertEqual(self.first.messages[-1]["level"], "error")
            self.assertEqual(self.first.messages[-1]["id"], [operation, 1])

    def test_fragmented_large_utf8_line_and_malformed_line_isolation(self):
        message = {"op": "publish", "topic": "/large", "msg": "\u03bb\n" * 600_000}
        data = json.dumps(message, ensure_ascii=False).encode("utf-8") + b"\n"
        self.assertGreater(len(data), 1024 * 1024)
        self.assertLess(len(data), MAX_LINE_BYTES)
        parser = ParsingServer()
        serve_client(parser, InputConnection(
            b"\n{bad}\n{\"value\":NaN}\n{\"value\":1e9999}\n" + data))
        self.assertEqual(parser.requests, [message])
        self.assertEqual(len(parser.errors), 3)

    def test_oversized_connection_is_closed_without_harming_another(self):
        oversized = ParsingServer()
        serve_client(oversized, InputConnection(b"x" * (MAX_LINE_BYTES + 1) + b"\n"))
        self.assertEqual(oversized.requests, [])
        healthy = ParsingServer()
        serve_client(healthy, InputConnection(b'{"op":"subscribe"}\n'))
        self.assertEqual(healthy.requests, [{"op": "subscribe"}])


if __name__ == "__main__":
    unittest.main()
