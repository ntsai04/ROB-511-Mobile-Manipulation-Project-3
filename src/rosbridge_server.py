"""TCP/JSON rosbridge gateway on ``127.0.0.1:9095``.

HOW TO READ THIS FILE
---------------------
This is the small ROS-like switchboard from Projects 1 and 2. External clients
open TCP connections and send JSON objects with an op field. There are two
threads for each accepted socket, with the same responsibilities as before:

    reader   serve_client() -> parse one line -> handle_request()
    writer   outgoing.get() -> JSON serialization -> socket.sendall()

Only the writer sends bytes to its socket. Every other thread calls send(),
which puts a whole message on that client's queue. This prevents a service
response and a topic publication from interleaving their bytes. Each client
has its own queue and writer, so one slow socket does not block other clients.

The gateway keeps transport rules here and routing records in Hub. Project 1
called heap/A* handlers directly. Project 2 introduced local_services and
topic_listeners; Project 3 keeps that interface for the parameter server and
robot nodes. src/main.py registers them before starting client threads.

FOLLOW A MESSAGE
-----------------
- A topic publish delivers its payload to local listeners and the current
  matching external subscribers. Topics carry streams, with no reply needed.
- A local service call invokes its registered handler and returns the handler's
  result/values/status triple in a service_response envelope.
- An external service call gets a new provider-side id. Hub remembers the
  caller's connection and original id until a response, timeout or disconnect.
  Responses must match both the provider connection and the service name.

PYTHON NOTES
------------
- Queue.get() waits for work. Putting None on the queue asks its writer to stop;
  None cannot be confused with a JSON message because envelopes are dicts.
- A Lock protects the closed flag and queue insertion as one operation.
- daemon=True threads end when the foreground runtime exits.
- json.dumps(..., allow_nan=False) refuses non-finite numbers. Decoder hooks
  also reject NaN, Infinity and overflowing floats before forwarding them.
- TCP packets are not messages. A binary readline grows until the newline,
  allowing up to 4 MiB of UTF-8 payload for large robot descriptions.
- time.monotonic() measures service deadlines without wall-clock adjustments.
"""
from __future__ import annotations

import json
import math
import queue
import socket
import sys
import threading
import time
import uuid
from typing import Any

try:
    from src.hub import Hub, PendingCall
except ModuleNotFoundError:
    from hub import Hub, PendingCall


HOST = "127.0.0.1"
PORT = 9095
SERVICE_TIMEOUT_SECONDS = 5.0
MAX_LINE_BYTES = 4 * 1024 * 1024


class ClientConnection:
    """One external TCP client: socket, outbound queue, and Hub registrations."""

    def __init__(self, server: "RosbridgeServer", connection: socket.socket) -> None:
        self.server = server
        self.connection = connection
        # Writer waits on this queue.  dict = JSON to send.  None = shut down.
        self.outgoing: queue.Queue[dict[str, Any] | None] = queue.Queue()
        # Hub uses these sets to remove only this connection's registrations.
        self.subscriptions: set[str] = set()
        self.publishers: set[str] = set()
        self.services: set[str] = set()
        self._closed = False
        self._close_lock = threading.Lock()
        self._writer = threading.Thread(target=self._write_loop, daemon=True)

    def start(self) -> None:
        """Start the writer thread.  The reader is the thread that called us."""
        self._writer.start()

    def send(self, message: dict[str, Any]) -> None:
        """Enqueue one complete envelope without waiting for its socket write.

        The same lock protects send() and close(). A sender cannot enqueue
        more work after close() has put the shutdown sentinel on this queue.
        """
        with self._close_lock:
            if not self._closed:
                self.outgoing.put(message)

    def _write_loop(self) -> None:
        """Take queued messages and write them as UTF-8 JSON lines.

        The queue serializes all outbound messages for this one socket. Errors
        in a socket or an unserializable payload close only this connection.
        Finally always unregisters it, even when cleanup also runs in the reader.
        """
        try:
            while True:
                message = self.outgoing.get()
                if message is None:
                    return
                serialized = json.dumps(message, separators=(",", ":"), allow_nan=False)
                encoded = serialized.encode("utf-8")
                self.connection.sendall(encoded + b"\n")
        except (OSError, TypeError, ValueError, UnicodeError):
            pass
        finally:
            self.server.disconnect_client(self)

    def close(self) -> None:
        """Idempotent shutdown: one stop-sentinel, then close the socket.

        _closed is set first so late send() calls do not queue work after we
        have already asked the writer to exit.  shutdown() unblocks a reader
        stuck in readline.  close() is wrapped because the peer may have gone.
        """
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            self.outgoing.put(None)
            try:
                self.connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self.connection.close()
            except OSError:
                pass


class RosbridgeServer:
    """Shared transport state plus the in-process node endpoints."""

    def __init__(self) -> None:
        self.hub = Hub()
        # A handler receives (service name, JSON arguments) and returns the
        # result/values/status triple of a service response. Keeping the gateway
        # ignorant of individual service schemas makes its protocol reusable.
        self.local_services: dict[str, Any] = {}
        self.topic_listeners: dict[str, list[Any]] = {}

    def register_local_service(self, name: str, handler: Any) -> None:
        """Register handler(service_name, args), returning (result, values, status).

        main.py uses the same interface introduced in Project 2. Nodes own
        argument validation; the gateway owns the JSON response envelope.
        """
        self.local_services[name] = handler

    def register_topic_listener(self, topic: str, listener: Any) -> None:
        """Register listener(message) before client threads begin running."""
        if topic not in self.topic_listeners:
            self.topic_listeners[topic] = []
        self.topic_listeners[topic].append(listener)

    def publish_local(self, topic: str, message: Any) -> None:
        """Deliver a node's publication to local and external subscribers.

        Local delivery passes the same JSON-shaped value directly. External
        delivery queues an envelope for serialization by each client's writer.
        """
        self._notify_listeners(topic, message)
        self._broadcast(topic, message)

    def _notify_listeners(self, topic: str, message: Any) -> None:
        """Call each local subscriber, isolating a rejected message to that node.

        Keep the existing diagnostic for a failing callback and continue to
        the remaining listeners. A node error must not kill a TCP reader or
        prevent matching external subscribers from receiving the publication.
        """
        listeners = list(self.topic_listeners.get(topic, ()))
        for listener in listeners:
            try:
                listener(message)
            except Exception as error:
                print(f"subscriber for {topic} rejected message: {error}",
                      file=sys.stderr, flush=True)

    @staticmethod
    def send_status(client: ClientConnection, level: str, message: str,
                    request: dict[str, Any]) -> None:
        """Send a protocol diagnostic, preserving an optional opaque request id.

        Check key presence rather than truthiness: null, 0 and an empty string
        are still ids when the sender explicitly supplied them.
        """
        envelope: dict[str, Any] = {"op": "status", "level": level, "msg": message}
        if "id" in request:
            envelope["id"] = request["id"]
        client.send(envelope)

    @staticmethod
    def send_service_response(client: ClientConnection, service: str, request_id: Any,
                              result: bool, values: Any = None, status: str = "") -> None:
        """Build a local service/failure reply; unspecified values become {}.

        External provider responses use their own envelope below so an explicit
        JSON null in a generic provider's values field remains null.
        """
        client.send({
            "op": "service_response",
            "service": service,
            "id": request_id,
            "values": {} if values is None else values,
            "result": result,
            "status": status,
        })

    def disconnect_client(self, client: ClientConnection) -> None:
        """Close the socket, unregister in Hub, and fail its waiting callers.

        close() is safe to call twice (reader finally + writer finally).
        Mark the client closed first so begin_call() cannot register new work
        for a provider after Hub has already removed that provider's records.
        Failure messages are queued only after Hub releases its lock.
        """
        client.close()
        failures = self.hub.disconnect(client)
        for pending in failures:
            self.send_service_response(
                pending.caller, pending.service, pending.caller_id,
                False, status="service provider disconnected",
            )

    def expire_calls_forever(self) -> None:
        """Periodically fail forwarded calls whose monotonic deadlines expired.

        Hub removes each record before returning it. A late provider response
        then finds no route and cannot complete the same request a second time.
        """
        while True:
            time.sleep(0.25)
            for pending in self.hub.expired_calls():
                self.send_service_response(
                    pending.caller, pending.service, pending.caller_id,
                    False, status="service call timed out",
                )

    def handle_request(self, client: ClientConnection, request: Any) -> None:
        """Validate the envelope, then dispatch by the same Project 1/2 op table.

        Each handler checks its own required fields. Unknown object fields are
        left alone, allowing future clients to add optional protocol metadata.
        """
        if not isinstance(request, dict):
            self.send_status(client, "error", "request must contain a string op", {})
            return
        if not isinstance(request.get("op"), str):
            self.send_status(client, "error", "request must contain a string op", request)
            return

        operation = request["op"]
        handlers = {
            "advertise": self._advertise,
            "unadvertise": self._unadvertise,
            "publish": self._publish,
            "subscribe": self._subscribe,
            "unsubscribe": self._unsubscribe,
            "advertise_service": self._advertise_service,
            "unadvertise_service": self._unadvertise_service,
            "call_service": self._call_service,
            "service_response": self._forward_service_response,
        }
        handler = handlers.get(operation)
        if handler is None:
            self.send_status(client, "error", f"unknown operation: {operation}", request)
            return
        handler(client, request)

    def _advertise(self, client: ClientConnection, request: dict[str, Any]) -> None:
        """Record an advertised topic on its client; repeated requests are safe.

        Type names are required informational strings. The wire contract does
        not require payload schema checks or prior advertisement to publish.
        """
        topic = request.get("topic")
        if isinstance(topic, str) and isinstance(request.get("type"), str):
            client.publishers.add(topic)
        else:
            self.send_status(client, "error", "advertise requires topic and type", request)

    def _unadvertise(self, client: ClientConnection, request: dict[str, Any]) -> None:
        """Remove this client's publisher registration if it exists."""
        topic = request.get("topic")
        if isinstance(topic, str):
            client.publishers.discard(topic)
        else:
            self.send_status(client, "error", "unadvertise requires topic", request)

    def _publish(self, client: ClientConnection, request: dict[str, Any]) -> None:
        """Deliver any JSON msg value only to current matching subscribers."""
        topic = request.get("topic")
        if not isinstance(topic, str) or "msg" not in request:
            self.send_status(client, "error", "publish requires topic and msg", request)
            return
        self._notify_listeners(topic, request["msg"])
        self._broadcast(topic, request["msg"])

    def _broadcast(self, topic: str, msg: Any) -> None:
        """Copy recipients from Hub, then send without holding Hub's lock."""
        envelope = {"op": "publish", "topic": topic, "msg": msg}
        for subscriber in self.hub.publication_recipients(topic):
            subscriber.send(envelope)

    def _subscribe(self, client: ClientConnection, request: dict[str, Any]) -> None:
        """Register before acknowledging, so a following publish can find it."""
        topic = request.get("topic")
        if not isinstance(topic, str) or not isinstance(request.get("type"), str):
            self.send_status(client, "error", "subscribe requires topic and type", request)
            return
        self.hub.subscribe(client, topic)
        self.send_status(client, "info", f"subscribed to {topic}", request)

    def _unsubscribe(self, client: ClientConnection, request: dict[str, Any]) -> None:
        """Stop routing this topic to the client; repeated requests are safe."""
        topic = request.get("topic")
        if isinstance(topic, str):
            self.hub.unsubscribe(client, topic)
        else:
            self.send_status(client, "error", "unsubscribe requires topic", request)

    def _advertise_service(self, client: ClientConnection, request: dict[str, Any]) -> None:
        """Last advertiser becomes the unique provider of this service name."""
        service = request.get("service")
        if not isinstance(service, str) or not isinstance(request.get("type"), str):
            self.send_status(client, "error", "advertise_service requires service and type", request)
            return
        self.hub.advertise_service(client, service)
        self.send_status(client, "info", f"advertised service {service}", request)

    def _unadvertise_service(self, client: ClientConnection, request: dict[str, Any]) -> None:
        """Withdraw a service only if this client remains its current provider."""
        service = request.get("service")
        if isinstance(service, str):
            self.hub.unadvertise_service(client, service)
        else:
            self.send_status(client, "error", "unadvertise_service requires service", request)

    def _call_service(self, client: ClientConnection, request: dict[str, Any]) -> None:
        """Answer locally, or forward to the current provider.

        Required fields: service (string), id (any JSON), args (any JSON).
        Local calls never create a PendingCall.  Forwarded calls do.
        """
        service = request.get("service")
        if not isinstance(service, str) or "id" not in request or "args" not in request:
            self.send_status(client, "error", "call_service requires service, id, and args", request)
            return

        # Built-in services are implemented in this process.  Their handler
        # returns the same fields the caller will see in service_response.
        local_handler = self.local_services.get(service)
        if local_handler is not None:
            try:
                result, values, status = local_handler(service, request["args"])
            except Exception as error:
                result = False
                values = {}
                status = f"service rejected request: {error}"
            self.send_service_response(client, service, request["id"], result, values, status)
            return

        # Caller ids are only unique *per connection*.  Two clients can both
        # send id "call-1".  We invent a unique provider_id so the later
        # service_response cannot be matched to the wrong caller.
        provider_id = "provider-call-" + uuid.uuid4().hex
        deadline = time.monotonic() + SERVICE_TIMEOUT_SECONDS
        pending = PendingCall(client, request["id"], service, deadline)
        # Select the provider and register its pending call under the same Hub
        # lock; separate operations could lose a provider disconnect between them.
        provider = self.hub.begin_call(
            service,
            provider_id,
            pending,
        )
        if provider is None:
            self.send_service_response(
                client, service, request["id"], False, status="service unavailable",
            )
            return
        provider.send({
            "op": "call_service",
            "service": service,
            "id": provider_id,
            "args": request["args"],
        })

    def _forward_service_response(self, client: ClientConnection, request: dict[str, Any]) -> None:
        """Provider answered.  Look up the original caller and rewrite the id.

        Unknown or already-expired ids are ignored (timeout may have already
        sent result:false).  Generic provider payloads are preserved as sent.
        """
        provider_id = request.get("id")
        service = request.get("service")
        if (not isinstance(provider_id, str) or not isinstance(service, str)
                or "values" not in request or "result" not in request):
            self.send_status(
                client, "error", "service_response requires service, id, values, and result", request,
            )
            return
        pending = self.hub.take_pending_call(client, provider_id, service)
        if pending is None:
            return
        # Do not coerce generic provider values/result/status. In particular,
        # values:null must remain null, unlike an omitted local failure value.
        pending.caller.send({
            "op": "service_response",
            "service": pending.service,
            "id": pending.caller_id,
            "values": request["values"],
            "result": request["result"],
            "status": request.get("status", ""),
        })


def serve_client(server: RosbridgeServer, connection: socket.socket) -> None:
    """Reader thread for one socket: one JSON object per line, then disconnect.

    A bounded binary readline grows through the full 4 MiB allowed by the
    protocol; it never truncates large descriptions at a socket packet or
    small fixed buffer.  Invalid JSON only affects the offending connection.

    Count bytes before decoding: the contract's bound concerns UTF-8 bytes,
    not Python character count. The extra two bytes distinguish a permitted
    maximum-size payload plus newline from an oversized payload. Close an
    oversized connection instead of parsing its remainder as a new request.
    """
    client = ClientConnection(server, connection)
    client.start()
    try:
        with connection.makefile("rb") as incoming:
            while True:
                line = incoming.readline(MAX_LINE_BYTES + 2)
                if not line:
                    break
                if line.endswith(b"\n"):
                    frame = line[:-1]
                else:
                    frame = line
                if len(frame) > MAX_LINE_BYTES:
                    break
                if not frame.strip():
                    continue
                try:
                    text = frame.decode("utf-8")
                    request = json.loads(
                        text,
                        parse_constant=_invalid_constant,
                        parse_float=_finite_float,
                    )
                    server.handle_request(client, request)
                except (json.JSONDecodeError, ValueError, RecursionError):
                    server.send_status(client, "error", "invalid JSON", {})
    except (OSError, UnicodeError):
        pass
    finally:
        server.disconnect_client(client)


def _invalid_constant(value: str) -> None:
    """Python's JSON decoder otherwise accepts non-JSON NaN and Infinity."""
    raise ValueError(f"invalid JSON constant: {value}")


def _finite_float(value: str) -> float:
    """Reject overflowing numbers before they can reach another writer."""
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("non-finite JSON number")
    return number


def main() -> int:
    """Listen on HOST:PORT.  Each accept() gets its own daemon reader thread."""
    server = RosbridgeServer()
    threading.Thread(target=server.expire_calls_forever, daemon=True).start()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            # SO_REUSEADDR lets us bind again quickly after a restart.
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((HOST, PORT))
            listener.listen()
            print(f"Autorob gateway listening on {HOST}:{PORT}", file=sys.stderr, flush=True)
            while True:
                connection, _address = listener.accept()
                threading.Thread(
                    target=serve_client, args=(server, connection), daemon=True,
                ).start()
    except OSError as error:
        print(f"unable to start gateway: {error}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
