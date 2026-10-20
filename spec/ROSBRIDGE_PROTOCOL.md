# External rosbridge subset

Version: 1.0

This is the authoritative black-box grading protocol. It intentionally uses
raw TCP rather than WebSockets and has no ROS installation dependency.

## Transport and framing

The server must be reachable at `127.0.0.1:9095` and accepts concurrent TCP clients.
Traffic is UTF-8. Each frame is exactly one JSON value serialized on one line
and terminated by byte `0x0a` (`\n`). Senders escape newlines inside JSON
strings. JSON objects are compared semantically: key order and insignificant
whitespace do not matter.

Messages in this protocol are JSON objects. A blank line may be ignored. A
malformed JSON line may be rejected or close the offending connection, but
must not terminate the gateway or unrelated connections. Implementations may
reject a line larger than 4 MiB.

Normal Project 1 inputs use non-empty topic and service names beginning with
`/`; malformed or degenerate-name behavior is unspecified. Type names are
informational strings. IDs are opaque JSON values, unique
among a client's outstanding operations, and are echoed without rewriting.
Unknown object fields are ignored for forward compatibility.

## Topic operations

### `advertise`

Required: `op`, `topic`, `type`. Optional: `id`.

```json
{"op":"advertise","topic":"/map","type":"nav_msgs/OccupancyGrid","id":"a1"}
```

It registers that connection as a publisher. Repeating the same advertisement
is idempotent. No acknowledgement is required.

### `unadvertise`

Required: `op`, `topic`. Optional: `id`.

```json
{"op":"unadvertise","topic":"/map"}
```

It removes that connection's publisher. Repetition is harmless.

### `publish`

Required: `op`, `topic`, `msg`. Optional: `id`.

```json
{"op":"publish","topic":"/chatter","msg":{"data":"hello"}}
```

`msg` may be any JSON value. The gateway forwards the payload only to current
matching subscribers. There is no history, latching, or replay.

### `subscribe`

Required: `op`, `topic`, `type`. Optional: `id`.

```json
{"op":"subscribe","topic":"/path","type":"nav_msgs/Path","id":"s1"}
```

The gateway replies with an informational `status` after the subscription is
registered. Each subsequent message arrives as:

```json
{"op":"publish","topic":"/path","msg":{}}
```

A client receives only topics it subscribed to. Repeating a subscription is
idempotent.

### `unsubscribe`

Required: `op`, `topic`. Optional: `id`.

```json
{"op":"unsubscribe","topic":"/path"}
```

It removes that connection's subscription. Repetition is harmless.

## Service operations

### `advertise_service`

Required: `op`, `service`, `type`. Optional: `id`.

```json
{"op":"advertise_service","service":"/echo","type":"example/Echo","id":"p1"}
```

The connection becomes the service provider. The gateway replies with an
informational `status` once registration completes. The reference middleware
supports one current provider per service name; a newer registration replaces
the discovery entry for future calls.

### `unadvertise_service`

Required: `op`, `service`. Optional: `id`.

```json
{"op":"unadvertise_service","service":"/echo"}
```

It removes the service only when this connection is its current provider.

### `call_service`

Required: `op`, `service`, `id`, `args`.

```json
{
  "op":"call_service",
  "service":"/plan_path",
  "id":"grader-42",
  "args":{"start":{},"goal":{},"tolerance":0.0}
}
```

At the generic transport layer `args` is any JSON value. Named argument
objects are canonical; Project 1 APIs define their own accepted forms. A
provided external service receives a forwarded call with the same shape but
with a gateway-generated provider-side ID. It responds with:

```json
{
  "op":"service_response",
  "service":"/echo",
  "id":"provider_call_7",
  "values":{"echo":"value"},
  "result":true,
  "status":""
}
```

The gateway returns a response to the original caller, restoring the caller's
ID:

```json
{
  "op":"service_response",
  "service":"/echo",
  "id":"grader-42",
  "values":{"echo":"value"},
  "result":true,
  "status":""
}
```

Project 1-owned services use object `values`, Boolean `result`, and string
`status`; the generic gateway need not type-check arbitrary external provider
responses. A missing provider, timeout, or provider disconnect fails cleanly
with the original ID and `result:false`. Calls are correlated by both
connection and ID; unrelated publish or
status traffic may arrive before the response.

### `service_response`

Required: `op`, `service`, `id`, `values`, `result`. Optional: `status`.
It is valid only for an outstanding provider-side call on that connection.
Unknown or stale IDs produce an error status or are ignored; they must never
complete another caller's request.

## Status messages

Gateway-generated status messages have this form:

```json
{"op":"status","level":"info","msg":"subscribed to /path","id":"s1"}
```

`level` is `"info"` or `"error"`; `msg` is a diagnostic string; `id` is
included and echoed when the triggering request supplied one. Unknown `op`
values and invalid required fields produce an error status. Errors are scoped
to the offending connection and do not crash the gateway.

## Connection lifecycle

Closing an external connection withdraws all publishers, subscribers, and
services owned by it. Delivery is best-effort and non-persistent. Different
clients may send and receive concurrently; one slow client must not prevent
unrelated clients from making progress within grader deadlines.
