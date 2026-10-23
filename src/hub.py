"""In-memory registry of subscribers, service providers, and forwarded calls.

HOW TO READ THIS FILE
---------------------
This is the same Hub used in Projects 1 and 2. Hub never opens sockets and
never runs a robot node. The gateway asks questions such as "who subscribes
to /tf?" or "which caller is waiting for this service response?". Hub answers
from three dictionaries protected by one re-entrant lock:

    _subscribers  topic name -> set of listening ClientConnection objects
    _providers    service name -> the current ClientConnection provider
    _pending      (provider connection, provider-side id) -> PendingCall

A topic can have many subscribers. A service has one current provider, with
the most recent advertisement replacing its discovery entry. A forwarded
call also remembers its particular provider, so replacing the discovery entry
does not transfer an already outstanding call to a different connection.

WHY COPY UNDER THE LOCK
-----------------------
publication_recipients() returns a snapshot, then the gateway sends after the
lock is released. Holding Hub's lock while a socket blocks would prevent
every unrelated client from making progress. The same rule applies to failed
and expired calls: Hub returns records; the gateway sends their responses.

PYTHON NOTES
------------
- Sets make repeated subscribe/unsubscribe operations harmless.
- discard(value) removes a set member if present; missing members are fine.
- pop(key, None) removes a dictionary entry without failing when it is absent.
- ``provider is client`` compares object identity: connections are distinct
  even when their clients happen to use identical JSON request ids.
- RLock can be acquired again by its owner. disconnect() calls unsubscribe()
  while already holding it; replacing this with Lock would deadlock.
- A monotonic deadline measures elapsed time without wall-clock adjustments.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any


@dataclass
class PendingCall:
    """Remember the original caller while an external provider handles a call.

    The provider receives a new gateway-generated id. caller_id preserves the
    original opaque JSON value, which can even be a list or object. deadline
    is an absolute time.monotonic() timestamp for failing an unanswered call.
    """

    caller: Any
    caller_id: Any
    service: str
    deadline: float


class Hub:
    """Registry shared by all connected clients."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._subscribers: dict[str, set[Any]] = {}
        self._providers: dict[str, Any] = {}
        self._pending: dict[tuple[Any, str], PendingCall] = {}

    def subscribe(self, client: Any, topic: str) -> None:
        """Record the subscription both by topic and on its owning client.

        The topic index supports publication; the client set supports cleanup
        without scanning every topic when that one connection disappears.
        """
        with self._lock:
            if topic not in self._subscribers:
                self._subscribers[topic] = set()
            self._subscribers[topic].add(client)
            client.subscriptions.add(topic)

    def unsubscribe(self, client: Any, topic: str) -> None:
        """Remove both subscription records and discard empty topic entries."""
        with self._lock:
            subscribers = self._subscribers.get(topic)
            if subscribers is not None:
                subscribers.discard(client)
                if len(subscribers) == 0:
                    self._subscribers.pop(topic, None)
            client.subscriptions.discard(topic)

    def publication_recipients(self, topic: str) -> list[Any]:
        """Return a snapshot that callers can use after releasing Hub's lock."""
        with self._lock:
            current = self._subscribers.get(topic, set())
            return list(current)

    def advertise_service(self, client: Any, service: str) -> None:
        """Make this connection the sole provider for future calls to service.

        Remove the old owner's bookkeeping as well as replacing discovery.
        Pending calls still refer to the provider that originally received them.
        """
        with self._lock:
            previous = self._providers.get(service)
            if previous is not None:
                previous.services.discard(service)
            self._providers[service] = client
            client.services.add(service)

    def unadvertise_service(self, client: Any, service: str) -> None:
        """Withdraw service only when this connection is still its provider.

        An older provider cannot remove a newer provider's advertisement.
        """
        with self._lock:
            if self._providers.get(service) is client:
                self._providers.pop(service, None)
            client.services.discard(service)

    def provider_for(self, service: str) -> Any | None:
        """Look up the current provider, using the Project 1/2 helper interface.

        The gateway uses begin_call() when it also needs to register a call:
        lookup alone cannot protect that later registration from disconnects.
        """
        with self._lock:
            return self._providers.get(service)

    def add_pending_call(self, provider: Any, provider_id: str, call: PendingCall) -> None:
        """Record an already selected provider using the Project 1/2 interface.

        The connection and generated id together form the key. A response from
        any other connection must never complete this caller's request.
        """
        with self._lock:
            self._pending[(provider, provider_id)] = call

    def begin_call(self, service: str, provider_id: str, call: PendingCall) -> Any | None:
        """Select a live provider and record the call in one atomic operation.

        This combines the older provider_for()/add_pending_call() sequence.
        Selecting and recording separately could lose a disconnect between
        the two operations, leaving the caller waiting until a timeout.

        The gateway marks a connection closed before asking Hub to remove its
        records. If closing wins the race, refuse the call here. If registering
        wins, disconnect() will find and fail the pending call. getattr's
        default also permits the simple client objects used by local callers.
        """
        with self._lock:
            provider = self._providers.get(service)
            if provider is None or getattr(provider, "_closed", False):
                return None
            if getattr(call.caller, "_closed", False):
                return None
            self._pending[(provider, provider_id)] = call
            return provider

    def take_pending_call(self, provider: Any, provider_id: str,
                          service: str | None = None) -> PendingCall | None:
        """Remove and return one matching response route, or return None.

        Timeout or an earlier response may already have removed it. When a
        service is supplied, a mismatched response leaves the real call intact.
        The optional argument preserves the original Project 1/2 interface.
        """
        with self._lock:
            key = (provider, provider_id)
            pending = self._pending.get(key)
            if pending is None:
                return None
            if service is not None and pending.service != service:
                return None
            return self._pending.pop(key)

    def expired_calls(self, now: float | None = None) -> list[PendingCall]:
        """Remove and return calls whose deadlines are at or before now.

        Collect keys before removing them: changing a dictionary's size during
        iteration is an error. Removing each record also prevents a late
        provider response from sending a second result to its caller.
        """
        if now is None:
            now = time.monotonic()

        expired: list[PendingCall] = []
        with self._lock:
            keys_to_remove = []
            for key, pending in self._pending.items():
                if pending.deadline <= now:
                    keys_to_remove.append(key)
            for key in keys_to_remove:
                expired.append(self._pending.pop(key))
        return expired

    def disconnect(self, client: Any) -> list[PendingCall]:
        """Forget a connection and return other callers that need failure replies.

        Calls made by the departing client are discarded silently: there is
        nobody left to receive them. Calls waiting on its service provider are
        returned so the gateway can report failure after releasing the lock.
        Reader and writer cleanup may both invoke this; removing twice is safe.
        """
        with self._lock:
            # unsubscribe() changes this set, so iterate over a separate list.
            for topic in list(client.subscriptions):
                self.unsubscribe(client, topic)

            for service in list(client.services):
                if self._providers.get(service) is client:
                    self._providers.pop(service, None)
            client.services.clear()
            client.publishers.clear()

            failed: list[PendingCall] = []
            for key, pending in list(self._pending.items()):
                provider, _provider_id = key
                caller_left = pending.caller is client
                provider_left = provider is client
                if not caller_left and not provider_left:
                    continue
                self._pending.pop(key)
                if provider_left and not caller_left:
                    failed.append(pending)
            return failed
