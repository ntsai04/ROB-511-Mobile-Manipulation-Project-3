"""Thread-safe registry for rosbridge topics and services.

The hub has no socket knowledge.  It records subscriptions, service providers,
and in-flight forwarded calls so the gateway can route messages without
holding a lock while writing to a client.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any


@dataclass
class PendingCall:
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
        with self._lock:
            self._subscribers.setdefault(topic, set()).add(client)
            client.subscriptions.add(topic)

    def unsubscribe(self, client: Any, topic: str) -> None:
        with self._lock:
            subscribers = self._subscribers.get(topic)
            if subscribers is not None:
                subscribers.discard(client)
                if not subscribers:
                    self._subscribers.pop(topic, None)
            client.subscriptions.discard(topic)

    def publication_recipients(self, topic: str) -> list[Any]:
        with self._lock:
            return list(self._subscribers.get(topic, ()))

    def advertise_service(self, client: Any, service: str) -> None:
        with self._lock:
            previous = self._providers.get(service)
            if previous is not None:
                previous.services.discard(service)
            self._providers[service] = client
            client.services.add(service)

    def unadvertise_service(self, client: Any, service: str) -> None:
        with self._lock:
            if self._providers.get(service) is client:
                self._providers.pop(service, None)
            client.services.discard(service)

    def provider_for(self, service: str) -> Any | None:
        with self._lock:
            return self._providers.get(service)

    def add_pending_call(self, provider: Any, provider_id: str, call: PendingCall) -> None:
        with self._lock:
            self._pending[(provider, provider_id)] = call

    def begin_call(self, service: str, provider_id: str, call: PendingCall) -> Any | None:
        """Select a live provider and record the call in one atomic operation.

        Selecting and recording separately could lose a disconnect between
        the two operations, leaving the caller waiting until a timeout.
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
        with self._lock:
            key = (provider, provider_id)
            call = self._pending.get(key)
            if call is None or (service is not None and call.service != service):
                return None
            return self._pending.pop(key)

    def expired_calls(self, now: float | None = None) -> list[PendingCall]:
        now = time.monotonic() if now is None else now
        with self._lock:
            expired_keys = [
                key for key, call in self._pending.items()
                if call.deadline <= now
            ]
            expired = [self._pending.pop(key) for key in expired_keys]
            return expired

    def disconnect(self, client: Any) -> list[PendingCall]:
        with self._lock:
            for topic in tuple(client.subscriptions):
                self.unsubscribe(client, topic)

            for service in tuple(client.services):
                if self._providers.get(service) is client:
                    self._providers.pop(service, None)
            client.services.clear()
            client.publishers.clear()

            failed = []
            for key, call in list(self._pending.items()):
                provider, _ = key
                caller_left = call.caller is client
                provider_left = provider is client
                if caller_left or provider_left:
                    self._pending.pop(key)
                    if provider_left and not caller_left:
                        failed.append(call)
            return failed
