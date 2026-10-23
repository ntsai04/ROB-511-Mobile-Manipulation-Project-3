"""The parameter server: named JSON values with independent version counters.

HOW TO READ THIS FILE
---------------------
This node follows the service-node pattern used by Project 2's ArmSimNode:
service_handlers() lists the public names, and each handler returns
(result, values, status). The gateway adds op, service and id to the reply.

The entire database is one dictionary:

    parameter name -> (value, version)

A parameter that has never been set has version 0. Each successful set adds
one to THAT NAME'S version, even if the value did not change. The server does
not know what URDF means: storing a description and accepting a robot are two
different jobs. RobotStatePublisher performs the latter job.

Reader threads can call these methods concurrently. Holding the lock across
read/increment/write prevents two simultaneous sets from using one version.
"""

from __future__ import annotations

import threading
from typing import Any


class ParameterServer:
    def __init__(self) -> None:
        # Keep the value and version together so get_param reads one snapshot.
        self._parameters: dict[str, tuple[Any, int]] = {}
        self._lock = threading.Lock()

    def service_handlers(self) -> dict:
        """The same name -> bound-method table used by Project 2's nodes."""
        return {
            "/param_server/set_param": self.set_param,
            "/param_server/get_param": self.get_param,
        }

    def set_param(self, args: Any) -> tuple[bool, dict, str]:
        """Store arbitrary JSON; a missing value intentionally stores None/null."""
        if not isinstance(args, dict):
            return False, {}, "name must be a string"
        name = args.get("name")
        if not isinstance(name, str):
            return False, {}, "name must be a string"

        with self._lock:
            previous_version = 0
            if name in self._parameters:
                previous_value, previous_version = self._parameters[name]
            new_version = previous_version + 1

            # Each TCP request owns its decoded JSON. Stored values are never
            # mutated: a later set replaces the whole value. Local consumers
            # also treat service values as read-only. Keeping that ownership
            # avoids deepcopy's lower recursion limit on valid nested JSON.
            value = args.get("value")
            self._parameters[name] = (value, new_version)

        # True / values / empty error is the usual Project 2 service triple.
        return True, {"version": new_version}, ""

    def get_param(self, args: Any) -> tuple[bool, dict, str]:
        """Distinguish an unset name from a name explicitly set to JSON null."""
        missing = {"value": None, "version": 0}
        if not isinstance(args, dict):
            return False, missing, "name must be a string"
        name = args.get("name")
        if not isinstance(name, str):
            return False, missing, "name must be a string"

        with self._lock:
            if name not in self._parameters:
                return False, missing, "parameter has not been set"
            value, version = self._parameters[name]
            return True, {"value": value, "version": version}, ""
