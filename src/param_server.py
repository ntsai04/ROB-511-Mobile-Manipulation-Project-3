"""Versioned JSON parameters exposed through the two parameter services."""

from __future__ import annotations

import threading
from typing import Any


class ParameterServer:
    def __init__(self) -> None:
        self._parameters: dict[str, tuple[Any, int]] = {}
        self._lock = threading.Lock()

    def service_handlers(self) -> dict:
        return {
            "/param_server/set_param": self.set_param,
            "/param_server/get_param": self.get_param,
        }

    def set_param(self, args: Any) -> tuple[bool, dict, str]:
        if not isinstance(args, dict) or not isinstance(args.get("name"), str):
            return False, {}, "name must be a string"
        name = args["name"]
        with self._lock:
            version = self._parameters.get(name, (None, 0))[1] + 1
            # No interpretation here: even a broken robot description is stored.
            # Each TCP request owns its decoded JSON. Stored values are never
            # mutated: a later set replaces the whole value. Local consumers
            # also treat service values as read-only. Keeping that ownership
            # avoids deepcopy's lower recursion limit on valid nested JSON.
            self._parameters[name] = (args.get("value"), version)
        return True, {"version": version}, ""

    def get_param(self, args: Any) -> tuple[bool, dict, str]:
        missing = {"value": None, "version": 0}
        if not isinstance(args, dict) or not isinstance(args.get("name"), str):
            return False, missing, "name must be a string"
        with self._lock:
            if args["name"] not in self._parameters:
                return False, missing, "parameter has not been set"
            value, version = self._parameters[args["name"]]
            return True, {"value": value, "version": version}, ""
