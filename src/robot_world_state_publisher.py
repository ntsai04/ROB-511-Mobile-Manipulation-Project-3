"""Compose only the latest /tf tree into column-major world matrices."""

from __future__ import annotations

from copy import deepcopy
import math
import threading

from transforms import compose_transforms, matrix_column_major, quaternion_normalize


class RobotWorldStatePublisher:
    def __init__(self, publish) -> None:
        self._publish = publish
        self._message: dict | None = None
        self._lock = threading.Lock()

    def accept_tf(self, message: object) -> None:
        """Rebuild from one complete message; never retain previous edges.

        Iterative traversal also bounds work for deep trees. Invalid external
        /tf inputs are ignored rather than disturbing the current snapshot.
        """
        try:
            replacement = self._compose(message)
        except (KeyError, TypeError, ValueError, OverflowError):
            return
        with self._lock:
            self._message = replacement

    @staticmethod
    def _compose(message: dict) -> dict:
        entries = message["transforms"]
        if not isinstance(entries, list):
            raise ValueError("transforms must be an array")
        if not entries:
            # No edge names the sole root of a one-link robot. The wire
            # contract leaves this particular /xform_world unspecified.
            return {"transforms": []}
        children = set()
        parents = set()
        adjacency: dict[str, list] = {}
        stamp = deepcopy(entries[0]["header"]["stamp"])
        for entry in entries:
            parent = entry["header"]["frame_id"]
            child = entry["child_frame_id"]
            if not isinstance(parent, str) or not isinstance(child, str) or child in children:
                raise ValueError("invalid or repeated child frame")
            children.add(child)
            parents.add(parent)
            transform = entry["transform"]
            xyz = tuple(transform["translation"][key] for key in ("x", "y", "z"))
            quat = tuple(transform["rotation"][key] for key in ("x", "y", "z", "w"))
            if any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in xyz + quat):
                raise ValueError("transform must contain finite numbers")
            quat = quaternion_normalize(quat)
            adjacency.setdefault(parent, []).append((child, xyz, quat))
        roots = parents - children
        if len(roots) != 1:
            raise ValueError("/tf must form one tree")
        root = next(iter(roots))
        stack = [(root, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))]
        visited = set()
        output = []
        while stack:
            link, xyz, quat = stack.pop()
            if link in visited:
                raise ValueError("cycle in /tf")
            visited.add(link)
            output.append({
                "header": {"stamp": stamp, "frame_id": "global_frame"},
                "child_frame_id": link,
                "matrix": matrix_column_major(xyz, quat),
            })
            for child, child_xyz, child_quat in adjacency.get(link, ()):
                next_xyz, next_quat = compose_transforms(xyz, quat, child_xyz, child_quat)
                stack.append((child, next_xyz, next_quat))
        if visited != parents | children:
            raise ValueError("disconnected /tf")
        return {"transforms": output}

    def update(self) -> None:
        with self._lock:
            message = self._message
        if message is not None:
            self._publish("/xform_world", message)
