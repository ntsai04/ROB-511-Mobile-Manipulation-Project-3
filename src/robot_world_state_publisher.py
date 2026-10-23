"""World state publisher: the latest /tf tree + root pose -> every world pose.

HOW TO READ THIS FILE
---------------------
The input /tf message contains LOCAL parent-to-child edges. It does not
already contain poses relative to the root or the world. This node does the
matrix-stack traversal from lecture using quaternion/translation pairs:

    T_global_child = T_global_parent * T_parent_child

_read_tree() interprets the complete message and finds its root.
_compose_tree() walks every branch and emits the column-major 4x4 matrices.
The root starts with /global_pose, or identity if no pose has arrived.

There are two independent inputs. Remember the tree so a new global pose can
move it without a new /tf; remember the pose so it survives a robot reload.
Cache the resulting output so update() can keep publishing at 10 Hz even
when neither input changes. Callbacks replace complete snapshots under a
lock; publication happens after releasing that lock, as in Project 2.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading

from transforms import (
    IDENTITY_QUATERNION,
    ZERO_TRANSLATION,
    compose_transforms,
    matrix_column_major,
    quaternion_normalize,
)


@dataclass(frozen=True)
class TransformTree:
    """One /tf message, indexed by parent frame for an iterative tree walk."""

    root_link: str | None
    stamp: dict
    children: dict[str, list]
    link_count: int


class RobotWorldStatePublisher:
    def __init__(self, publish) -> None:
        self._publish = publish
        self._message: dict | None = None
        self._tree: TransformTree | None = None
        self._global_xyz = ZERO_TRANSLATION
        self._global_quaternion = IDENTITY_QUATERNION
        self._lock = threading.Lock()

    def accept_tf(self, message: object) -> None:
        """Replace the ENTIRE previous tree; no previous robot's edges remain."""
        with self._lock:
            try:
                candidate_tree = self._read_tree(message)
                candidate_message = self._compose_tree(
                    candidate_tree, self._global_xyz, self._global_quaternion,
                )
            except (KeyError, TypeError, ValueError, OverflowError):
                # Compute before assigning: malformed input leaves the last
                # usable tree and its output intact.
                return
            self._tree = candidate_tree
            self._message = candidate_message

    def accept_global_pose(self, message: object) -> None:
        """Replace the root's pose while preserving all local /tf edges."""
        try:
            position = message["position"]
            orientation = message["orientation"]
            translation = (position["x"], position["y"], position["z"])
            quaternion = (orientation["x"], orientation["y"], orientation["z"], orientation["w"])
            for value in translation + quaternion:
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise ValueError("global pose must contain numbers")
                if not math.isfinite(value):
                    raise ValueError("global pose must contain finite numbers")
            quaternion = quaternion_normalize(quaternion)
        except (KeyError, TypeError, ValueError, OverflowError):
            return

        with self._lock:
            self._global_xyz = translation
            self._global_quaternion = quaternion
            # A pose may arrive before the first robot. Keep it anyway and
            # apply it when /tf eventually supplies a tree.
            if self._tree is not None:
                self._message = self._compose_tree(self._tree, translation, quaternion)

    @staticmethod
    def _read_tree(message: dict) -> TransformTree:
        """Build parent -> outgoing edges and identify the sole root frame."""
        entries = message["transforms"]
        if not isinstance(entries, list):
            raise ValueError("transforms must be an array")
        if not entries:
            # An empty /tf cannot name a one-link robot's root. Its world
            # output is unspecified by the course, so an empty output is OK.
            return TransformTree(None, {"sec": 0, "nanosec": 0}, {}, 0)

        received_stamp = entries[0]["header"]["stamp"]
        stamp = {"sec": received_stamp["sec"], "nanosec": received_stamp["nanosec"]}
        for component in stamp.values():
            if isinstance(component, bool) or not isinstance(component, int):
                raise ValueError("stamp components must be integers")

        parent_names = set()
        child_names = set()
        children_by_parent = {}
        for entry in entries:
            parent = entry["header"]["frame_id"]
            child = entry["child_frame_id"]
            if not isinstance(parent, str) or not isinstance(child, str):
                raise ValueError("frame names must be strings")
            if child in child_names:
                raise ValueError("a child frame has more than one parent")
            parent_names.add(parent)
            child_names.add(child)

            transform = entry["transform"]
            position = transform["translation"]
            orientation = transform["rotation"]
            translation = (position["x"], position["y"], position["z"])
            quaternion = (orientation["x"], orientation["y"], orientation["z"], orientation["w"])
            for value in translation + quaternion:
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise ValueError("transform must contain numbers")
                if not math.isfinite(value):
                    raise ValueError("transform must contain finite numbers")
            quaternion = quaternion_normalize(quaternion)

            if parent not in children_by_parent:
                children_by_parent[parent] = []
            children_by_parent[parent].append((child, translation, quaternion))

        # Every non-root link is a child of exactly one joint. The root is
        # therefore a parent name which never appears as a child name. Do not
        # assume the first edge or the first URDF link names the root.
        roots = parent_names - child_names
        if len(roots) != 1:
            raise ValueError("/tf must form one tree")
        root_link = next(iter(roots))
        all_link_names = parent_names | child_names
        return TransformTree(root_link, stamp, children_by_parent, len(all_link_names))

    @staticmethod
    def _compose_tree(tree: TransformTree, root_xyz: tuple, root_quaternion: tuple) -> dict:
        """Depth-first traversal, carrying the parent's WORLD transform on a stack."""
        if tree.root_link is None:
            return {"transforms": []}

        # Each stack entry already contains the pose of that link IN THE WORLD.
        # Separate entries keep siblings from accidentally sharing an updated
        # accumulator. This is the iterative version of push/recurse/pop.
        stack = [(tree.root_link, root_xyz, root_quaternion)]
        visited = set()
        output = []
        while stack:
            link_name, world_xyz, world_quaternion = stack.pop()
            if link_name in visited:
                raise ValueError("cycle in /tf")
            visited.add(link_name)

            # Emit the root too: it has no /tf edge of its own, but does have
            # a world pose. The source stamp is copied, never replaced by now.
            output.append({
                "header": {"stamp": dict(tree.stamp), "frame_id": "global_frame"},
                "child_frame_id": link_name,
                "matrix": matrix_column_major(world_xyz, world_quaternion),
            })

            outgoing_edges = tree.children.get(link_name, [])
            for child_name, local_xyz, local_quaternion in outgoing_edges:
                # compose_transforms rotates the local translation by the
                # parent's rotation BEFORE adding its world translation.
                child_world_xyz, child_world_quaternion = compose_transforms(
                    world_xyz, world_quaternion, local_xyz, local_quaternion,
                )
                stack.append((child_name, child_world_xyz, child_world_quaternion))

        if len(visited) != tree.link_count:
            raise ValueError("disconnected /tf")
        return {"transforms": output}

    def update(self) -> None:
        """Keep complete snapshots arriving, even without new inputs."""
        with self._lock:
            message = self._message
        if message is not None:
            self._publish("/xform_world", message)
