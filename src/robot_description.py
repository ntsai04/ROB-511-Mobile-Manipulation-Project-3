"""Parse and validate the flat URDF descriptions used by Project 3.

Parsing produces an independent model; callers can keep their previous model
when ``RobotDescriptionError`` is raised.  Only direct URDF children are read,
so plugin, transmission, collision and inertial data have no kinematic effect.
"""

from collections import deque
from dataclasses import dataclass
import math
import re
import xml.etree.ElementTree as ET

from transforms import (
    IDENTITY_QUATERNION,
    ZERO_TRANSLATION,
    Quaternion,
    Vector3,
    compose_transforms,
    quaternion_from_axis_angle,
    quaternion_from_rpy,
)


class RobotDescriptionError(ValueError):
    """The candidate description cannot safely replace the loaded robot."""


@dataclass(frozen=True)
class Visual:
    shape: str
    attributes: dict[str, object]
    origin_xyz: Vector3 = ZERO_TRANSLATION
    origin_quaternion: Quaternion = IDENTITY_QUATERNION


@dataclass(frozen=True)
class Link:
    name: str
    visual: Visual | None = None


@dataclass(frozen=True)
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin_xyz: Vector3 = ZERO_TRANSLATION
    origin_quaternion: Quaternion = IDENTITY_QUATERNION
    axis: Vector3 = (1.0, 0.0, 0.0)
    lower: float = 0.0
    upper: float = 0.0

    def joint_transform(self, position: float = 0.0) -> tuple[Vector3, Quaternion]:
        """Return ``T_origin * T_motion(position)`` without clamping limits."""
        if self.type == "fixed" or position == 0.0:
            return self.origin_xyz, self.origin_quaternion
        if self.type == "prismatic":
            motion_xyz = tuple(position * component for component in self.axis)
            return compose_transforms(
                self.origin_xyz, self.origin_quaternion,
                motion_xyz, IDENTITY_QUATERNION,
            )
        return compose_transforms(
            self.origin_xyz, self.origin_quaternion, ZERO_TRANSLATION,
            quaternion_from_axis_angle(self.axis, position),
        )


@dataclass(frozen=True)
class Robot:
    name: str
    root_link: str
    links: dict[str, Link]
    joints: list[Joint]


_DECIMAL = re.compile(r"[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?")
_XML_WHITESPACE = " \t\r\n"
_JOINT_TYPES = frozenset(("revolute", "continuous", "prismatic", "fixed"))


def _number(text: str, label: str) -> float:
    token = text.strip(_XML_WHITESPACE)
    if _DECIMAL.fullmatch(token) is None:
        raise RobotDescriptionError(f"{label} must be a decimal number")
    value = float(token)
    if not math.isfinite(value):
        raise RobotDescriptionError(f"{label} must be finite")
    return value


def _vector(text: str, label: str) -> Vector3:
    tokens = re.split(r"[ \t\r\n]+", text.strip(_XML_WHITESPACE))
    if len(tokens) != 3:
        raise RobotDescriptionError(f"{label} must have exactly three numbers")
    return tuple(_number(token, label) for token in tokens)


def _required(element: ET.Element, attribute: str, label: str) -> str:
    value = element.get(attribute)
    if value is None:
        raise RobotDescriptionError(f"{label} requires {attribute}")
    return value


def _origin(element: ET.Element | None) -> tuple[Vector3, Quaternion]:
    if element is None:
        return ZERO_TRANSLATION, IDENTITY_QUATERNION
    xyz = _vector(element.get("xyz", "0 0 0"), "origin xyz")
    rpy = _vector(element.get("rpy", "0 0 0"), "origin rpy")
    return xyz, quaternion_from_rpy(*rpy)


def _visual(element: ET.Element, namespace: str) -> Visual:
    geometry = element.find(namespace + "geometry")
    if geometry is None:
        raise RobotDescriptionError("visual requires geometry")
    shape_element = next((child for child in geometry if child.tag in {
        namespace + "box", namespace + "cylinder", namespace + "sphere", namespace + "mesh"
    }), None)
    if shape_element is None:
        raise RobotDescriptionError("visual geometry requires a supported shape")
    shape = shape_element.tag[len(namespace):]
    attributes: dict[str, object] = {}
    if shape == "box":
        attributes["size"] = _vector(_required(shape_element, "size", "box"), "box size")
    elif shape == "cylinder":
        for attribute in ("radius", "length"):
            attributes[attribute] = _number(
                _required(shape_element, attribute, "cylinder"), "cylinder " + attribute
            )
    elif shape == "sphere":
        attributes["radius"] = _number(
            _required(shape_element, "radius", "sphere"), "sphere radius"
        )
    else:
        attributes["filename"] = _required(shape_element, "filename", "mesh")
        attributes["scale"] = _vector(shape_element.get("scale", "1 1 1"), "mesh scale")
    xyz, quaternion = _origin(element.find(namespace + "origin"))
    return Visual(shape, attributes, xyz, quaternion)


def parse_robot_description(value: object) -> Robot:
    """Load one complete URDF tree, or raise a descriptive validation error.

    The traversal is iterative, so even a long serial arm does not depend on
    Python's recursion limit.  Ambiguous graphs (duplicates, cycles and links
    with multiple parents) are rejected instead of risking a traversal loop.
    """
    if not isinstance(value, str) or not value.strip():
        raise RobotDescriptionError("robot_description must be a nonempty XML string")
    try:
        # ElementTree otherwise rejects a declaration after leading whitespace.
        root = ET.fromstring(value.lstrip("\ufeff" + _XML_WHITESPACE))
    except (ET.ParseError, ValueError) as error:
        raise RobotDescriptionError(f"invalid XML: {error}") from error
    namespace = ""
    if root.tag.startswith("{"):
        namespace = root.tag[:root.tag.index("}") + 1]
    if root.tag != namespace + "robot":
        raise RobotDescriptionError("root element must be robot")

    links: dict[str, Link] = {}
    for element in root.findall(namespace + "link"):
        name = _required(element, "name", "link")
        if not name.strip() or name in links:
            raise RobotDescriptionError("link names must be nonempty and unique")
        visual_element = element.find(namespace + "visual")
        visual = None if visual_element is None else _visual(visual_element, namespace)
        links[name] = Link(name, visual)
    if not links:
        raise RobotDescriptionError("robot requires at least one direct-child link")

    joints: list[Joint] = []
    joint_names: set[str] = set()
    child_names: set[str] = set()
    children: dict[str, list[str]] = {name: [] for name in links}
    for element in root.findall(namespace + "joint"):
        name = _required(element, "name", "joint")
        kind = _required(element, "type", "joint")
        if not name.strip() or name in joint_names:
            raise RobotDescriptionError("joint names must be nonempty and unique")
        joint_names.add(name)
        if kind not in _JOINT_TYPES:
            raise RobotDescriptionError(f"unsupported joint type: {kind}")
        parent_element = element.find(namespace + "parent")
        child_element = element.find(namespace + "child")
        if parent_element is None or child_element is None:
            raise RobotDescriptionError(f"joint {name} requires parent and child")
        parent = _required(parent_element, "link", "parent")
        child = _required(child_element, "link", "child")
        if parent not in links or child not in links:
            raise RobotDescriptionError(f"joint {name} references an unknown link")
        if parent == child or child in child_names:
            raise RobotDescriptionError("each child link must have exactly one distinct parent")
        child_names.add(child)
        children[parent].append(child)
        xyz, quaternion = _origin(element.find(namespace + "origin"))
        axis_element = element.find(namespace + "axis")
        axis = (1.0, 0.0, 0.0) if axis_element is None else _vector(
            axis_element.get("xyz", "1 0 0"), "axis xyz"
        )
        if kind != "fixed" and math.hypot(*axis) == 0.0:
            raise RobotDescriptionError("movable joint axis must have nonzero length")
        lower = upper = 0.0
        if kind in ("revolute", "prismatic"):
            limit = element.find(namespace + "limit")
            if limit is None:
                raise RobotDescriptionError(f"joint {name} requires a limit")
            lower = _number(limit.get("lower", "0"), "limit lower")
            upper = _number(limit.get("upper", "0"), "limit upper")
        joints.append(Joint(name, kind, parent, child, xyz, quaternion, axis, lower, upper))

    roots = links.keys() - child_names
    if len(roots) != 1:
        raise RobotDescriptionError("robot must have exactly one root link")
    root_link = next(iter(roots))
    queue = deque([root_link])
    visited: set[str] = set()
    while queue:
        name = queue.popleft()
        if name in visited:
            raise RobotDescriptionError("robot contains a cycle")
        visited.add(name)
        queue.extend(children[name])
    if len(visited) != len(links):
        raise RobotDescriptionError("all links must form one connected tree")
    return Robot(root.get("name", "robot"), root_link, links, joints)
