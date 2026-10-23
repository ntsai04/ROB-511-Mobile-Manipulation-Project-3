"""Read a flat URDF into the robot model used by the Project 3 nodes.

HOW THE DESCRIPTION BECOMES A TREE
--------------------------------
1. Parse XML and check that its root is <robot>.
2. Read the robot's direct-child <link> elements, including their first visual.
3. Read its direct-child <joint> elements and resolve their parent/child links.
4. Find the sole root link and verify that every link belongs to one tree.
5. Return the complete model. Until then, the caller's loaded robot is untouched.

Only direct children count: a <joint> hidden inside <transmission> or <gazebo>
is not a kinematic joint. Unknown elements, collisions and inertials are ignored.
ElementTree handles XML syntax; the functions here interpret URDF semantics.
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
    """Display geometry and its pose inside a link; this never changes FK."""

    shape: str
    attributes: dict[str, object]
    origin_xyz: Vector3 = ZERO_TRANSLATION
    origin_quaternion: Quaternion = IDENTITY_QUATERNION


@dataclass(frozen=True)
class Link:
    """One named robot frame, optionally with geometry to draw in that frame."""

    name: str
    visual: Visual | None = None


@dataclass(frozen=True)
class Joint:
    """One parent-to-child edge, with a fixed origin and optional joint motion."""

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
        # At zero configuration, every kind of motion is identity. A fixed
        # joint stays at its origin for all supplied positions.
        if self.type == "fixed" or position == 0.0:
            return self.origin_xyz, self.origin_quaternion

        # Motion is measured in the joint frame AFTER the origin transform.
        # A prismatic axis keeps its written length: axis (2, 0, 0) and q=.7
        # give a translation of (1.4, 0, 0) in that joint frame.
        if self.type == "prismatic":
            motion_x = position * self.axis[0]
            motion_y = position * self.axis[1]
            motion_z = position * self.axis[2]
            motion_xyz = (motion_x, motion_y, motion_z)
            motion_quaternion = IDENTITY_QUATERNION
        else:
            # Revolute and continuous joints rotate about a normalized axis.
            # Their angle is used exactly as received, without applying limits.
            motion_xyz = ZERO_TRANSLATION
            motion_quaternion = quaternion_from_axis_angle(self.axis, position)

        # Putting origin first rotates a prismatic displacement into the parent
        # frame and makes a revolute joint turn about its own mounted origin.
        return compose_transforms(
            self.origin_xyz, self.origin_quaternion, motion_xyz, motion_quaternion,
        )


@dataclass(frozen=True)
class Robot:
    """A fully validated description, with joints retained in document order."""

    name: str
    root_link: str
    links: dict[str, Link]
    joints: list[Joint]


_DECIMAL = re.compile(r"[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?")
_XML_WHITESPACE = " \t\r\n"
_JOINT_TYPES = frozenset(("revolute", "continuous", "prismatic", "fixed"))


def _parse_number(text: str, label: str) -> float:
    """Accept ordinary finite decimal notation, including exponents."""
    token = text.strip(_XML_WHITESPACE)
    # float() alone would also accept spellings such as 1_000 and nan. The
    # course's URDF grammar allows neither; check the spelling before conversion.
    if _DECIMAL.fullmatch(token) is None:
        raise RobotDescriptionError(f"{label} must be a decimal number")
    value = float(token)
    if not math.isfinite(value):
        raise RobotDescriptionError(f"{label} must be finite")
    return value


def _parse_vector(text: str, label: str) -> Vector3:
    """Read exactly three decimal numbers separated by XML whitespace."""
    stripped_text = text.strip(_XML_WHITESPACE)
    tokens = re.split(r"[ \t\r\n]+", stripped_text)
    if len(tokens) != 3:
        raise RobotDescriptionError(f"{label} must have exactly three numbers")
    x = _parse_number(tokens[0], label)
    y = _parse_number(tokens[1], label)
    z = _parse_number(tokens[2], label)
    return x, y, z


def _require_attribute(element: ET.Element, attribute: str, label: str) -> str:
    """Read an attribute whose absence is an error, rather than a URDF default."""
    value = element.get(attribute)
    if value is None:
        raise RobotDescriptionError(f"{label} requires {attribute}")
    return value


def _parse_origin(origin_element: ET.Element | None) -> tuple[Vector3, Quaternion]:
    """Read a pose, with independent zero defaults for xyz and rpy."""
    if origin_element is None:
        return ZERO_TRANSLATION, IDENTITY_QUATERNION

    xyz_text = origin_element.get("xyz", "0 0 0")
    rpy_text = origin_element.get("rpy", "0 0 0")
    xyz = _parse_vector(xyz_text, "origin xyz")
    roll, pitch, yaw = _parse_vector(rpy_text, "origin rpy")
    quaternion = quaternion_from_rpy(roll, pitch, yaw)
    return xyz, quaternion


def _parse_visual(visual_element: ET.Element, namespace: str) -> Visual:
    """Validate one visual's supported shape, dimensions and optional origin."""
    geometry = visual_element.find(namespace + "geometry")
    if geometry is None:
        raise RobotDescriptionError("visual requires geometry")
    supported_shapes = {
        namespace + "box", namespace + "cylinder", namespace + "sphere", namespace + "mesh"
    }
    shape_element = None
    for child_element in geometry:
        if child_element.tag in supported_shapes:
            shape_element = child_element
            break
    if shape_element is None:
        raise RobotDescriptionError("visual geometry requires a supported shape")
    shape = shape_element.tag[len(namespace):]
    attributes: dict[str, object] = {}

    # Each shape has different required attributes. Keep these cases explicit
    # so a missing dimension is easy to distinguish from an invalid number.
    if shape == "box":
        size_text = _require_attribute(shape_element, "size", "box")
        attributes["size"] = _parse_vector(size_text, "box size")
    elif shape == "cylinder":
        radius_text = _require_attribute(shape_element, "radius", "cylinder")
        length_text = _require_attribute(shape_element, "length", "cylinder")
        attributes["radius"] = _parse_number(radius_text, "cylinder radius")
        attributes["length"] = _parse_number(length_text, "cylinder length")
    elif shape == "sphere":
        radius_text = _require_attribute(shape_element, "radius", "sphere")
        attributes["radius"] = _parse_number(radius_text, "sphere radius")
    else:
        attributes["filename"] = _require_attribute(shape_element, "filename", "mesh")
        scale_text = shape_element.get("scale", "1 1 1")
        attributes["scale"] = _parse_vector(scale_text, "mesh scale")

    origin_element = visual_element.find(namespace + "origin")
    xyz, quaternion = _parse_origin(origin_element)
    return Visual(shape, attributes, xyz, quaternion)


def _parse_robot_xml(value: object) -> tuple[ET.Element, str]:
    """Check the parameter type, XML syntax, robot root and default namespace."""
    if not isinstance(value, str) or not value.strip():
        raise RobotDescriptionError("robot_description must be a nonempty XML string")
    try:
        # Real descriptions may have whitespace before an XML declaration.
        # ElementTree requires that declaration to start at the first character.
        xml_text = value.lstrip("\ufeff" + _XML_WHITESPACE)
        root = ET.fromstring(xml_text)
    except (ET.ParseError, ValueError) as error:
        raise RobotDescriptionError(f"invalid XML: {error}") from error
    namespace = ""
    # With a default xmlns, ElementTree spells tags as "{namespace}robot".
    # Use the same prefix for direct-child lookups so that such URDFs still load.
    if root.tag.startswith("{"):
        namespace = root.tag[:root.tag.index("}") + 1]
    if root.tag != namespace + "robot":
        raise RobotDescriptionError("root element must be robot")
    return root, namespace


def _parse_links(robot_element: ET.Element, namespace: str) -> dict[str, Link]:
    """Collect all links before reading joints, regardless of document order."""
    links: dict[str, Link] = {}

    # findall("link") visits direct children. A recursive search like .//link
    # would incorrectly turn plugin or transmission contents into robot links.
    for link_element in robot_element.findall(namespace + "link"):
        name = _require_attribute(link_element, "name", "link")
        if not name.strip() or name in links:
            raise RobotDescriptionError("link names must be nonempty and unique")
        # find() returns only the first visual. Later visuals have no effect.
        visual_element = link_element.find(namespace + "visual")
        visual = None
        if visual_element is not None:
            visual = _parse_visual(visual_element, namespace)
        links[name] = Link(name, visual)
    if not links:
        raise RobotDescriptionError("robot requires at least one direct-child link")
    return links


def _parse_joint(joint_element: ET.Element, namespace: str, links: dict[str, Link]) -> Joint:
    """Read one joint's endpoints, mounted origin, axis and informational limits."""
    name = _require_attribute(joint_element, "name", "joint")
    joint_type = _require_attribute(joint_element, "type", "joint")
    if not name.strip():
        raise RobotDescriptionError("joint names must be nonempty and unique")
    if joint_type not in _JOINT_TYPES:
        raise RobotDescriptionError(f"unsupported joint type: {joint_type}")

    parent_element = joint_element.find(namespace + "parent")
    child_element = joint_element.find(namespace + "child")
    if parent_element is None or child_element is None:
        raise RobotDescriptionError(f"joint {name} requires parent and child")
    parent = _require_attribute(parent_element, "link", "parent")
    child = _require_attribute(child_element, "link", "child")
    if parent not in links or child not in links:
        raise RobotDescriptionError(f"joint {name} references an unknown link")

    origin_element = joint_element.find(namespace + "origin")
    xyz, quaternion = _parse_origin(origin_element)

    # A missing axis element OR a missing xyz attribute defaults to positive X
    # in the joint frame. Parse written axes even for fixed joints so malformed
    # triples are rejected, but accept a fixed joint's otherwise ignored zero axis.
    axis = (1.0, 0.0, 0.0)
    axis_element = joint_element.find(namespace + "axis")
    if axis_element is not None:
        axis_text = axis_element.get("xyz", "1 0 0")
        axis = _parse_vector(axis_text, "axis xyz")
    if joint_type != "fixed" and math.hypot(*axis) == 0.0:
        raise RobotDescriptionError("movable joint axis must have nonzero length")

    # Only revolute/prismatic joints require limits. Their missing bounds are
    # zero. Continuous/fixed limits are ignored entirely, even if malformed.
    lower = 0.0
    upper = 0.0
    if joint_type in ("revolute", "prismatic"):
        limit_element = joint_element.find(namespace + "limit")
        if limit_element is None:
            raise RobotDescriptionError(f"joint {name} requires a limit")
        lower = _parse_number(limit_element.get("lower", "0"), "limit lower")
        upper = _parse_number(limit_element.get("upper", "0"), "limit upper")

    return Joint(name, joint_type, parent, child, xyz, quaternion, axis, lower, upper)


def _parse_joints(robot_element: ET.Element, namespace: str, links: dict[str, Link]) -> list[Joint]:
    """Collect direct-child joints, preserving the document's entry order."""
    joints: list[Joint] = []
    joint_names: set[str] = set()
    for joint_element in robot_element.findall(namespace + "joint"):
        joint = _parse_joint(joint_element, namespace, links)
        if joint.name in joint_names:
            raise RobotDescriptionError("joint names must be nonempty and unique")
        joint_names.add(joint.name)
        joints.append(joint)
    return joints


def _validate_tree(links: dict[str, Link], joints: list[Joint]) -> str:
    """Require one root, unique parentage and a connected tree; return its root."""
    child_names: set[str] = set()
    children: dict[str, list[str]] = {}
    for link_name in links:
        children[link_name] = []

    # Every non-root link has exactly one incoming joint. Keeping both a set
    # of children and an adjacency list makes the two tree checks straightforward.
    for joint in joints:
        if joint.parent == joint.child or joint.child in child_names:
            raise RobotDescriptionError("each child link must have exactly one distinct parent")
        child_names.add(joint.child)
        children[joint.parent].append(joint.child)

    # The root is defined by connectivity, not by the order of the <link> tags.
    roots = []
    for link_name in links:
        if link_name not in child_names:
            roots.append(link_name)
    if len(roots) != 1:
        raise RobotDescriptionError("robot must have exactly one root link")
    root_link = roots[0]
    queue = deque([root_link])
    visited: set[str] = set()

    # Breadth-first traversal is iterative: a long serial arm cannot exhaust
    # Python's recursion depth. Revisiting a link signals a cycle; unvisited
    # links after traversal signal a disconnected component, possibly a cycle.
    while queue:
        link_name = queue.popleft()
        if link_name in visited:
            raise RobotDescriptionError("robot contains a cycle")
        visited.add(link_name)
        for child_name in children[link_name]:
            queue.append(child_name)
    if len(visited) != len(links):
        raise RobotDescriptionError("all links must form one connected tree")
    return root_link


def parse_robot_description(value: object) -> Robot:
    """Build a complete valid robot or raise RobotDescriptionError.

    Each phase operates only on this candidate. The publisher replaces its
    current Robot only after this function returns successfully; an error in a
    late joint or in tree validation therefore cannot leave a partial reload.
    """
    robot_element, namespace = _parse_robot_xml(value)
    links = _parse_links(robot_element, namespace)
    joints = _parse_joints(robot_element, namespace, links)
    root_link = _validate_tree(links, joints)
    robot_name = robot_element.get("name", "robot")
    return Robot(robot_name, root_link, links, joints)
