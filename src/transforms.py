"""Small rigid-transform primitives, using URDF conventions and xyzw quaternions.

Translations are expressed in the parent frame.  Composition applies the child
transform first, then its parent, just like multiplication of column-vector
homogeneous matrices.  Everything here uses the Python standard library.
"""

import math


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
ZERO_TRANSLATION: Vector3 = (0.0, 0.0, 0.0)
IDENTITY_QUATERNION: Quaternion = (0.0, 0.0, 0.0, 1.0)


def quaternion_normalize(quaternion: Quaternion) -> Quaternion:
    """Return a unit quaternion, rejecting invalid rotations."""
    if not all(math.isfinite(component) for component in quaternion):
        raise ValueError("quaternion components must be finite")
    norm = math.hypot(*quaternion)
    if norm == 0.0 or not math.isfinite(norm):
        raise ValueError("quaternion must have nonzero, finite length")
    return tuple(component / norm for component in quaternion)


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    """Hamilton product: the rotation ``left`` followed by local ``right``."""
    x, y, z, w = left
    a, b, c, d = right
    return quaternion_normalize((
        w * a + x * d + y * c - z * b,
        w * b - x * c + y * d + z * a,
        w * c + x * b - y * a + z * d,
        w * d - x * a - y * b - z * c,
    ))


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """URDF fixed-axis RPY: ``Rz(yaw) * Ry(pitch) * Rx(roll)``."""
    sr, cr = math.sin(roll / 2.0), math.cos(roll / 2.0)
    sp, cp = math.sin(pitch / 2.0), math.cos(pitch / 2.0)
    sy, cy = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return quaternion_normalize((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def quaternion_from_axis_angle(axis: Vector3, angle: float) -> Quaternion:
    """Rotate about a normalized copy of ``axis`` by ``angle`` radians."""
    norm = math.hypot(*axis)
    if norm == 0.0 or not math.isfinite(norm) or not math.isfinite(angle):
        raise ValueError("axis and angle must define a finite rotation")
    sine = math.sin(angle / 2.0)
    return quaternion_normalize((
        axis[0] / norm * sine,
        axis[1] / norm * sine,
        axis[2] / norm * sine,
        math.cos(angle / 2.0),
    ))


def rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    """Apply a quaternion rotation without constructing a matrix."""
    x, y, z, w = quaternion_normalize(quaternion)
    vx, vy, vz = vector
    # v' = v + 2*w*(q_xyz cross v) + 2*(q_xyz cross (q_xyz cross v)).
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def compose_transforms(
    parent_xyz: Vector3,
    parent_quaternion: Quaternion,
    child_xyz: Vector3,
    child_quaternion: Quaternion,
) -> tuple[Vector3, Quaternion]:
    """Compose ``T_parent * T_child`` and return translation and xyzw rotation."""
    rotated = rotate_vector(parent_quaternion, child_xyz)
    translation = tuple(parent_xyz[index] + rotated[index] for index in range(3))
    rotation = quaternion_multiply(parent_quaternion, child_quaternion)
    return translation, rotation


def matrix_column_major(translation: Vector3, quaternion: Quaternion) -> list[float]:
    """Return the 16 values required by Project 3's MatrixTransform message."""
    x, y, z, w = quaternion_normalize(quaternion)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy), 0.0,
        2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx), 0.0,
        2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy), 0.0,
        translation[0], translation[1], translation[2], 1.0,
    ]
