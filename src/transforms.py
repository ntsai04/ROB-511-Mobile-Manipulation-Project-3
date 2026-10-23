"""Rigid-transform math, written from the quaternion formulas.

HOW TO READ A TRANSFORM
----------------------
A transform has a translation (x, y, z) and a quaternion (x, y, z, w).
The quaternion's first three components are its vector part: the unit rotation
axis multiplied by sin(angle/2). Its scalar component w is cos(angle/2).
A rotation quaternion has unit length.

We use column vectors: a point in a child frame becomes a point in its parent
frame through ``parent_point = rotation * child_point + translation``.
Consequently ``T_parent * T_child`` first applies the child transform, then
the parent transform. The multiplication order matters for 3-D rotations.

The math stays in these small functions so the publishers can concentrate on
walking the robot tree. No kinematics or transform library is used.
"""

import math


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
ZERO_TRANSLATION: Vector3 = (0.0, 0.0, 0.0)
IDENTITY_QUATERNION: Quaternion = (0.0, 0.0, 0.0, 1.0)


def quaternion_normalize(quaternion: Quaternion) -> Quaternion:
    """Return a unit quaternion, rejecting invalid rotations."""
    for component in quaternion:
        if not math.isfinite(component):
            raise ValueError("quaternion components must be finite")

    # Dividing all four components by the same length preserves the rotation.
    # hypot computes that length without squaring huge components directly.
    length = math.hypot(*quaternion)
    if length == 0.0 or not math.isfinite(length):
        raise ValueError("quaternion must have nonzero, finite length")
    x, y, z, w = quaternion
    return x / length, y / length, z / length, w / length


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    """Hamilton product ``left * right``: apply right, then apply left.

    For a parent orientation and a child's local orientation, the result is
    the child's orientation expressed in the parent's parent frame.
    """
    left_x, left_y, left_z, left_w = left
    right_x, right_y, right_z, right_w = right

    # The vector part is w_left*v_right + w_right*v_left + v_left cross v_right.
    # The scalar part is w_left*w_right - v_left dot v_right.
    product_x = left_w * right_x + left_x * right_w + left_y * right_z - left_z * right_y
    product_y = left_w * right_y - left_x * right_z + left_y * right_w + left_z * right_x
    product_z = left_w * right_z + left_x * right_y - left_y * right_x + left_z * right_w
    product_w = left_w * right_w - left_x * right_x - left_y * right_y - left_z * right_z

    # Repeated tree composition can accumulate roundoff; keep every result unit.
    return quaternion_normalize((product_x, product_y, product_z, product_w))


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """URDF fixed-axis roll, pitch, yaw: ``Rz(yaw) * Ry(pitch) * Rx(roll)``.

    Roll acts about the fixed parent X axis first, pitch about fixed Y second,
    and yaw about fixed Z last. These are radians, and the axes do not move
    between the three rotations. The matching quaternion is q_yaw*q_pitch*q_roll.
    """
    # A rotation about a coordinate axis has only one nonzero vector component:
    # that axis's sin(angle/2). Its scalar component is always cos(angle/2).
    roll_quaternion = (math.sin(roll / 2.0), 0.0, 0.0, math.cos(roll / 2.0))
    pitch_quaternion = (0.0, math.sin(pitch / 2.0), 0.0, math.cos(pitch / 2.0))
    yaw_quaternion = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    # The rightmost rotation acts first. Compose pitch*roll, then yaw*(pitch*roll),
    # using the same Hamilton product used throughout the robot's transform tree.
    pitch_roll_quaternion = quaternion_multiply(pitch_quaternion, roll_quaternion)
    return quaternion_multiply(yaw_quaternion, pitch_roll_quaternion)


def quaternion_from_axis_angle(axis: Vector3, angle: float) -> Quaternion:
    """Rotate about a normalized copy of ``axis`` by ``angle`` radians."""
    for component in axis:
        if not math.isfinite(component):
            raise ValueError("axis and angle must define a finite rotation")
    axis_scale = max(abs(axis[0]), abs(axis[1]), abs(axis[2]))
    if axis_scale == 0.0 or not math.isfinite(angle):
        raise ValueError("axis and angle must define a finite rotation")

    # Rotational axes describe a direction: (0, 0, 2) rotates just like (0, 0, 1).
    # Prismatic motion is different and uses the unnormalized axis in Joint.
    # First scale the largest component to one. The length is then between one
    # and sqrt(3), avoiding overflow for huge axes and rounding of tiny lengths.
    scaled_axis_x = axis[0] / axis_scale
    scaled_axis_y = axis[1] / axis_scale
    scaled_axis_z = axis[2] / axis_scale
    scaled_axis_length = math.hypot(scaled_axis_x, scaled_axis_y, scaled_axis_z)
    unit_axis_x = scaled_axis_x / scaled_axis_length
    unit_axis_y = scaled_axis_y / scaled_axis_length
    unit_axis_z = scaled_axis_z / scaled_axis_length
    sin_half_angle = math.sin(angle / 2.0)
    cos_half_angle = math.cos(angle / 2.0)
    x = unit_axis_x * sin_half_angle
    y = unit_axis_y * sin_half_angle
    z = unit_axis_z * sin_half_angle
    return quaternion_normalize((x, y, z, cos_half_angle))


def rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    """Apply a quaternion rotation without constructing a matrix."""
    quaternion_x, quaternion_y, quaternion_z, quaternion_w = quaternion_normalize(quaternion)
    vector_x, vector_y, vector_z = vector
    for component in vector:
        if not math.isfinite(component):
            raise ValueError("vector components must be finite")

    # Rotation preserves magnitude. Work with bounded coordinates so the doubled
    # cross products cannot overflow before their terms cancel. A zero vector
    # needs no rotation and must be handled before dividing by its scale.
    vector_scale = max(abs(vector_x), abs(vector_y), abs(vector_z))
    if vector_scale == 0.0:
        return ZERO_TRANSLATION
    vector_x /= vector_scale
    vector_y /= vector_scale
    vector_z /= vector_scale

    # A vector is the pure quaternion (vx, vy, vz, 0). Expanding q*v*conjugate(q)
    # for a unit q gives v' = v + w*t + q_xyz cross t, where t = 2*(q_xyz cross v).
    # We calculate those cross products directly, keeping the scalar term out.
    twice_cross_x = 2.0 * (quaternion_y * vector_z - quaternion_z * vector_y)
    twice_cross_y = 2.0 * (quaternion_z * vector_x - quaternion_x * vector_z)
    twice_cross_z = 2.0 * (quaternion_x * vector_y - quaternion_y * vector_x)

    rotated_x = (vector_x + quaternion_w * twice_cross_x
                 + quaternion_y * twice_cross_z - quaternion_z * twice_cross_y)
    rotated_y = (vector_y + quaternion_w * twice_cross_y
                 + quaternion_z * twice_cross_x - quaternion_x * twice_cross_z)
    rotated_z = (vector_z + quaternion_w * twice_cross_z
                 + quaternion_x * twice_cross_y - quaternion_y * twice_cross_x)
    # Restore the original magnitude only after the rotation is complete.
    return rotated_x * vector_scale, rotated_y * vector_scale, rotated_z * vector_scale


def compose_transforms(
    parent_xyz: Vector3,
    parent_quaternion: Quaternion,
    child_xyz: Vector3,
    child_quaternion: Quaternion,
) -> tuple[Vector3, Quaternion]:
    """Compose ``T_parent * T_child`` and return translation and xyzw rotation."""
    # The child's translation is measured along the parent's local axes. Rotate
    # it into the outer frame before adding the parent's translation there.
    rotated_child_xyz = rotate_vector(parent_quaternion, child_xyz)
    translation_x = parent_xyz[0] + rotated_child_xyz[0]
    translation_y = parent_xyz[1] + rotated_child_xyz[1]
    translation_z = parent_xyz[2] + rotated_child_xyz[2]
    translation = (translation_x, translation_y, translation_z)

    # Orientations compose in exactly the same parent*child order.
    rotation = quaternion_multiply(parent_quaternion, child_quaternion)
    return translation, rotation


def matrix_column_major(translation: Vector3, quaternion: Quaternion) -> list[float]:
    """Return the 16 values required by Project 3's MatrixTransform message.

    The homogeneous matrix has rotation in its upper-left 3x3 block, translation
    in its last column, and bottom row [0, 0, 0, 1]. The wire format stores one
    whole column at a time: result[4*column + row] = matrix[row][column].
    """
    x, y, z, w = quaternion_normalize(quaternion)
    squared_x = x * x
    squared_y = y * y
    squared_z = z * z
    product_xy = x * y
    product_xz = x * z
    product_yz = y * z
    product_wx = w * x
    product_wy = w * y
    product_wz = w * z

    # These are the rows of the quaternion's rotation matrix. In particular,
    # the off-diagonal signs distinguish a rotation from its inverse.
    rotation_00 = 1.0 - 2.0 * (squared_y + squared_z)
    rotation_01 = 2.0 * (product_xy - product_wz)
    rotation_02 = 2.0 * (product_xz + product_wy)
    rotation_10 = 2.0 * (product_xy + product_wz)
    rotation_11 = 1.0 - 2.0 * (squared_x + squared_z)
    rotation_12 = 2.0 * (product_yz - product_wx)
    rotation_20 = 2.0 * (product_xz - product_wy)
    rotation_21 = 2.0 * (product_yz + product_wx)
    rotation_22 = 1.0 - 2.0 * (squared_x + squared_y)

    # Each line below is a column, not a row. Thus translation occupies 12-14,
    # while indices 3, 7, 11, 15 carry the homogeneous bottom row 0, 0, 0, 1.
    return [
        rotation_00, rotation_10, rotation_20, 0.0,
        rotation_01, rotation_11, rotation_21, 0.0,
        rotation_02, rotation_12, rotation_22, 0.0,
        translation[0], translation[1], translation[2], 1.0,
    ]
