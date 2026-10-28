import math
from checks import *
from transforms import (quaternion_normalize, quaternion_multiply, quaternion_from_rpy,
                        quaternion_from_axis_angle, rotate_vector, compose_transforms, matrix_column_major)
from robot_description import parse_robot_description
from robot_state_publisher import joint_configuration


@silent
def test_01_quaternion_normalization():
    close(quaternion_normalize((2, -4, 6, -8)), [x / math.sqrt(120) for x in (2, -4, 6, -8)])
    close(quaternion_normalize((1e200, 0, 0, 1e200)), (math.sqrt(.5), 0, 0, math.sqrt(.5)))
    for invalid in ((0, 0, 0, 0), (float("nan"), 0, 0, 1), (0, 0, float("inf"), 1)):
        try:
            quaternion_normalize(invalid)
        except ValueError:
            continue
        assert False


@silent
def test_02_hamilton_product_order():
    h = math.sqrt(.5)
    result = quaternion_multiply((h, 0, 0, h), (0, 0, h, h))
    check_transform(((0, 0, 0), result), multiply(pose(rpy=(math.pi/2, 0, 0)), pose(rpy=(0, 0, math.pi/2))))


@silent
def test_03_rpy_fixed_axis_order():
    for angles in ((.4, -.7, 1.2), (-1.1, .3, -.6), (math.pi/2, math.pi/2, 0)):
        check_transform(((0, 0, 0), quaternion_from_rpy(*angles)), pose(rpy=angles))


@silent
def test_04_axis_angle_normalizes_direction():
    for axis, angle in (((2, -3, 4), .8), ((0, 0, -7), math.pi/2), ((1e200, 0, 0), -.6)):
        check_transform(((0, 0, 0), quaternion_from_axis_angle(axis, angle)), axis_rotation(axis, angle))


@silent
def test_05_rotate_vector_handedness():
    h = math.sqrt(.5)
    close(rotate_vector((0, 0, h, h), (2, -3, 4)), (3, 2, 4))
    close(rotate_vector((2*h, 0, 0, 2*h), (0, 2, 3)), (0, -3, 2))
    close(rotate_vector((0, 0, 0, 3), (0, 0, 0)), (0, 0, 0))


@silent
def test_06_compose_rotates_child_translation():
    p, c = (.2, -.4, .8), (-.3, .5, 1.2)
    a, b = (.4, -.6, .8), (-.2, .7, -.9)
    check_transform(compose_transforms(p, quaternion_from_rpy(*a), c, quaternion_from_rpy(*b)), multiply(pose(p, a), pose(c, b)))


@silent
def test_07_matrix_column_major_layout():
    xyz, angles = (1.2, -.7, 3.4), (.5, -.8, .3)
    close(matrix_column_major(xyz, quaternion_from_rpy(*angles)), flatten(pose(xyz, angles)))


@silent
def test_08_urdf_defaults_namespace_direct_children():
    xml = description(joint(axis="<axis/>"), extra='<transmission><joint name="ignored" type="bad"/><link name="ignored"/></transmission>', namespace="urn:robot")
    robot = parse_robot_description(xml)
    assert robot.root_link == "base" and set(robot.links) == {"base", "arm"} and len(robot.joints) == 1
    j = robot.joints[0]
    assert j.axis == (1, 0, 0)
    check_transform(j.joint_transform(math.pi/2), pose(rpy=(math.pi/2, 0, 0)))


@silent
def test_09_origin_xyz_is_parent_frame():
    origin = '<origin xyz="1 2 3" rpy=".4 -.7 1.2"/>'
    j = parse_robot_description(description(joint(kind="fixed", origin=origin))).joints[0]
    check_transform(j.joint_transform(), pose((1, 2, 3), (.4, -.7, 1.2)))


@silent
def test_10_fixed_joint_ignores_position():
    xml = description(joint(kind="fixed", origin='<origin xyz=".3 -.2 .9" rpy=".2 .3 -.4"/>', axis='<axis xyz="0 0 0"/>'))
    j = parse_robot_description(xml).joints[0]
    check_transform(j.joint_transform(12.5), pose((.3, -.2, .9), (.2, .3, -.4)))


@silent
def test_11_revolute_origin_then_motion():
    xml = description(joint(kind="revolute", origin='<origin xyz="1 -.5 .2" rpy=".3 -.4 .8"/>', axis='<axis xyz="1 2 -3"/>', limits='<limit lower="-.1" upper=".1"/>'))
    j = parse_robot_description(xml).joints[0]
    check_transform(j.joint_transform(.9), multiply(pose((1, -.5, .2), (.3, -.4, .8)), axis_rotation((1, 2, -3), .9)))


@silent
def test_12_continuous_negative_unwrapped_angle():
    xml = description(joint(axis='<axis xyz="0 -2 0"/>', limits='<limit lower="bad" upper="bad"/>'))
    j = parse_robot_description(xml).joints[0]
    check_transform(j.joint_transform(-7.2), axis_rotation((0, -2, 0), -7.2))


@silent
def test_13_prismatic_preserves_axis_length():
    xml = description(joint(kind="prismatic", axis='<axis xyz="2 -3 4"/>', limits='<limit lower="-.1" upper=".1"/>'))
    j = parse_robot_description(xml).joints[0]
    check_transform(j.joint_transform(.7), pose((1.4, -2.1, 2.8)))
    check_transform(j.joint_transform(-.4), pose((-.8, 1.2, -1.6)))


@silent
def test_14_prismatic_motion_rotated_by_origin():
    xml = description(joint(kind="prismatic", origin='<origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/>', axis='<axis xyz="2 0 0"/>', limits='<limit lower="-1" upper="1"/>'))
    j = parse_robot_description(xml).joints[0]
    check_transform(j.joint_transform(.7), pose((1, 3.4, 3), (0, 0, math.pi/2)))


@silent
def test_15_urdf_rejects_invalid_topology():
    variants = [description(""), description(joint(parent="missing")),
                description(joint(child="base")),
                description(joint()+joint(name="k")),
                description(joint()+joint(name="k", parent="arm", child="base")),
                description(joint(), links=("base", "arm", "orphan")),
                description(joint(), links=("base", "arm", "arm")),
                description(joint()+joint(name="j", parent="arm", child="tip"), links=("base", "arm", "tip"))]
    for xml in variants:
        expect_invalid(xml)


@silent
def test_16_visual_origin_never_changes_fk():
    xml = '<robot name="v"><link name="base"/><link name="arm"><visual><origin xyz="8 9 10" rpy=".5 .6 .7"/><geometry><box size="1 2 3"/></geometry></visual><visual><geometry><bad/></geometry></visual></link>'+joint(kind="fixed", origin='<origin xyz="1 2 3"/>')+'</robot>'
    robot = parse_robot_description(xml)
    assert robot.links["arm"].visual.shape == "box"
    close(robot.links["arm"].visual.attributes["size"], (1, 2, 3))
    check_transform(robot.joints[0].joint_transform(), pose((1, 2, 3)))


@silent
def test_17_jointstate_replacement_names_and_stamp():
    xml = description(joint(name="a", axis='<axis xyz="0 0 1"/>')+joint(name="b", parent="arm", child="tip", axis='<axis xyz="0 1 0"/>')+joint(name="fixed", kind="fixed", parent="tip", child="end"), links=("base", "arm", "tip", "end"))
    _, local, _, out = publishers(xml)
    local.accept_joint_states(state(["b", "a", "unknown", "a", "fixed"], [.4, .1, 12, .7, 9]))
    local.update()
    first = entries(out["/tf"])
    check_edge(first["arm"], "base", "arm", pose(rpy=(0, 0, .7)))
    check_edge(first["tip"], "arm", "tip", pose(rpy=(0, .4, 0)))
    check_edge(first["end"], "tip", "end", identity())
    later = {"sec": 9, "nanosec": 123}
    local.accept_joint_states(state(["b"], [-.2], later)); local.update()
    second = entries(out["/tf"])
    check_edge(second["arm"], "base", "arm", identity(), later)
    check_edge(second["tip"], "arm", "tip", pose(rpy=(0, -.2, 0)), later)


@silent
def test_18_tf_quaternion_xyzw_fields():
    xml = description(joint(origin='<origin xyz=".2 -.3 .8" rpy=".4 -.7 1.1"/>', axis='<axis xyz="2 -1 3"/>'))
    robot = parse_robot_description(xml)
    message = joint_configuration(robot, {"j": -.6}, STAMP)
    assert len(message["transforms"]) == 1
    check_edge(message["transforms"][0], "base", "arm", multiply(pose((.2, -.3, .8), (.4, -.7, 1.1)), axis_rotation((2, -1, 3), -.6)))


@silent
def test_19_world_chain_rotates_offsets():
    xml = description(joint(name="a", axis='<axis xyz="0 0 1"/>')+joint(name="b", kind="fixed", parent="arm", child="tip", origin='<origin xyz="2 0 0" rpy=".2 .3 .4"/>'), links=("base", "arm", "tip"))
    _, local, world, out = publishers(xml)
    local.accept_joint_states(state(["a"], [math.pi/2])); local.update(); world.update()
    result = entries(out["/xform_world"])
    assert set(result) == {"base", "arm", "tip"}
    close(result["base"]["matrix"], flatten(identity()))
    close(result["tip"]["matrix"], flatten(multiply(pose(rpy=(0, 0, math.pi/2)), pose((2, 0, 0), (.2, .3, .4)))))


@silent
def test_20_world_branches_unordered_root():
    from robot_world_state_publisher import RobotWorldStatePublisher
    out = {}
    world = RobotWorldStatePublisher(lambda topic, msg: out.__setitem__(topic, msg))
    h = math.sqrt(.5)
    world.accept_tf({"transforms": [edge("left", "tip", (1, 0, 0)), edge("root", "right", (0, -2, 0)), edge("root", "left", (0, 1, 0), (0, 0, h, h))]})
    world.update(); result = entries(out["/xform_world"])
    assert set(result) == {"root", "right", "left", "tip"}
    close(result["tip"]["matrix"], flatten(pose((0, 2, 0), (0, 0, math.pi/2))))
    close(result["right"]["matrix"], flatten(pose((0, -2, 0))))
    for entry in result.values():
        assert entry["header"] == {"stamp": STAMP, "frame_id": "global_frame"}


@silent
def test_21_global_pose_before_and_after_tf():
    from robot_world_state_publisher import RobotWorldStatePublisher
    out = {}
    world = RobotWorldStatePublisher(lambda topic, msg: out.__setitem__(topic, msg))
    world.accept_global_pose({"position": dict(zip("xyz", (1, 2, 3))), "orientation": dict(zip("xyzw", (0, 0, 2, 2)))})
    world.accept_tf({"transforms": [edge("base", "tip", (2, 0, 0))]}); world.update()
    result = entries(out["/xform_world"])
    close(result["base"]["matrix"], flatten(pose((1, 2, 3), (0, 0, math.pi/2))))
    close(result["tip"]["matrix"], flatten(pose((1, 4, 3), (0, 0, math.pi/2))))
    world.accept_global_pose({"position": dict(zip("xyz", (-1, .5, 2))), "orientation": dict(zip("xyzw", (0, 0, 0, 1)))})
    world.update(); result = entries(out["/xform_world"])
    close(result["tip"]["matrix"], flatten(pose((1, .5, 2))))
    assert result["tip"]["header"]["stamp"] == STAMP


@silent
def test_22_description_rejection_is_atomic():
    xml = description(joint(axis='<axis xyz="0 0 1"/>'))
    parameters, local, world, out = publishers(xml)
    local.accept_joint_states(state(["j"], [.8])); local.update(); world.update()
    previous = out["/tf"]
    accepted_version = parameters.get_param({"name": "robot_description"})[1]["version"]
    version = parameters.set_param({"name": "robot_description", "value": description(joint(), links=("base", "arm", "orphan"))})[1]["version"]
    local.update(); world.update()
    assert out["/tf"] == previous
    status = parameters.get_param({"name": "/robot_state_publisher/description_status"})[1]["value"]
    assert status["accepted"] is False and status["version"] == version and status["loaded_version"] == accepted_version
    parameters.set_param({"name": "robot_description", "value": description(joint(name="k", parent="new", child="end", kind="fixed", origin='<origin xyz="3 4 5"/>'), links=("end", "new"))})
    local.update(); world.update()
    assert set(entries(out["/xform_world"])) == {"new", "end"}
    close(entries(out["/xform_world"])["end"]["matrix"], flatten(pose((3, 4, 5))))


@silent
def test_23_parameter_versions_missing_null():
    from param_server import ParameterServer
    p = ParameterServer()
    assert p.get_param({"name": "unset"})[:2] == (False, {"value": None, "version": 0})
    for version, value in enumerate((None, {"items": [1, False, "a"]}, {"items": [1, False, "a"]}), 1):
        assert p.set_param({"name": "a", "value": value})[:2] == (True, {"version": version})
        assert p.get_param({"name": "a"})[:2] == (True, {"value": value, "version": version})
    assert p.set_param({"name": "b"})[:2] == (True, {"version": 1})
    assert p.get_param({"name": "b"})[:2] == (True, {"value": None, "version": 1})
    assert p.get_param({"name": "a"})[1]["version"] == 3
    assert p.set_param({"name": 1})[0] is False and p.get_param([])[0] is False


@silent
def test_24_urdf_numeric_axis_limits_validation():
    for origin in ('<origin xyz="1 2"/>', '<origin xyz="nan 0 0"/>', '<origin rpy="0 0 1_000"/>'):
        expect_invalid(description(joint(origin=origin)))
    expect_invalid(description(joint(axis='<axis xyz="0 0 0"/>')))
    expect_invalid(description(joint(kind="prismatic")))
    expect_invalid(description(joint(kind="revolute", limits='<limit lower="1e999"/>')))
    expect_invalid(description(joint(kind="floating")))
    j = parse_robot_description(description(joint(kind="revolute", limits='<limit/>'))).joints[0]
    assert j.lower == 0 and j.upper == 0
    check_transform(j.joint_transform(.3), pose(rpy=(.3, 0, 0)))


@silent
def test_25_invalid_messages_preserve_last_snapshot():
    xml = description(joint(axis='<axis xyz="0 0 1"/>'))
    _, local, world, out = publishers(xml)
    local.accept_joint_states(state(["j"], [.6])); local.update(); world.update()
    old_tf, old_world = out["/tf"], out["/xform_world"]
    for invalid in (state(["j"], []), state(["j"], [True]), state(["j"], [float("nan")]), state(["j"], [1], {"sec": 1.5, "nanosec": 0})):
        local.accept_joint_states(invalid); local.update(); assert out["/tf"] == old_tf
    for invalid in ({"transforms": [edge("a", "b"), edge("c", "b")]}, {"transforms": [edge("a", "b", q=(0, 0, 0, 0))]}):
        world.accept_tf(invalid); world.update(); assert out["/xform_world"] == old_world
    world.accept_global_pose({"position": {"x": 4, "y": 5, "z": 6}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 0}})
    world.update(); assert out["/xform_world"] == old_world
