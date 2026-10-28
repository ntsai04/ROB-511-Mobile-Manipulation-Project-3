"""Silent assertions and independent matrix oracles for the exam tests."""
import contextlib
import functools
import io
import math
import os
from pathlib import Path
import sys

sys.excepthook = lambda *_args: None
sys.path.insert(0, str(Path(os.environ.get("AUTOROB_SOURCE_ROOT", Path(__file__).resolve().parents[1])) / "src"))


def silent(function):
    @functools.wraps(function)
    def run():
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            try:
                function()
            except BaseException:
                raise SystemExit(1) from None
    return run


def close(actual, expected, tolerance=1e-9):
    assert len(actual) == len(expected)
    assert all(math.isclose(a, b, rel_tol=tolerance, abs_tol=tolerance) for a, b in zip(actual, expected))


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def pose(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, xyz[0]],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, xyz[1]],
            [-sp, cp*sr, cp*cr, xyz[2]], [0, 0, 0, 1]]


def axis_rotation(axis, angle):
    length = math.hypot(*axis)
    u = [x / length for x in axis]
    c, s, d = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    x, y, z = u
    return [[c+x*x*d, x*y*d-z*s, x*z*d+y*s, 0],
            [y*x*d+z*s, c+y*y*d, y*z*d-x*s, 0],
            [z*x*d-y*s, z*y*d+x*s, c+z*z*d, 0], [0, 0, 0, 1]]


def flatten(matrix):
    return [matrix[r][c] for c in range(4) for r in range(4)]


def quat_matrix(q, xyz=(0, 0, 0)):
    x, y, z, w = q
    scale = math.hypot(*q)
    assert math.isclose(scale, 1, rel_tol=1e-9, abs_tol=1e-9)
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), xyz[0]],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), xyz[1]],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), xyz[2]], [0, 0, 0, 1]]


def check_transform(actual, expected):
    xyz, q = actual
    close(flatten(quat_matrix(q, xyz)), flatten(expected))


def description(joints, links=("base", "arm"), extra="", namespace=""):
    names = "".join('<link name="%s"/>' % name for name in links)
    return '<robot name="probe"%s>%s%s%s</robot>' % (' xmlns="'+namespace+'"' if namespace else '', names, joints, extra)


def joint(name="j", kind="continuous", parent="base", child="arm", origin="", axis="", limits=""):
    return ('<joint name="%s" type="%s"><parent link="%s"/><child link="%s"/>%s%s%s</joint>'
            % (name, kind, parent, child, origin, axis, limits))


STAMP = {"sec": 41, "nanosec": 7654321}


def state(names, positions, stamp=None):
    return {"header": {"stamp": dict(STAMP if stamp is None else stamp)}, "name": names, "position": positions}


def edge(parent, child, xyz=(0, 0, 0), q=(0, 0, 0, 1)):
    return {"header": {"stamp": dict(STAMP), "frame_id": parent}, "child_frame_id": child,
            "transform": {"translation": dict(zip("xyz", xyz)), "rotation": dict(zip("xyzw", q))}}


def entries(message):
    return {x["child_frame_id"]: x for x in message["transforms"]}


def check_edge(entry, parent, child, expected, stamp=STAMP):
    assert entry["header"] == {"stamp": stamp, "frame_id": parent}
    assert entry["child_frame_id"] == child
    t = entry["transform"]
    check_transform((tuple(t["translation"][k] for k in "xyz"), tuple(t["rotation"][k] for k in "xyzw")), expected)


def publishers(xml):
    from param_server import ParameterServer
    from robot_state_publisher import RobotStatePublisher
    from robot_world_state_publisher import RobotWorldStatePublisher
    parameters = ParameterServer()
    messages = {}
    world = RobotWorldStatePublisher(lambda topic, msg: messages.__setitem__(topic, msg))
    def publish(topic, msg):
        messages[topic] = msg
        if topic == "/tf":
            world.accept_tf(msg)
    def call(name, args):
        return parameters.service_handlers()[name](args)
    local = RobotStatePublisher(call, publish)
    parameters.set_param({"name": "robot_description", "value": xml})
    local.update()
    world.update()
    return parameters, local, world, messages


def expect_invalid(xml):
    from robot_description import RobotDescriptionError, parse_robot_description
    try:
        parse_robot_description(xml)
    except RobotDescriptionError:
        return
    assert False
