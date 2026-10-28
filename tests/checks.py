"""Independent wire-level fixtures and matrix oracle; imports no project code."""
import contextlib
import functools
import io
import math
import time
from xml.sax.saxutils import quoteattr

from rosbridge_client import Rosbridge


def silent(function):
    @functools.wraps(function)
    def wrapped():
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            try:
                function()
            except BaseException:
                raise SystemExit(1) from None
    return wrapped


def service(ros, name, args):
    response = ros.call_service(name, args, timeout=3)
    assert response.get('result') is True
    return response['values']


def get(ros, name):
    return ros.call_service('/param_server/get_param', {'name': name}, timeout=3)


def put(ros, name, value):
    return service(ros, '/param_server/set_param', {'name': name, 'value': value})['version']


def load(ros, xml, accepted=True, old_version=None, root='base'):
    version = put(ros, 'robot_description', xml)
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        response = get(ros, '/robot_state_publisher/description_status')
        status = response.get('values', {}).get('value')
        if isinstance(status, dict) and status.get('version') == version:
            assert status['accepted'] is accepted
            assert status['root_link'] == root
            assert status['loaded_version'] == (version if accepted else old_version)
            assert (status['error'] == '') if accepted else bool(status['error'])
            return version
        time.sleep(.025)
    raise AssertionError('description pickup exceeded 2 seconds')


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def translation(xyz):
    m = identity()
    for i in range(3):
        m[i][3] = xyz[i]
    return m


def rotation(axis, angle):
    length = math.hypot(*axis)
    x, y, z = [v / length for v in axis]
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    m = identity()
    m[0][:3] = [t*x*x+c, t*x*y-s*z, t*x*z+s*y]
    m[1][:3] = [t*x*y+s*z, t*y*y+c, t*y*z-s*x]
    m[2][:3] = [t*x*z-s*y, t*y*z+s*x, t*z*z+c]
    return m


def origin(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    m = translation(xyz)
    for axis, angle in [((0, 0, 1), rpy[2]), ((0, 1, 0), rpy[1]), ((1, 0, 0), rpy[0])]:
        m = mul(m, rotation(axis, angle))
    return m


def joint(name='joint', parent='base', child='tip', kind='fixed', xyz=(0, 0, 0),
          rpy=(0, 0, 0), axis=(1, 0, 0), extra=''):
    return dict(name=name, parent=parent, child=child, kind=kind, xyz=xyz, rpy=rpy, axis=axis, extra=extra)


def document(joints, reverse=False):
    names = list(dict.fromkeys([j['parent'] for j in joints] + [j['child'] for j in joints]))
    links = ''.join('<link name=' + quoteattr(n) + '/>' for n in reversed(names))
    edges = []
    for j in (reversed(joints) if reverse else joints):
        text = '<joint name=%s type=%s><parent link=%s/><child link=%s/>' % tuple(
            quoteattr(j[k]) for k in ('name', 'kind', 'parent', 'child'))
        text += '<origin xyz=%s rpy=%s/><axis xyz=%s/>' % tuple(
            quoteattr(' '.join(str(x) for x in j[k])) for k in ('xyz', 'rpy', 'axis'))
        if j['kind'] in ('revolute', 'prismatic'):
            text += '<limit lower="-10" upper="10"/>'
        edges.append(text + j['extra'] + '</joint>')
    return '<robot name="mutation_fixture">' + ''.join(edges) + links + '</robot>'


def matrices(joints, positions, global_matrix):
    local = {}
    for j in joints:
        q = positions.get(j['name'], 0)
        motion = identity()
        if j['kind'] in ('revolute', 'continuous'):
            motion = rotation(j['axis'], q)
        elif j['kind'] == 'prismatic':
            motion = translation([q*a for a in j['axis']])
        local[j['child']] = mul(origin(j['xyz'], j['rpy']), motion)
    roots = set(j['parent'] for j in joints) - set(local)
    assert len(roots) == 1
    root = roots.pop()
    world = {root: global_matrix}
    pending = list(joints)
    while pending:
        ready = [j for j in pending if j['parent'] in world]
        assert ready
        for j in ready:
            world[j['child']] = mul(world[j['parent']], local[j['child']])
            pending.remove(j)
    return local, world


def quaternion_matrix(q):
    values = [q[k] for k in ('x', 'y', 'z', 'w')]
    assert all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values)
    assert abs(math.hypot(*values) - 1) < 1e-4
    x, y, z, w = values
    vlength = math.hypot(x, y, z)
    return identity() if vlength < 1e-14 else rotation((x, y, z), 2*math.atan2(vlength, w))


def compare(actual, expected):
    assert all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v)
               for row in actual for v in row)
    assert max(abs(actual[r][c] - expected[r][c]) for r in range(4) for c in range(4)) < 2e-5


def pose(xyz=(0, 0, 0), axis=(0, 0, 1), angle=0):
    scale = math.sin(angle/2) / math.hypot(*axis)
    return {'position': dict(zip('xyz', xyz)),
            'orientation': dict(zip('xyzw', [a*scale for a in axis] + [math.cos(angle/2)]))}


def observe(ros, joints, positions=None, global_pose=None, ignored_fields=None):
    positions = positions or {}
    global_pose = global_pose or pose()
    # Advertise before publish, and subscribe before generating a unique stamped state.
    ros.subscribe('/tf', 'tf/tfMessage')
    ros.subscribe('/xform_world', 'xform_world/MatrixTransformArray')
    ros.advertise('/joint_states', 'sensor_msgs/JointState')
    ros.advertise('/global_pose', 'geometry_msgs/Pose')
    ros.publish('/global_pose', global_pose)
    stamp_number = time.time_ns()
    stamp = {'sec': stamp_number // 10**9, 'nanosec': stamp_number % 10**9}
    state = {'header': {'stamp': stamp, 'frame_id': ''}, 'name': list(positions),
             'position': list(positions.values())}
    if ignored_fields:
        state.update(ignored_fields)
    ros.publish('/joint_states', state)
    tf = ros.next_message('/tf', timeout=3, match=lambda m: bool(m.get('transforms')) and
                          all(t.get('header', {}).get('stamp') == stamp for t in m['transforms']))
    world = ros.next_message('/xform_world', timeout=3, match=lambda m: bool(m.get('transforms')) and
                             all(t.get('header', {}).get('stamp') == stamp for t in m['transforms']))
    local_matrix = quaternion_matrix(global_pose['orientation'])
    for i, axis in enumerate('xyz'):
        local_matrix[i][3] = global_pose['position'][axis]
    expected_local, expected_world = matrices(joints, positions, local_matrix)
    assert len(tf['transforms']) == len(joints)
    entries = {t['child_frame_id']: t for t in tf['transforms']}
    assert set(entries) == set(expected_local)
    for j in joints:
        entry = entries[j['child']]
        assert entry['header']['frame_id'] == j['parent']
        m = quaternion_matrix(entry['transform']['rotation'])
        for i, axis in enumerate('xyz'):
            m[i][3] = entry['transform']['translation'][axis]
        compare(m, expected_local[j['child']])
    world_entries = {t['child_frame_id']: t for t in world['transforms']}
    assert len(world_entries) == len(world['transforms'])
    if 'global_frame' in world_entries:
        entry = world_entries.pop('global_frame')
        compare([[entry['matrix'][4*c+r] for c in range(4)] for r in range(4)], identity())
    assert set(world_entries) == set(expected_world)
    for name, expected in expected_world.items():
        entry = world_entries[name]
        assert entry['header']['frame_id'] == 'global_frame'
        assert len(entry['matrix']) == 16
        compare([[entry['matrix'][4*c+r] for c in range(4)] for r in range(4)], expected)
    return stamp, tf, world


def scenario(joints, positions=None, xml=None, global_pose=None, ignored_fields=None):
    with Rosbridge(timeout=3) as ros:
        roots = set(j['parent'] for j in joints) - set(j['child'] for j in joints)
        load(ros, xml if xml is not None else document(joints), root=roots.pop())
        return observe(ros, joints, positions, global_pose, ignored_fields)


def reject_all(candidates):
    joints = [joint(xyz=(.3, -.2, .7), rpy=(.2, .3, -.4))]
    with Rosbridge(timeout=3) as ros:
        version = load(ros, document(joints))
        for candidate in candidates:
            load(ros, candidate, False, version)
            observe(ros, joints)
