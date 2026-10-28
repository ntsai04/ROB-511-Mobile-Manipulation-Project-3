import math
import time

from checks import (Rosbridge, document, get, joint, load, observe, pose, put,
                    reject_all, scenario, service, silent)


@silent
def test_01_parameter_json_roundtrip_and_version():
    with Rosbridge() as ros:
        name = 'mutation_json_' + str(time.time_ns())
        values = [None, False, 0, -2.75, 'quote" newline\n / 雪', [1, None, True],
                  {'nested': {'a': [1, 2, {'z': False}]}}]
        for index, value in enumerate(values, 1):
            assert put(ros, name, value) == index
            response = get(ros, name)
            assert response['result'] is True
            assert response['values'] == {'value': value, 'version': index}
        assert put(ros, name, values[-1]) == len(values) + 1


@silent
def test_02_parameter_missing_null_validation_and_isolation():
    with Rosbridge() as ros:
        name = 'mutation_isolation_' + str(time.time_ns())
        missing = get(ros, name)
        assert missing['result'] is False and missing['status']
        assert missing['values'] == {'value': None, 'version': 0}
        a = service(ros, '/param_server/set_param', {'name': name})
        assert a['version'] == 1
        assert get(ros, name)['values'] == {'value': None, 'version': 1}
        assert put(ros, name+'b', 7) == 1
        assert put(ros, name, 6) == 2
        assert get(ros, name+'b')['values'] == {'value': 7, 'version': 1}
        for args in ({}, {'name': 5}, {'name': None}, {'name': ['x']}):
            reply = ros.call_service('/param_server/set_param', args, timeout=3)
            assert reply['result'] is False and reply['status']


@silent
def test_03_large_urdf_roundtrip_and_pipeline():
    joints = [joint(xyz=(.4, -.3, .2))]
    xml = document(joints).replace('</robot>', '<!--' + 'padding_'*110000 + '--></robot>')
    assert len(xml.encode()) < 1000000
    with Rosbridge() as ros:
        version = load(ros, xml)
        assert get(ros, 'robot_description')['values'] == {'value': xml, 'version': version}
        observe(ros, joints)


@silent
def test_04_invalid_xml_types_and_root_preserve_robot():
    reject_all([None, {'robot': []}, '', '   ', '<robot>', '<not_robot><link name="b"/></not_robot>',
                '<robot/>', '<robot><gazebo><link name="nested"/></gazebo></robot>'])


@silent
def test_05_invalid_joint_structure_and_limits_preserve_robot():
    base = document([joint(kind='revolute')])
    reject_all([base.replace('name="tip"', ''), base.replace('name="joint"', ''),
                base.replace('type="revolute"', ''), base.replace('type="revolute"', 'type="Revolute"'),
                base.replace('<parent link="base"/>', ''), base.replace('child link="tip"', 'child link="missing"'),
                base.replace('<limit lower="-10" upper="10"/>', ''),
                base.replace('lower="-10"', 'lower="two"'), base.replace('</robot>', '<link name="orphan"/></robot>')])


@silent
def test_06_invalid_vectors_and_visual_geometry_preserve_robot():
    base = document([joint()])
    candidates = [base.replace('xyz="0 0 0"', 'xyz='+repr(v).replace("'", '"'), 1)
                  for v in ['1 2', '1 2 3 4', '', '0,0,1', '1_000 0 0']]
    for geometry in ['<box size="1 2"/>', '<sphere/>', '<cylinder radius="1"/>',
                     '<mesh/>', '<capsule radius="1"/>']:
        candidates.append(base.replace('<link name="tip"/>',
                          '<link name="tip"><visual><geometry>'+geometry+'</geometry></visual></link>'))
    reject_all(candidates)


@silent
def test_07_xml_features_namespace_entities_and_direct_children():
    joints = [joint(name='hinge&x', child='tip&A', kind='continuous', xyz=(.25, -.2, 3))]
    xml = '''  <?xml version="1.0" encoding="UTF-8"?>
<?practice ignored?>
<robot xmlns="urn:robot" xmlns:xacro="urn:xacro">
<!-- <joint name="fake"> -->
<joint name='hinge&amp;x' type='continuous'>
<parent link='base'/><child link='tip&amp;&#x41;'/>
<origin xyz=' .25&#9; -2E-1&#10; 3. '></origin><axis/>
</joint>
<link name='tip&amp;A'/><link name='base'/>
<transmission><joint name='not_a_joint'/></transmission>
<gazebo><![CDATA[<link name="not_a_link"/>]]><link name='ignored'/></gazebo>
</robot>'''.replace('\n', '\r\n')
    scenario(joints, {'hinge&x': .7}, xml)


@silent
def test_08_all_visual_shapes_do_not_change_fk():
    joints = [joint(name='j'+str(i), parent='base', child='l'+str(i), xyz=(i*.2, .1, .4)) for i in range(4)]
    xml = document(joints)
    shapes = ['<box size="1 2 3"/>', '<sphere radius=".3"/>',
              '<cylinder radius=".4" length="2"/>', '<mesh filename="package://ignored" scale="2 3 4"/>']
    for i, shape in enumerate(shapes):
        xml = xml.replace('<link name="l%d"/>' % i, '<link name="l%d"><visual><origin xyz="4 5 6" rpy=".4 .5 .6"/><geometry>%s</geometry></visual><collision><bad/></collision><inertial/></link>' % (i, shape))
    scenario(joints, xml=xml)


@silent
def test_09_joint_defaults_and_fixed_ignored_axis_limit():
    joints = [joint(kind='revolute'), joint(name='mount', parent='tip', child='end', xyz=(0, .5, 0))]
    xml = '''<robot><link name="base"/><link name="tip"/><link name="end"/>
<joint name="joint" type="revolute"><parent link="base"/><child link="tip"/><origin/><axis/><limit lower="-2" upper="2"/></joint>
<joint name="mount" type="fixed"><parent link="tip"/><child link="end"/><origin xyz="0 .5 0"/><axis xyz="0 0 0"/><limit lower="ignored"/></joint></robot>'''
    scenario(joints, {'joint': .8}, xml)


@silent
def test_10_fixed_origin_translation_and_all_rpy_axes():
    scenario([joint(xyz=(.3, -.4, .7), rpy=(.42, -.61, 1.23))])


@silent
def test_11_revolute_coordinate_axes_and_sign():
    joints = [joint(name='j'+str(i), parent='base', child='l'+str(i), kind='revolute', axis=axis)
              for i, axis in enumerate([(1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 0, -1)])]
    scenario(joints, {'j0': .4, 'j1': -.7, 'j2': 1.2, 'j3': -.9})


@silent
def test_12_oblique_revolute_axes_are_normalized():
    joints = [joint(name='a', child='a_link', kind='revolute', axis=(2, -3, 4)),
              joint(name='b', child='b_link', kind='revolute', axis=(.02, -.03, .04))]
    scenario(joints, {'a': 1.37, 'b': 1.37})


@silent
def test_13_continuous_negative_and_multiturn_angles():
    joints = [joint(kind='continuous', axis=(1, 2, -.5), rpy=(.1, -.2, .3))]
    with Rosbridge() as ros:
        load(ros, document(joints))
        for q in [-5.8, 0, 2*math.pi, 3.9]:
            observe(ros, joints, {'joint': q})


@silent
def test_14_prismatic_axis_length_sign_and_mounted_direction():
    joints = [joint(kind='prismatic', axis=(2, -1, .5), xyz=(.1, .2, .3), rpy=(.3, .5, -.7))]
    with Rosbridge() as ros:
        load(ros, document(joints))
        for q in [.7, -.45, 0]:
            observe(ros, joints, {'joint': q})


@silent
def test_15_revolute_origin_precedes_motion_and_mimic_is_ignored():
    joints = [joint(name='a', child='upper', kind='revolute', axis=(.3, 1, .4), xyz=(.7, -.1, .4), rpy=(.4, -.3, .6)),
              joint(name='b', parent='upper', child='end', kind='continuous', axis=(1, 0, 1),
                    xyz=(.8, .1, 0), extra='<mimic joint="a" multiplier="2" offset="1"/>')]
    scenario(joints, {'a': .9, 'b': -.4})


@silent
def test_16_reverse_document_serial_noncommuting_chain():
    joints = [joint(name='a', child='a_link', kind='continuous', xyz=(.4, .2, 0), axis=(0, 0, 1)),
              joint(name='b', parent='a_link', child='b_link', kind='revolute', xyz=(1, 0, 0), axis=(0, 1, 0), rpy=(.2, .3, 0)),
              joint(name='c', parent='b_link', child='end', kind='prismatic', xyz=(.3, .2, .1), axis=(1, 2, 0), rpy=(.4, 0, .6))]
    scenario(joints, {'a': .7, 'b': -.8, 'c': .3}, document(joints, reverse=True))


@silent
def test_17_branched_tree_world_composition():
    joints = [joint(name='trunk', child='fork', kind='continuous', axis=(0, 1, 0), xyz=(.4, .2, .8)),
              joint(name='left', parent='fork', child='left_link', xyz=(1, 0, 0), rpy=(.2, .4, .6)),
              joint(name='right', parent='fork', child='right_link', kind='prismatic', axis=(0, 2, 0), xyz=(-.3, -.5, .2)),
              joint(name='end', parent='left_link', child='left_end', xyz=(.2, .1, .4))]
    scenario(joints, {'trunk': -.9, 'right': .6}, document(joints, reverse=True))


@silent
def test_18_global_nontrivial_pose_and_column_major_matrix():
    joints = [joint(xyz=(1.2, -.7, .4), rpy=(.2, .4, -.5))]
    scenario(joints, global_pose=pose((2.3, -1.4, .8), (1, -2, 3), 1.1))


@silent
def test_19_global_pose_replaces_previous_and_quaternion_sign():
    joints = [joint(xyz=(.4, .7, -.1), rpy=(.2, .3, .5))]
    with Rosbridge() as ros:
        load(ros, document(joints))
        p = pose((1, 2, 3), (1, 1, -1), .8)
        observe(ros, joints, global_pose=p)
        p['orientation'] = {k: -v for k, v in p['orientation'].items()}
        p['seq'] = 123
        observe(ros, joints, global_pose=p)
        observe(ros, joints, global_pose=pose((-2, .2, 1), (0, 1, 1), -.9))


@silent
def test_20_joint_message_replaces_all_positions():
    joints = [joint(name='a', child='a_link', kind='continuous'),
              joint(name='b', parent='a_link', child='end', kind='prismatic', xyz=(.5, 0, 0))]
    with Rosbridge() as ros:
        load(ros, document(joints))
        observe(ros, joints, {'a': 1.1, 'b': .7})
        observe(ros, joints, {'b': -.3})
        observe(ros, joints, {})


@silent
def test_21_unknown_fixed_names_and_name_order_are_ignored():
    joints = [joint(name='a', child='a_link', kind='continuous'),
              joint(name='b', parent='a_link', child='b_link', kind='prismatic'),
              joint(name='fixed', parent='b_link', child='end', xyz=(.5, .4, .3))]
    scenario(joints, {'unknown': 5, 'fixed': 7, 'b': .4, 'a': -.9},
             ignored_fields={'velocity': [100, -100, 50, 60], 'effort': [20, 30, 40, 50]})


@silent
def test_22_joint_stamp_propagates_exactly_to_every_frame():
    joints = [joint(name='a', child='a_link', kind='continuous'),
              joint(name='b', parent='a_link', child='end', xyz=(1, 0, 0))]
    with Rosbridge() as ros:
        load(ros, document(joints))
        first, _, _ = observe(ros, joints, {'a': .3})
        second, _, _ = observe(ros, joints, {'a': .3})
        assert first != second


@silent
def test_23_reload_smaller_robot_removes_old_frames():
    old = [joint(name='a', child='a_link'), joint(name='b', parent='a_link', child='old_tip')]
    new = [joint(parent='new_root', child='new_tip', xyz=(.3, .2, .1))]
    with Rosbridge() as ros:
        load(ros, document(old))
        observe(ros, old)
        load(ros, document(new), root='new_root')
        observe(ros, new)


@silent
def test_24_rejected_reload_preserves_config_and_global_pose():
    joints = [joint(kind='continuous', xyz=(.5, .2, .3), rpy=(.3, .1, -.5))]
    with Rosbridge() as ros:
        version = load(ros, document(joints))
        p = pose((1, -2, .5), (1, 2, 3), -.6)
        _, before_tf, before_world = observe(ros, joints, {'joint': 1.2}, p)
        invalid = document(joints).replace('</robot>', '<link name="unattached"/></robot>')
        load(ros, invalid, False, version)
        # Read a new tick WITHOUT republishing the state or pose after rejection.
        ros.pending.clear()
        for _ in range(2):
            tf = ros.next_message('/tf', timeout=3)
            world = ros.next_message('/xform_world', timeout=3)
        assert tf['transforms'][0]['child_frame_id'] == 'tip'
        assert {t['child_frame_id'] for t in world['transforms']} - {'global_frame'} == {'base', 'tip'}
        assert tf == before_tf and world == before_world
        # Re-observe also checks exact preserved geometry against independent matrices.
        observe(ros, joints, {'joint': 1.2}, p)


@silent
def test_25_long_tree_complete_frames_and_numeric_notation():
    joints = [joint(name='j'+str(i), parent='base' if i == 0 else 'l'+str(i-1), child='l'+str(i),
                    kind='continuous' if i % 4 == 0 else 'fixed', xyz=(.01, -.02, .03),
                    rpy=(.02, -.01, .03), axis=(1, 2, 3)) for i in range(100)]
    xml = document(joints, reverse=True).replace('0.01 -0.02 0.03', '+1E-2  -2e-2\t .03')
    scenario(joints, {'j'+str(i): .2 for i in range(0, 100, 4)}, xml)
