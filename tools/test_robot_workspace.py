"""Analytical FK and workspace semantics, independent of live hardware."""

import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from robot_workspace import boundary_lines, forward, read_chain, sample_workspace, voxel_points


def model(joints):
    links = {'base'}
    for joint in ET.fromstring('<robot>' + joints + '</robot>'):
        links.update(joint.find(tag).get('link') for tag in ('parent', 'child'))
    return ET.fromstring('<robot>' + ''.join(f'<link name="{link}"/>' for link in links) + joints + '</robot>')


def test_rotating_offset_and_origin_order_against_planar_geometry():
    robot = model('''
      <joint name="mount" type="fixed"><parent link="base"/><child link="mount"/>
        <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/></joint>
      <joint name="a" type="revolute"><parent link="mount"/><child link="a"/>
        <axis xyz="0 0 2"/><limit lower="-3.14" upper="3.14"/></joint>
      <joint name="b" type="revolute"><parent link="a"/><child link="b"/>
        <origin xyz="2 0 0"/><axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14"/></joint>
      <joint name="tcp" type="fixed"><parent link="b"/><child link="tcp"/>
        <origin xyz="1 0 0"/></joint>''')
    angles = np.array([[0., 0.], [-math.pi / 2, math.pi / 2], [math.pi / 2, -math.pi / 2]])
    xyz, _ = forward(read_chain(robot, 'base', 'tcp'), angles)
    np.testing.assert_allclose(xyz, [[1, 5, 3], [3, 3, 3], [-1, 3, 3]], atol=1e-12)


def test_fixed_transform_uses_urdf_roll_pitch_yaw_order():
    # Rz(90) Ry(90) Rx(90) maps local +X to global -Z.
    robot = model('''
      <joint name="mount" type="fixed"><parent link="base"/><child link="mount"/>
        <origin rpy="1.5707963267948966 1.5707963267948966 1.5707963267948966"/></joint>
      <joint name="tcp" type="fixed"><parent link="mount"/><child link="tcp"/>
        <origin xyz="1 0 0"/></joint>''')
    xyz, _ = forward(read_chain(robot, 'base', 'tcp'), np.empty((1, 0)))
    np.testing.assert_allclose(xyz, [[0, 0, -1]], atol=1e-12)


def six_joint_robot(tcp_type='fixed'):
    joints = ''
    parent = 'base'
    for i in range(6):
        joints += f'''<joint name="j{i}" type="revolute"><parent link="{parent}"/>
          <child link="j{i}"/><origin xyz="0.1 0 0"/><axis xyz="0 0 1"/>
          <limit lower="-3" upper="3"/><safety_controller soft_lower_limit="-2" soft_upper_limit="2"/></joint>'''
        parent = f'j{i}'
    joints += f'''<joint name="tcp" type="{tcp_type}"><parent link="j5"/><child link="tcp"/>
      <origin xyz="0.1 0.2 0.3"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint>'''
    return model(joints)


def test_tcp_distance_is_preserved_but_direction_rotates_and_soft_limits_apply():
    arm, tcp, offset, joints = sample_workspace(six_joint_robot(), 'base', 'j5', 'tcp', 1000, 5)
    np.testing.assert_allclose(np.linalg.norm(tcp - arm, axis=1), np.linalg.norm(offset), atol=1e-12)
    assert np.ptp((tcp - arm)[:, 0]) > .1  # Not a constant world-frame translation.
    assert all(j.bounds == (-2., 2.) for j in joints)
    repeated = sample_workspace(six_joint_robot(), 'base', 'j5', 'tcp', 1000, 5)
    np.testing.assert_array_equal(arm, repeated[0])


def test_dynamic_tcp_and_disconnected_frames_are_rejected():
    with pytest.raises(ValueError, match='固定关节'):
        sample_workspace(six_joint_robot('revolute'), 'base', 'j5', 'tcp', 100, 5)
    with pytest.raises(ValueError, match='缺少坐标系'):
        read_chain(six_joint_robot(), 'unknown', 'tcp')
    with pytest.raises(ValueError, match='无法建立'):
        read_chain(six_joint_robot(), 'tcp', 'base')


def test_downsampling_retains_only_reachable_samples():
    points = np.array([[.01, .01, .01], [.02, .02, .02], [-.01, 0, 0], [.21, 0, 0]])
    result = voxel_points(points, .1)
    np.testing.assert_array_equal(result, points[[0, 2, 3]])


def test_outer_boundary_excludes_interior_samples_and_has_unique_surface_edges():
    from itertools import product

    corners = np.array(list(product((-1., 1.), repeat=3)))
    points = np.vstack((corners, [[0, 0, 0], [.1, .2, .3]]))
    segments = boundary_lines(points).reshape(-1, 2, 3)
    endpoints = {tuple(point) for point in segments.reshape(-1, 3)}
    assert endpoints == {tuple(point) for point in corners}
    # Every line lies on at least one cube face, never through its interior.
    for start, end in segments:
        assert np.any(start == end)
    undirected = {tuple(sorted((tuple(start), tuple(end)))) for start, end in segments}
    assert len(undirected) == len(segments)
    translated = boundary_lines(points + [10, -3, 2]).reshape(-1, 3)
    np.testing.assert_allclose(translated.min(axis=0), [9, -4, 1])
    np.testing.assert_allclose(translated.max(axis=0), [11, -2, 3])


def test_boundary_rejects_degenerate_samples_instead_of_inventing_volume():
    with pytest.raises(ValueError, match='至少四个'):
        boundary_lines(np.zeros((3, 3)))
    with pytest.raises(ValueError, match='三维外包络'):
        boundary_lines(np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]]))
