"""Analytical ground sections and mesh-to-base coordinate checks."""

from itertools import product
import math
import struct
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from robot_workspace import boundary_lines
from workspace_ground import GroundPlane, fixed_transform, ground_plane, model_bottom, plane_section, read_stl


def plane(origin=(0., 0., 0.), normal=(0., 0., 1.)):
    normal = np.asarray(normal, dtype=float)
    normal /= np.linalg.norm(normal)
    u = np.cross([0., 1., 0.], normal)
    u /= np.linalg.norm(u)
    return GroundPlane(np.asarray(origin), np.column_stack((u, np.cross(normal, u))), normal, 0., False)


def cube_edges():
    return boundary_lines(np.array(list(product((-1., 1.), repeat=3))))


def test_horizontal_cube_sections_and_coplanar_face_have_known_area():
    for z in (0., .3, 1., -1.):
        section = plane_section(cube_edges(), plane((0., 0., z)))
        assert section.area == pytest.approx(4.)
        np.testing.assert_allclose(section.center, [0, 0, z], atol=1e-10)
        np.testing.assert_allclose(section.points[:, 2], z)
        assert len(section.points) == 4


def test_oblique_cube_section_is_regular_hexagon():
    floor = plane(normal=(1., 1., 1.))
    section = plane_section(cube_edges(), floor)
    assert section.area == pytest.approx(3 * math.sqrt(3))
    assert len(section.points) == 6
    np.testing.assert_allclose(section.points @ floor.normal, 0., atol=1e-10)
    np.testing.assert_allclose(section.center, [0, 0, 0], atol=1e-10)


def test_tetrahedron_section_is_not_replaced_by_sphere_or_xy_projection():
    edges = boundary_lines(np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]]))
    section = plane_section(edges, plane((0., 0., .5)))
    assert section.area == pytest.approx(.125)
    np.testing.assert_allclose(section.center, [1/6, 1/6, .5], atol=1e-10)


def test_outside_and_tangent_contacts_have_zero_area():
    section = plane_section(cube_edges(), plane((0., 0., 2.)))
    assert section.area == 0 and len(section.points) == 0
    section = plane_section(cube_edges(), plane((1., 1., 0.), normal=(1., 1., 0.)))
    assert section.area == 0 and len(section.points) == 2
    section = plane_section(cube_edges(), plane((1., 1., 1.), normal=(1., 1., 1.)))
    assert section.area == 0 and len(section.points) == 1


@pytest.mark.parametrize('binary', [False, True])
def test_stl_scale_visual_origin_and_fixed_mount_are_applied(tmp_path, binary):
    vertices = np.array([[0., 0, -.03], [1, 0, .1], [0, 1, .1]])
    path = tmp_path / 'base.stl'
    if binary:
        # Even a binary STL may have a header that starts with 'solid'.
        path.write_bytes(b'solid binary'.ljust(80, b' ') + struct.pack('<I', 1)
                         + struct.pack('<12fH', 0, 0, 1, *vertices.ravel(), 0))
    else:
        path.write_text('solid test\n' + '\n'.join('vertex ' + ' '.join(map(str, v)) for v in vertices) + '\nendsolid\n')
    np.testing.assert_allclose(read_stl(path), vertices, atol=1e-8)
    model = ET.fromstring(f'''<robot>
      <link name="mobile"><visual><origin xyz="0 0 0.01"/>
      <geometry><mesh filename="{path}" scale="1 1 2"/></geometry></visual></link>
      <link name="base"/>
      <joint name="mount" type="fixed"><parent link="mobile"/><child link="base"/>
      <origin xyz="1 2 0.7" rpy="0 0 1.5707963267948966"/></joint></robot>''')
    assert model_bottom(model, 'mobile') == pytest.approx(-.05, abs=1e-8)
    floor = ground_plane(model, 'base', 'mobile')
    np.testing.assert_allclose(floor.origin, [-2, 1, -.75], atol=1e-8)
    np.testing.assert_allclose(floor.normal, [0, 0, 1])
    assert floor.estimated
    overridden = ground_plane(model, 'base', 'mobile', height=0.)
    assert not overridden.estimated
    assert overridden.origin[2] == pytest.approx(-.7)
    np.testing.assert_allclose(fixed_transform(model, 'mobile', 'base') @ fixed_transform(model, 'base', 'mobile'), np.eye(4), atol=1e-10)


def test_ground_frame_cannot_follow_an_arm_joint():
    model = ET.fromstring('''<robot><link name="base"/><link name="tip"/>
      <joint name="joint" type="revolute"><parent link="base"/><child link="tip"/></joint></robot>''')
    with pytest.raises(ValueError, match='固定连接'):
        ground_plane(model, 'base', 'tip', height=0.)
