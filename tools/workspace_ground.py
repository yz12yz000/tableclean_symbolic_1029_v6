"""Model-ground estimate and geometric plane sections, without ROS commands."""

from dataclasses import dataclass
from pathlib import Path
import struct
from urllib.parse import unquote, urlparse

import numpy as np


def origin_transform(element):
    from scipy.spatial.transform import Rotation

    result = np.eye(4)
    if element is not None:
        xyz = np.asarray([float(v) for v in element.get('xyz', '0 0 0').split()])
        rpy = np.asarray([float(v) for v in element.get('rpy', '0 0 0').split()])
        if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite([xyz, rpy]).all():
            raise ValueError('地面模型的 origin 无效')
        result[:3, 3] = xyz
        result[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    return result


def fixed_transform(model, reference, link):
    """Pose of link in reference; only fixed paths can define model ground."""
    links = {node.get('name') for node in model.findall('link')}
    if reference not in links or link not in links:
        raise ValueError(f'地面坐标系不在 URDF 中：{reference} / {link}')
    graph = {name: [] for name in links}
    for joint in model.findall('joint'):
        if joint.get('type') != 'fixed':
            continue
        parent, child = (joint.find(tag).get('link') for tag in ('parent', 'child'))
        pose = origin_transform(joint.find('origin'))
        graph[parent].append((child, pose))
        graph[child].append((parent, np.linalg.inv(pose)))
    pending, seen = [(reference, np.eye(4))], set()
    while pending:
        name, pose = pending.pop()
        if name == link:
            return pose
        if name in seen:
            continue
        seen.add(name)
        pending.extend((child, pose @ step) for child, step in graph[name] if child not in seen)
    raise ValueError(f'地面与显示基座之间必须是固定连接：{reference} -> {link}')


def read_stl(path):
    """Read binary or ASCII STL vertices; do not assume a 'solid' header is ASCII."""
    raw = Path(path).read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0] if len(raw) >= 84 else 0
    if count and len(raw) == 84 + 50 * count:
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
        vertices = np.frombuffer(raw, dtype=dtype, count=count, offset=84)['vertices'].reshape(-1, 3)
    else:
        vertices = np.asarray([[float(v) for v in line.split()[1:]]
                               for line in raw.decode('ascii').splitlines()
                               if line.strip().startswith('vertex ')])
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices) or not np.isfinite(vertices).all():
        raise ValueError(f'无效 STL：{path}')
    return vertices


def mesh_path(uri, urdf_directory=None):
    if uri.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory

        package, relative = uri[len('package://'):].split('/', 1)
        return Path(get_package_share_directory(package)) / unquote(relative)
    if uri.startswith('file://'):
        return Path(unquote(urlparse(uri).path))
    path = Path(uri)
    if not path.is_absolute():
        if urdf_directory is None:
            raise ValueError('相对 mesh 路径需要 --urdf 文件，或使用 --ground-z 指定接地高度')
        path = urdf_directory / path
    return path


def model_bottom(model, link, urdf_directory=None):
    """Minimum visual STL Z in the merged chassis frame, not measured wheel contacts."""
    node = next((node for node in model.findall('link') if node.get('name') == link), None)
    if node is None:
        raise ValueError(f'URDF 缺少底盘 link：{link}')
    minima = []
    for visual in node.findall('visual'):
        mesh = visual.find('geometry/mesh')
        if mesh is None:
            raise ValueError('自动地面估计需要底盘 STL；可用 --ground-z 指定地面高度')
        path = mesh_path(mesh.get('filename'), urdf_directory)
        if path.suffix.lower() != '.stl':
            raise ValueError('自动地面估计仅支持 STL；可用 --ground-z 指定地面高度')
        vertices = read_stl(path)
        scale = np.asarray([float(v) for v in mesh.get('scale', '1 1 1').split()])
        if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale == 0):
            raise ValueError('底盘 STL scale 无效')
        pose = origin_transform(visual.find('origin'))
        heights = (vertices * scale) @ pose[2, :3] + pose[2, 3]
        minima.append(float(heights.min()))
    if not minima:
        raise ValueError('地面参考 link 没有可用 STL；请使用 --ground-z 指定高度')
    return min(minima)


@dataclass
class GroundPlane:
    origin: np.ndarray  # All vectors expressed in the workspace's base frame.
    axes: np.ndarray    # Two orthonormal in-plane axes, as columns.
    normal: np.ndarray
    height: float       # Height in ground_frame, not base frame.
    estimated: bool


def ground_plane(model, base, ground_frame, height=None, urdf_directory=None):
    pose = fixed_transform(model, base, ground_frame)
    estimated = height is None
    if estimated:
        height = model_bottom(model, ground_frame, urdf_directory)
    if not np.isfinite(height):
        raise ValueError('地面高度必须是有限数值')
    return GroundPlane(pose[:3, 3] + pose[:3, 2] * height,
                       pose[:3, :2], pose[:3, 2], float(height), estimated)


@dataclass
class GroundSection:
    points: np.ndarray  # Convex, counterclockwise polygon on the exact plane.
    uv: np.ndarray
    area: float
    center: np.ndarray  # Area centroid, in workspace base frame.


def plane_section(lines, plane):
    """Intersect the displayed hull's edges with a plane, then order the polygon.

    Area is for this convex envelope, including concavities bridged by the hull.
    Empty, point and line contact have zero area; a coplanar face retains its area.
    """
    from scipy.spatial import ConvexHull, QhullError

    edges = np.asarray(lines, dtype=float).reshape(-1, 2, 3)
    distances = (edges - plane.origin) @ plane.normal
    on_plane = edges[np.abs(distances) <= 1e-9]
    crossing = ((distances[:, 0] < -1e-9) & (distances[:, 1] > 1e-9)
                | (distances[:, 0] > 1e-9) & (distances[:, 1] < -1e-9))
    d = distances[crossing]
    t = -d[:, 0] / (d[:, 1] - d[:, 0])
    cut = edges[crossing, 0] + t[:, None] * (edges[crossing, 1] - edges[crossing, 0])
    uv = (np.concatenate((on_plane, cut)) - plane.origin) @ plane.axes
    # Round only to deduplicate; keep the original exact-plane coordinates.
    _, indices = np.unique(np.round(uv, 9), axis=0, return_index=True)
    uv = uv[indices]
    area, center = 0., plane.origin.copy()
    if len(uv) >= 3:
        try:
            uv = uv[ConvexHull(uv).vertices]
            next_uv = np.roll(uv, -1, axis=0)
            cross = uv[:, 0] * next_uv[:, 1] - next_uv[:, 0] * uv[:, 1]
            area = float(cross.sum() / 2)
            centroid = ((uv + next_uv) * cross[:, None]).sum(axis=0) / (6 * area)
            center = plane.origin + plane.axes @ centroid
        except QhullError:
            pass  # Tangent contact with no enclosed area.
    points = plane.origin + uv @ plane.axes.T
    if len(points) and area == 0:
        center = points.mean(axis=0)
    return GroundSection(points, uv, area, center)


def ground_markers(args, plane, section):
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    layers = {name: MarkerArray() for name in ('ground_plane', 'ground_section')}

    def marker(layer, index, kind, color, alpha=1.):
        result = Marker()
        result.header.frame_id = args.base_frame
        result.ns, result.id = f'ur5e_workspace/{layer}', index
        result.type = kind
        result.pose.orientation.w = 1.
        result.scale.x = result.scale.y = result.scale.z = 1.
        result.color.r, result.color.g, result.color.b = color
        result.color.a = alpha
        result.frame_locked = True
        result.lifetime.sec = 2
        layers[layer].markers.append(result)
        return result

    def points(values):
        return [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in values]

    half = args.ground_size / 2
    corners = plane.origin + np.array([[-half, -half], [half, -half], [half, half], [-half, half]]) @ plane.axes.T
    floor = marker('ground_plane', 0, Marker.TRIANGLE_LIST, (.45, .52, .59), .18)
    floor.points = points(corners[[0, 1, 2, 0, 2, 3]])
    grid = marker('ground_plane', 1, Marker.LINE_LIST, (.5, .6, .66), .55)
    grid.scale.x = .0015
    segments = []
    for coordinate in np.linspace(-half, half, max(2, int(args.ground_size / .1) + 1)):
        segments.extend([[coordinate, -half], [coordinate, half], [-half, coordinate], [half, coordinate]])
    grid.points = points(plane.origin + np.asarray(segments) @ plane.axes.T + plane.normal * .001)

    # Lift rendered section 3 mm to prevent depth fighting; calculations stay exact.
    raised = section.points + plane.normal * .003
    if section.area > 0:
        fill = marker('ground_section', 0, Marker.TRIANGLE_LIST, (1., .78, .08), .38)
        triangles = []
        for index in range(1, len(raised) - 1):
            triangles.extend([raised[0], raised[index], raised[index + 1]])
        fill.points = points(triangles)
    if len(raised) >= 2:
        outline = marker('ground_section', 1, Marker.LINE_STRIP, (1., .9, .12))
        outline.scale.x = .007
        outline.points = points(np.vstack((raised, raised[0])))
    label = marker('ground_section', 2, Marker.TEXT_VIEW_FACING, (1., .9, .3))
    label.scale.z = .045
    anchor = section.center + plane.normal * .10
    if len(section.uv):
        anchor = plane.origin + plane.axes @ np.array([section.uv[:, 0].mean(), section.uv[:, 1].min() - .13]) + plane.normal * .10
    label.pose.position = points([anchor])[0]
    label.text = f'Ground-section:{section.area:.3f}m^2\nApprox-hull/No-collision-filter'
    return layers
