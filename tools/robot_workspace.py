"""Optional, read-only URDF workspace sampling and finite-lifetime RViz markers.

Points represent the union over sampled joint angles, with free tool orientation.
No collision, IK, trajectory, controller or hardware command is sent.
"""

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np


TOPIC = '/ur5e_workspace/markers'
NAMESPACE = 'ur5e_workspace'
COLORS = ((0.12, 0.55, 1.0), (1.0, 0.42, 0.06))
LAYERS = ('arm_boundary', 'gripper_boundary', 'arm_points', 'gripper_points',
          'ground_plane', 'ground_section')


def layer_topic(layer):
    return f'/ur5e_workspace/{layer}'


def selected_layer(layer, args):
    if layer == 'ground_plane':
        return args.ground
    if layer == 'ground_section':
        return args.ground_section
    robot, style = layer.split('_')
    return (args.only in ('both', robot) and args.view in ('both', style))


def triple(value):
    result = np.asarray([float(v) for v in value.split()])
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError('URDF 中存在无效的三维向量')
    return result


def rotations(axis, angles):
    """Rodrigues rotation matrices for a normalized axis and N joint angles."""
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    angles = np.asarray(angles).reshape(-1, 1, 1)
    c, s = np.cos(angles), np.sin(angles)
    return c * np.eye(3) + (1 - c) * np.outer(axis, axis) + s * skew


@dataclass
class Joint:
    name: str
    xyz: np.ndarray
    rotation: np.ndarray
    axis: np.ndarray
    bounds: object  # None for fixed joints; (lower, upper) otherwise.


def read_chain(model, base, tip):
    links = {link.get('name') for link in model.findall('link')}
    if base not in links or tip not in links:
        raise ValueError(f'URDF 缺少坐标系：{base} 或 {tip}')
    parents = {}
    for joint in model.findall('joint'):
        child = joint.find('child').get('link')
        if child in parents:
            raise ValueError(f'URDF 重复父关节：{child}')
        parents[child] = joint
    child, chain, seen = tip, [], set()
    while child != base:
        if child in seen or child not in parents:
            raise ValueError(f'无法建立 {base} -> {tip} 链')
        seen.add(child)
        joint = parents[child]
        name, kind = joint.get('name'), joint.get('type')
        if kind not in ('fixed', 'revolute', 'continuous') or joint.find('mimic') is not None:
            raise ValueError(f'不支持的关节：{name} ({kind})')
        origin = joint.find('origin')
        xyz = triple(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0')
        rpy = triple(origin.get('rpy', '0 0 0') if origin is not None else '0 0 0')
        rotation = np.eye(3)
        for axis, angle in zip(np.eye(3)[::-1], rpy[::-1]):
            rotation = rotation @ rotations(axis, [angle])[0]
        axis, bounds = np.array([1., 0., 0.]), None
        if kind != 'fixed':
            element = joint.find('axis')
            axis = triple(element.get('xyz', '1 0 0') if element is not None else '1 0 0')
            length = np.linalg.norm(axis)
            if length < 1e-12:
                raise ValueError(f'无效关节轴：{name}')
            axis /= length
            if kind == 'continuous':
                bounds = (-math.pi, math.pi)
            else:
                limit = joint.find('limit')
                if limit is None:
                    raise ValueError(f'缺少关节限位：{name}')
                lower, upper = float(limit.get('lower')), float(limit.get('upper'))
                safety = joint.find('safety_controller')
                if safety is not None:
                    lower = max(lower, float(safety.get('soft_lower_limit', lower)))
                    upper = min(upper, float(safety.get('soft_upper_limit', upper)))
                bounds = (lower, upper)
            if not np.isfinite(bounds).all() or bounds[0] >= bounds[1]:
                raise ValueError(f'无效关节限位：{name}')
        chain.append(Joint(name, xyz, rotation, axis, bounds))
        child = joint.find('parent').get('link')
    return list(reversed(chain))


def forward(chain, positions):
    """Batched FK. positions has one column per moving joint in chain order."""
    count = len(positions)
    expected = sum(j.bounds is not None for j in chain)
    if positions.shape != (count, expected) or not np.isfinite(positions).all():
        raise ValueError('关节角数组维度或数值错误')
    xyz = np.zeros((count, 3))
    rotation = np.broadcast_to(np.eye(3), (count, 3, 3)).copy()
    column = 0
    for joint in chain:
        xyz += np.einsum('nij,j->ni', rotation, joint.xyz)
        rotation = rotation @ joint.rotation
        if joint.bounds is not None:
            rotation = rotation @ rotations(joint.axis, positions[:, column])
            column += 1
    return xyz, rotation


def sample_workspace(model, base, tool, tcp, samples, seed):
    chain = read_chain(model, base, tool)
    moving = [j for j in chain if j.bounds is not None]
    if len(moving) != 6:
        raise ValueError(f'预期 UR5e 的六个转动关节，实际找到 {len(moving)} 个')
    extension = read_chain(model, tool, tcp)
    if any(j.bounds is not None for j in extension):
        raise ValueError('抓取点必须通过固定关节连接到法兰；不能忽略活动夹爪关节')
    offset = forward(extension, np.empty((1, 0)))[0][0]
    bounds = np.asarray([j.bounds for j in moving])
    angles = np.random.default_rng(seed).uniform(bounds[:, 0], bounds[:, 1], (samples, 6))
    arm, orientation = forward(chain, angles)
    gripper = arm + np.einsum('nij,j->ni', orientation, offset)
    return arm, gripper, offset, moving


def voxel_points(points, size):
    # Keep a real sampled position per voxel, not an interpolated/convex hull point.
    _, indices = np.unique(np.floor(points / size).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(indices)]


def boundary_lines(points):
    """Unique edges of the sampled convex outer envelope, NOT an exact workspace.

    A convex hull bridges concavities/holes and its edges are interpolated.
    Use the full samples so voxel reduction cannot discard extreme points.
    """
    from scipy.spatial import ConvexHull, QhullError

    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 4 or not np.isfinite(points).all():
        raise ValueError('边界需要至少四个有效的三维采样点')
    try:
        triangles = ConvexHull(points).simplices
    except QhullError as exc:
        raise ValueError('采样点无法构成三维外包络；请增加采样数或检查模型') from exc
    edges = np.concatenate([triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]])
    edges = np.unique(np.sort(edges, axis=1), axis=0)
    return points[edges].reshape(-1, 3)


def workspace_markers(samples, args, offset, boundaries, ground_layers):
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    layers = {layer: MarkerArray() for layer in LAYERS}
    info = MarkerArray()

    def marker(group, index, kind, color, result):
        m = Marker()
        m.header.frame_id = args.base_frame
        m.ns, m.id, m.type = f'{NAMESPACE}/{group}', index, kind
        m.pose.orientation.w = 1.0
        m.frame_locked = True
        m.lifetime.sec = 2  # Also vanishes after an unclean process exit.
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0
        result.markers.append(m)
        return m

    for index, (points, color, label) in enumerate(zip(samples, COLORS, (args.tool_frame, args.tcp_frame))):
        robot = ('arm', 'gripper')[index]
        m = marker(f'{robot}_points', 0, Marker.POINTS, color, layers[f'{robot}_points'])
        m.scale.x = m.scale.y = args.point_size
        m.color.a = args.alpha
        m.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in voxel_points(points, args.voxel)]
        m = marker(f'{robot}_boundary', 0, Marker.LINE_LIST, color, layers[f'{robot}_boundary'])
        m.scale.x = args.boundary_width
        m.color.a = .65
        m.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in boundaries[index]]
        if args.only not in ('both', robot):
            continue
        text = marker('info', 10 + index, Marker.TEXT_VIEW_FACING, color, info)
        text.pose.position.z = 1.45 - index * .09
        text.scale.z = .045
        text.text = f'{label}:{("ARM", "TCP")[index]}'
    text = marker('info', 12, Marker.TEXT_VIEW_FACING, (0.92, 0.92, 0.92), info)
    text.pose.position.z = 1.25
    text.scale.z = .028
    text.text = f'Approx-outer-hull/No-collision-filter\nTCP-offset:{np.linalg.norm(offset)*1000:.1f}mm'
    layers.update(ground_layers)
    selected = MarkerArray()
    for layer, message in layers.items():
        if selected_layer(layer, args):
            selected.markers.extend(message.markers)
    selected.markers.extend(info.markers)
    return layers, selected


def rviz_settings(args):
    """Independent checkboxes; CLI options only set their initial state."""
    import yaml

    config = yaml.safe_load(Path(__file__).with_name('robot_workspace.rviz').read_text())
    topics = {layer_topic(layer): layer for layer in LAYERS}
    for display in config['Visualization Manager']['Displays']:
        layer = topics.get(display.get('Topic', {}).get('Value'))
        if layer:
            display['Enabled'] = display['Value'] = selected_layer(layer, args)
        if display['Class'] == 'rviz_default_plugins/RobotModel':
            display['Description Topic']['Value'] = args.description_topic
    return config


def parser():
    p = argparse.ArgumentParser(description='临时显示 UR5e 法兰与夹爪抓取点的运动学工作空间；不发送运动指令。',
        epilog='临时 RViz 有六个独立开关；现有 RViz 可添加 /ur5e_workspace 下各 MarkerArray 话题。Ctrl+C 后 2 秒内消失。')
    p.add_argument('--rviz', action='store_true', help='另外打开已配好的临时 RViz 窗口；退出时一并关闭')
    p.add_argument('--check', action='store_true', help='只读取模型、计算范围并退出，不发布图形')
    p.add_argument('--only', choices=('both', 'arm', 'gripper'), default='both', help='初始显示两者/仅机械臂/仅夹爪，默认 both')
    p.add_argument('--view', choices=('boundary', 'points', 'both'), default='boundary', help='初始显示边界/原始点云/两者，默认 boundary')
    p.add_argument('--boundary-width', type=float, default=.003, help='外包络线宽（米），默认 0.003')
    p.add_argument('--ground', action=argparse.BooleanOptionalAction, default=True, help='初始显示地面平面，--no-ground 可隐藏')
    p.add_argument('--ground-section', action=argparse.BooleanOptionalAction, default=True, help='初始显示地面与夹爪外包络交集，--no-ground-section 可隐藏')
    p.add_argument('--ground-frame', default='mobile_base_link', help='地面参考坐标系，默认 mobile_base_link；与显示基座固定连接')
    p.add_argument('--ground-z', type=float, help='地面在 ground-frame 中的 Z（米），默认由底盘 STL 最低点估算')
    p.add_argument('--ground-size', type=float, default=3., help='地面显示方形边长（米），默认 3；不裁剪计算的交集')
    p.add_argument('--samples', type=int, default=100000, help='关节采样数量，默认 100000')
    p.add_argument('--voxel', type=float, default=.035, help='显示点去重网格大小（米），默认 0.035')
    p.add_argument('--point-size', type=float, default=.008, help='显示点大小（米），默认 0.008')
    p.add_argument('--alpha', type=float, default=.32, help='显示点透明度，默认 0.32')
    p.add_argument('--seed', type=int, default=5, help='采样随机种子，默认 5')
    p.add_argument('--base-frame', default='base_link')
    p.add_argument('--tool-frame', default='tool0')
    p.add_argument('--tcp-frame', default='grasp_center')
    p.add_argument('--description-topic', default='/robot_description')
    p.add_argument('--timeout', type=float, default=15., help='等待当前机器人模型的秒数')
    p.add_argument('--urdf', type=Path, help='可选：读取已展开的 URDF 文件，默认读取正在运行的机器人模型')
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if not 100 <= args.samples <= 2000000:
        p.error('--samples 必须在 100 到 2000000 之间')
    for key in ('voxel', 'point_size', 'boundary_width', 'ground_size', 'timeout'):
        if not math.isfinite(getattr(args, key)) or getattr(args, key) <= 0:
            p.error(f'--{key.replace("_", "-")} 必须为有限正数')
    if not math.isfinite(args.alpha) or not 0 < args.alpha <= 1 or args.seed < 0:
        p.error('--alpha 必须在 (0, 1] 内，--seed 必须非负')
    if args.ground_z is not None and not math.isfinite(args.ground_z):
        p.error('--ground-z 必须是有限数值')

    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray

    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    previous = {s: signal.signal(s, stop) for s in (signal.SIGINT, signal.SIGTERM)}
    node = rviz = rviz_directory = None
    outputs = []
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = rclpy.create_node('ur5e_workspace_display')
        if args.urdf:
            xml = args.urdf.read_text()
        else:
            received = []
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            subscription = node.create_subscription(String, args.description_topic, lambda m: received.append(m.data), qos)
            print(f'读取当前模型 {args.description_topic} ...', flush=True)
            deadline = time.monotonic() + args.timeout
            while not received and not stopped and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            node.destroy_subscription(subscription)
            if stopped:
                return 0
            if not received:
                raise RuntimeError('未收到 robot_description。先运行准备脚本，并保持相同 ROS_DOMAIN_ID / DDS 环境。')
            xml = received[0]
        model = ET.fromstring(xml)
        arm, gripper, offset, joints = sample_workspace(model, args.base_frame, args.tool_frame,
                                                       args.tcp_frame, args.samples, args.seed)
        print(f'固定基座：{args.base_frame}；采样 {args.samples} 组关节角；姿态自由；未检查碰撞。')
        print(f'{args.tool_frame} -> {args.tcp_frame} 平移：{offset.tolist()} m；距离 {np.linalg.norm(offset)*1000:.2f} mm')
        for joint in joints:
            lo, hi = np.degrees(joint.bounds)
            print(f'  {joint.name}: [{lo:.2f}, {hi:.2f}] deg（URDF 硬/软限位交集）')
        for label, points in ((args.tool_frame, arm), (args.tcp_frame, gripper)):
            print(f'{label} 采样 XYZ 包围盒（米，非精确边界）：{np.round(points.min(axis=0), 4).tolist()} -> {np.round(points.max(axis=0), 4).tolist()}')
        from workspace_ground import ground_plane, ground_markers, plane_section

        boundaries = [boundary_lines(points) for points in (arm, gripper)]
        plane = ground_plane(model, args.base_frame, args.ground_frame, args.ground_z,
                             args.urdf.resolve().parent if args.urdf else None)
        section = plane_section(boundaries[1], plane)
        source = '底盘 STL 最低点估算，非实测四轮接地标定' if plane.estimated else '手动指定'
        print(f'地面：{args.ground_frame} Z={plane.height:.6f} m（{source}）')
        print(f'地面在 {args.base_frame} 中：参考点 {np.round(plane.origin, 6).tolist()} m，法向 {np.round(plane.normal, 6).tolist()}')
        print(f'地面与夹爪外包络的交集面积 ≈ {section.area:.4f} m²；等面积圆半径 ≈ {math.sqrt(section.area/math.pi):.4f} m')
        if len(section.points):
            print(f'交集在 {args.base_frame} 的 XY 包围盒（米）：{np.round(section.points[:, :2].min(axis=0), 4).tolist()} -> {np.round(section.points[:, :2].max(axis=0), 4).tolist()}；形心 {np.round(section.center, 4).tolist()}')
        print('交集含底盘下方区域，未检查姿态、碰撞或路径；不能当作实际地面可抓取范围。', flush=True)
        if args.check or stopped:
            return 0
        layers, selected = workspace_markers((arm, gripper), args, offset, boundaries,
                                             ground_markers(args, plane, section))
        # Individual topics stay available even if initially unchecked in RViz.
        for layer, message in layers.items():
            outputs.append((node.create_publisher(MarkerArray, layer_topic(layer), 1), message))
        outputs.append((node.create_publisher(MarkerArray, TOPIC, 1), selected))
        if args.rviz:
            import yaml
            rviz_directory = tempfile.TemporaryDirectory(prefix='ur5e-workspace-')
            config = Path(rviz_directory.name) / 'workspace.rviz'
            config.write_text(yaml.safe_dump(rviz_settings(args), sort_keys=False))
            rviz = subprocess.Popen(['rviz2', '-d', str(config), '-f', args.base_frame,
                                     '--ros-args', '-r', '__node:=rviz2_workspace'], start_new_session=True)
        print(f'发布 {TOPIC}：蓝色={args.tool_frame}，橙色={args.tcp_frame}。', flush=True)
        print(f'初始显示：{args.only}/{args.view}；Boundary 为采样点的凸外包络，跨过凹陷和空洞，非精确可达边界。', flush=True)
        print('RViz 左侧六个开关：机械臂/夹爪边界和点云、Ground Plane、Ground - Gripper Section。Ctrl+C 或关闭临时窗口结束。', flush=True)
        next_publish = 0.
        while not stopped and rclpy.ok() and (rviz is None or rviz.poll() is None):
            if time.monotonic() >= next_publish:
                # Zero stamp uses the latest TF; frame_locked follows the base.
                for publisher, message in outputs:
                    if publisher.get_subscription_count():
                        publisher.publish(message)
                next_publish = time.monotonic() + .5
            rclpy.spin_once(node, timeout_sec=.1)
        return 0
    finally:
        if outputs and rclpy.ok():
            # Delete only our marker IDs, never markers owned by another tool.
            for publisher, message in outputs:
                for m in message.markers:
                    m.action = Marker.DELETE
                    m.points = []
                publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=.1)
        if rviz is not None and rviz.poll() is None:
            rviz.terminate()
            try:
                rviz.wait(timeout=3)
            except subprocess.TimeoutExpired:
                rviz.kill()
                rviz.wait()
        if rviz_directory is not None:
            rviz_directory.cleanup()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, ValueError, OSError, ET.ParseError) as exc:
        print(f'工作空间显示失败：{exc}', file=sys.stderr)
        sys.exit(1)
