"""TF dropout recovery must use fresh measured joints, never assumed arrival."""
from dataclasses import replace
import math
from threading import Event, RLock
from types import SimpleNamespace as NS
import time
import xml.etree.ElementTree as ET

import pytest
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import LookupException

from grasp_executor.cartesian import pose_error
from grasp_executor.config import GraspConfig
from grasp_executor.grasp import Pose, execute_grasp_place
from grasp_executor.joint_kinematics import JointKinematics
from grasp_executor.moveit_ros import JOINTS, MoveItArm
from test_grasp import completed


def model_xml():
    joints = []
    for index, name in enumerate(JOINTS):
        joints.append(f'<joint name="{name}" type="revolute">'
                      f'<parent link="link{index}"/><child link="link{index+1}"/>'
                      '<axis xyz="0 0 1"/></joint>')
    return ET.fromstring('<robot name="fixture">'
        '<joint name="base" type="fixed"><parent link="base_link"/><child link="link0"/>'
        '<origin xyz="0.4 0.1 0.3"/></joint>' + ''.join(joints) +
        '<joint name="tcp" type="fixed"><parent link="link6"/><child link="grasp_center"/>'
        '<origin xyz="0.1 0 0"/></joint></robot>')


def adapter():
    a = MoveItArm.__new__(MoveItArm)
    a.config = GraspConfig(settle_time=.001, poll_interval=.001, place_dwell_time=.001)
    a.cancel, a.stopped, a.lock = Event(), Event(), RLock()
    a.logs = []
    a.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000)),
                get_logger=lambda: NS(info=a.logs.append))
    a.joint_state = JointState()
    a.joint_state.header.stamp.sec = 100
    a.joint_state.name = list(JOINTS)
    a.joint_state.position = [0.] * 6
    a.joint_state.velocity = [0.] * 6
    a.joint_received = time.monotonic()
    a.feedback_kinematics = None
    a.pose_feedback_source, a.tf_feedback_age = 'tf', None
    a._verify_feedback_kinematics(model_xml(), a._state(),
                                Pose((.5, .1, .3), (0., 0., 0., 1.), 'base_link', 'grasp_center'))
    a.transform = TransformStamped()
    a.transform.header.stamp.sec = 100
    a.transform.transform.translation.x = .5
    a.transform.transform.translation.y = .1
    a.transform.transform.translation.z = .3
    a.transform.transform.rotation.w = 1.
    a.buffer = NS(lookup_transform=lambda *args: a.transform)
    a._robot_running = lambda: None
    a.action_error, a.result_future, a.settle_started = None, None, None
    return a


def test_local_fk_uses_named_measured_joints_and_fixed_tool_offset():
    k = JointKinematics(model_xml(), 'base_link', 'grasp_center', JOINTS)
    values = [math.pi/2, 0., 0., 0., 0., 0.]
    pose = k.pose(list(reversed(JOINTS)), list(reversed(values)))
    assert pose.position == pytest.approx((.4, .2, .3))
    assert pose.orientation == pytest.approx((0., 0., math.sqrt(.5), math.sqrt(.5)))


def test_local_fk_rotated_origin_axis_and_fixed_rpy_order():
    model = ET.fromstring('''<robot name="fixture">
      <joint name="j" type="revolute"><parent link="b"/><child link="l"/>
        <origin xyz="0.2 0.3 0.4" rpy="1.5707963267948966 0 1.5707963267948966"/>
        <axis xyz="0 0 2"/></joint>
      <joint name="tcp" type="fixed"><parent link="l"/><child link="t"/>
        <origin xyz="0.1 0.2 0.3"/></joint></robot>''')
    pose = JointKinematics(model, 'b', 't', ['j']).pose(['j'], [math.pi/2])
    # Rz(90) Rx(90) Rz(90) maps (.1,.2,.3) to (.3,-.2,.1).
    assert pose.position == pytest.approx((.5, .1, .5))


@pytest.mark.parametrize('defect', ['missing', 'cycle', 'mimic', 'prismatic', 'axis', 'nan'])
def test_unsupported_or_invalid_model_is_not_a_feedback_source(defect):
    model = model_xml()
    joint = model.findall('joint')[1]
    if defect == 'missing': joint.find('parent').set('link', 'missing')
    if defect == 'cycle': joint.find('parent').set('link', 'grasp_center')
    if defect == 'mimic': ET.SubElement(joint, 'mimic', joint='other')
    if defect == 'prismatic': joint.set('type', 'prismatic')
    if defect == 'axis': joint.find('axis').set('xyz', '0 0 0')
    if defect == 'nan': joint.find('axis').set('xyz', 'nan 0 1')
    with pytest.raises(ValueError):
        JointKinematics(model, 'base_link', 'grasp_center', JOINTS)


def test_model_mismatch_disables_fallback():
    a = adapter()
    with pytest.raises(RuntimeError, match='feedback_kinematics_and_moveit_disagree'):
        a._verify_feedback_kinematics(model_xml(), a._state(),
            Pose((.51, .1, .3), (0., 0., 0., 1.), 'base_link', 'grasp_center'))
    assert a.feedback_kinematics is None


@pytest.mark.parametrize('stamp', [99, 101, 0])
def test_invalid_tf_time_uses_fresh_joints_and_recovers(stamp):
    a = adapter()
    a.transform.header.stamp.sec = stamp
    a.joint_state.position[0] = math.pi/2
    assert a.read_pose().position == pytest.approx((.4, .2, .3))
    assert a.pose_feedback_source == 'joint_fk'
    a.read_pose()
    assert len(a.logs) == 1  # Log transitions, not every feedback poll.
    a.transform.header.stamp.sec = 100
    assert a.read_pose().position == pytest.approx((.5, .1, .3))
    assert a.pose_feedback_source == 'tf' and len(a.logs) == 2


def test_missing_tf_uses_fresh_joints():
    a = adapter()
    def missing(*args): raise LookupException('temporarily unavailable')
    a.buffer.lookup_transform = missing
    assert a.read_pose().position == pytest.approx((.5, .1, .3))


def test_startup_requires_tf_before_model_is_verified():
    a = adapter()
    a.feedback_kinematics = None
    a.transform.header.stamp.sec = 99
    with pytest.raises(RuntimeError, match='stale_robot_tf_feedback'):
        a.read_pose()


@pytest.mark.parametrize('defect', ['receipt', 'stamp', 'future', 'missing', 'nan', 'cancel'])
def test_untrustworthy_joint_feedback_cannot_rescue_tf(defect):
    a = adapter()
    a.transform.header.stamp.sec = 99
    if defect == 'receipt': a.joint_received -= 1
    if defect == 'stamp': a.joint_state.header.stamp.sec = 99
    if defect == 'future': a.joint_state.header.stamp.sec = 101
    if defect == 'missing': a.joint_state.name[0] = 'unknown'
    if defect == 'nan': a.joint_state.position[0] = math.nan
    if defect == 'cancel': a.cancel.set()
    with pytest.raises((RuntimeError, ValueError)):
        a.read_pose()


def test_invalid_tf_geometry_is_not_hidden_by_fallback():
    a = adapter()
    a.transform.transform.translation.x = math.nan
    with pytest.raises(ValueError, match='invalid_robot_tf_feedback'):
        a.read_pose()


def test_fk_feedback_cannot_bypass_action_or_arrival_checks():
    a = adapter()
    a.transform.header.stamp.sec = 99
    goal = a.read_pose()
    a.active = NS(name='place_drop', linear=True, goal=goal)
    assert not a.is_pose_reached(.1)  # No action result, even though already at goal.
    a.result_future = completed(NS(status=6, result=NS(error_code=NS(val=1))))
    with pytest.raises(RuntimeError, match='trajectory_execution_failed'):
        a.is_pose_reached(.1)
    a.result_future = completed(NS(status=4, result=NS(error_code=NS(val=1))))
    a.joint_state.position[0] = .3
    with pytest.raises(RuntimeError, match='linear_tracking_error'):
        a.is_pose_reached(.1)
    a.joint_state.position[0] = 0.
    assert not a.is_pose_reached(.1)
    a.settle_started = time.monotonic() - .01
    assert a.is_pose_reached(.1)


@pytest.mark.parametrize('lost_joints', [False, True])
def test_place_release_during_tf_dropout_requires_fresh_pose(lost_joints):
    a = adapter()
    events = []
    # Simulated measured poses for the prepared stages; only TF fails at place_drop.
    actual = [Pose((.4,.1,.4), (0.,1.,0.,0.), 'base_link', 'grasp_center')]
    a.feedback_kinematics = NS(pose=lambda names, positions: actual[0])
    def read_tf():
        if events.count('motion') >= 5:
            raise RuntimeError('stale_robot_tf_feedback')
        return actual[0]
    a._read_tf_pose = read_tf
    def motion(goal, timeout):
        events.append('motion')
        actual[0] = goal
        a.active = NS(name='stage', linear=True, goal=goal)
        a.settle_started = None
        a.result_future = completed(NS(status=4, result=NS(error_code=NS(val=1))))
        a.joint_received = time.monotonic() - (1 if lost_joints and events.count('motion') == 5 else 0)
        return True
    def event(name):
        events.append(name)
        return True
    interfaces = NS(move_to_pose=motion, move_linear=motion, is_pose_reached=a.is_pose_reached,
                    open_gripper=lambda timeout:event('open'), close_gripper=lambda timeout:event('close'),
                    stop_motion=lambda timeout:event('stop'))
    approach = actual[0]
    grasp = replace(approach, position=(.4,.1,.3))
    ready = replace(approach, position=(.5,.2,.4))
    drop = replace(ready, position=(.5,.2,.3))
    result = execute_grasp_place(approach, grasp, ready, drop, interfaces, a.config)
    if lost_joints:
        assert result.status == 'failed' and result.failed_step == 'moving_to_place'
        assert events.count('open') == 1 and events[-1] == 'stop'
    else:
        assert result.status == 'success'
        assert events.count('open') == 2 and 'stop' not in events
