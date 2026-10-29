"""Controller selection must wait for fresh stationary feedback after each phase."""
from types import SimpleNamespace as NS
import pytest
import robot_control as control


def probe(monkeypatch, *, unavailable=False, busy_after_wait=False):
    clock = [0.]
    ready_after = [.35]
    switched = []
    selected = [False]
    p = control.RosProbe.__new__(control.RosProbe)
    p.timeout = 1.
    p.node = object()
    p.ros = NS(spin_once=lambda node, timeout_sec: clock.__setitem__(0, clock[0]+timeout_sec))
    p.node_names = lambda: []
    p.spin_for = lambda seconds: clock.__setitem__(0, clock[0]+seconds)
    def idle():
        if busy_after_wait and clock[0] >= .4:
            raise RuntimeError('仍有未完成运动/任务 Action')
    p.idle = idle
    def ready():
        if unavailable or clock[0] < ready_after[0]:
            raise RuntimeError('尚无持续静止、实时的六关节反馈')
    p.robot_ready = ready
    def controllers():
        # Model a reset of the settling window during service discovery.
        ready_after[0] = clock[0]+.35
        return [control.Controller('scaled_joint_trajectory_controller',
                'active' if selected[0] else 'inactive',
                control.CONTROLLER_TYPES['scaled_joint_trajectory_controller'])]
    p.controllers = controllers
    def switch(activate, deactivate):
        ready()  # Selection is forbidden until the full settling window passes.
        switched.append(clock[0])
        selected[0] = True
        ready_after[0] = clock[0]+.35
    p.switch = switch
    monkeypatch.setattr(control.time, 'monotonic', lambda: clock[0])
    return p, clock, switched


def test_selection_waits_before_switch_and_after_switch(monkeypatch):
    p, clock, switched = probe(monkeypatch)
    p.select('scaled_joint_trajectory_controller')
    assert len(switched) == 1
    assert switched[0] >= .45 - 1e-12
    assert clock[0] >= switched[0]+.35


def test_feedback_never_recovers_no_switch_or_ownership(monkeypatch, tmp_path):
    p, clock, switched = probe(monkeypatch, unavailable=True)
    marker = tmp_path/'owner'
    with pytest.raises(RuntimeError, match='六关节反馈'):
        p.select('scaled_joint_trajectory_controller', marker)
    assert not switched and not marker.exists()
    assert clock[0] <= 1.2


def test_busy_action_appearing_while_waiting_blocks_switch(monkeypatch):
    p, clock, switched = probe(monkeypatch, busy_after_wait=True)
    with pytest.raises(RuntimeError, match='未完成运动'):
        p.select('scaled_joint_trajectory_controller')
    assert not switched


def test_wait_keeps_actual_latest_failure_reason(monkeypatch):
    p, clock, switched = probe(monkeypatch)
    def stopped():
        raise RuntimeError('External Control 未运行')
    p.robot_ready = stopped
    with pytest.raises(RuntimeError, match='External Control 未运行'):
        p.wait_robot_ready()
    assert not switched and clock[0] <= 1.1
