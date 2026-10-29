"""Local URDF forward kinematics for feedback, using measured joint angles."""

import math

from .cartesian import multiply, normalized, rotate
from .grasp import Pose


def _triple(text):
    values = tuple(float(v) for v in text.split())
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError('invalid_feedback_kinematics_vector')
    return values


def _rotation(axis, angle):
    scale = math.sin(angle / 2)
    return (*[v * scale for v in axis], math.cos(angle / 2))


class JointKinematics:
    """Only a fixed/revolute chain with exactly the expected measured joints.

    The caller must verify this model against MoveIt and TF before using it as
    a feedback source. No commanded or planned joint positions are substituted.
    """

    def __init__(self, model, base_frame, tool_frame, expected_joints):
        self.base_frame, self.tool_frame = base_frame, tool_frame
        parents = {}
        for joint in model.findall('joint'):
            child = joint.find('child').get('link')
            if child in parents:
                raise ValueError('ambiguous_feedback_kinematics_chain')
            parents[child] = joint
        child, visited, chain = tool_frame, set(), []
        while child != base_frame:
            if child in visited or child not in parents:
                raise ValueError('invalid_feedback_kinematics_chain')
            visited.add(child)
            joint = parents[child]
            kind, name = joint.get('type'), joint.get('name')
            if kind not in ('fixed', 'revolute', 'continuous') or joint.find('mimic') is not None:
                raise ValueError('unsupported_feedback_kinematics_joint:' + str(name))
            origin = joint.find('origin')
            xyz = _triple(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0')
            roll, pitch, yaw = _triple(origin.get('rpy', '0 0 0') if origin is not None else '0 0 0')
            q = multiply(multiply(_rotation((0, 0, 1), yaw), _rotation((0, 1, 0), pitch)),
                         _rotation((1, 0, 0), roll))
            axis = (1., 0., 0.)
            if kind != 'fixed':
                element = joint.find('axis')
                axis = _triple(element.get('xyz', '1 0 0') if element is not None else '1 0 0')
                length = math.hypot(*axis)
                if length < 1e-12:
                    raise ValueError('invalid_feedback_kinematics_axis')
                axis = tuple(v / length for v in axis)
            chain.append((None if kind == 'fixed' else name, xyz, q, axis))
            child = joint.find('parent').get('link')
        names = [item[0] for item in chain if item[0] is not None]
        if len(names) != len(set(names)) or set(names) != set(expected_joints):
            raise ValueError('feedback_kinematics_joint_mismatch')
        self.chain = tuple(reversed(chain))

    def pose(self, names, positions):
        if len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError('invalid_feedback_kinematics_state')
        measured = dict(zip(names, positions))
        position, orientation = (0., 0., 0.), (0., 0., 0., 1.)
        for name, xyz, q, axis in self.chain:
            delta = rotate(orientation, xyz)
            position = tuple(a + b for a, b in zip(position, delta))
            orientation = multiply(orientation, q)
            if name is not None:
                angle = measured.get(name)
                if angle is None or not math.isfinite(angle):
                    raise ValueError('missing_or_invalid_joint:' + name)
                orientation = multiply(orientation, _rotation(axis, angle))
        return Pose(position, normalized(orientation), self.base_frame, self.tool_frame)
