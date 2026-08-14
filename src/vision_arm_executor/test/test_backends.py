import os
import tempfile
import unittest
from unittest import mock

import numpy as np

from apriltag_pick.robot import CR5Robot as AprilTagRobot
from vision_arm_executor.backends import AprilTagBackend, IcpBackend


class FakeRobot:
    MODE_ENABLED = 5
    MODE_BACKDRIVE = 6

    def __init__(self):
        self.dragging = False
        self.mode = 5
        self.pose = [10.0, 20.0, 30.0, 0.0, 0.0, 0.0]
        self.joints = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        self.init_calls = 0
        self.enable_calls = 0
        self.targets = []
        self.joint_targets = []

    def init(self):
        self.init_calls += 1
        self.mode = self.MODE_ENABLED
        return True

    def initialize(self):
        return self.init()

    def get_mode(self):
        return self.mode

    def get_tool(self, warn=False):
        return list(self.pose)

    def get_joints(self):
        return list(self.joints)

    def movj(self, joints):
        self.joint_targets.append(list(joints))
        self.joints = list(joints)
        return True

    def movj_pose(self, target, label):
        self.targets.append((np.asarray(target), label))
        return True

    def move_pose(self, target):
        self.targets.append((np.asarray(target), 'move_pose'))
        return True

    def start_drag(self):
        self.dragging = True
        self.mode = self.MODE_BACKDRIVE
        return True

    def stop_drag(self):
        self.dragging = False
        self.mode = 4
        return True

    def enable(self):
        self.mode = self.MODE_ENABLED
        return True

    def enable_robot(self):
        self.enable_calls += 1
        self.mode = self.MODE_ENABLED
        return True


class FakeServo:
    def __init__(self, robot, unused_handeye):
        self.robot = robot
        self._T_base_tool_ref = np.eye(4)
        self.last_align_result = {'reason': 'icp_residual_converged'}
        self.motion_guard = None
        self.loaded = None

    def record_template(self, frames):
        self.frames = frames
        return True

    def save_template(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'wb') as stream:
            stream.write(b'template')

    def load_template(self, path):
        self.loaded = path

    def align(self, maximum, should_stop=None, progress_callback=None):
        target = np.eye(4)
        target[0, 3] = 1.0
        self.motion_guard(target)
        progress_callback(1, maximum, {'converged': True})
        return True


class FakeTagNode:
    def __init__(self):
        self.robot = FakeRobot()
        self.execute_enabled = False
        self.tag_frame = 'tag36h11:0'
        self.handeye_path = '/tmp/handeye.json'
        self.tcp_calibration_path = '/tmp/tcp.json'
        self.tag_to_tcp = np.eye(4)
        self.last_localization = None
        self.pick_saw_enabled = False
        self.requested_tag_ids = []

    def _target_tag_id(self, tag_frame=None):
        return int(str(tag_frame or self.tag_frame).rsplit(':', 1)[1])

    def locate(self, tag_id=None):
        selected = self._target_tag_id() if tag_id is None else int(tag_id)
        self.requested_tag_ids.append(selected)
        target = self.tag_to_tcp.copy()
        target[0, 3] += selected
        self.last_localization = {
            'tag_id': selected,
            'tag_frame': 'tag36h11:%d' % selected,
            'frames': 3,
            'spread_mm': 0.2,
            'flange_xyzrpy': list(self.robot.pose),
            'base_flange_target': target,
        }
        return target

    def record_manual_tag_center(self):
        return np.asarray([1.0, 2.0, 3.0])

    def pick(self, safety_check=None):
        self.pick_saw_enabled = self.execute_enabled
        safety_check('before_contact', np.eye(4))


class IcpBackendTests(unittest.TestCase):
    def make(self):
        robot = FakeRobot()
        cfg = {'robot_speed': 15, 'handeye_path': '/dev/null'}
        backend = IcpBackend(
            object(), cfg, robot_factory=lambda *unused: robot,
            servo_factory=FakeServo,
            handeye_loader=lambda unused: np.eye(4))
        return backend, robot

    def test_record_reference_hides_demo_template_state(self):
        backend, robot = self.make()
        path = os.path.join(tempfile.mkdtemp(), 'reference.npz')
        result = backend.record_reference(4, path)
        self.assertEqual(4, result.frames)
        self.assertTrue(os.path.isfile(result.template_path))
        self.assertEqual((4, 4), result.flange_transform.shape)
        self.assertEqual(robot.joints, result.joints)
        self.assertEqual(1, robot.init_calls)

    def test_move_to_reference_prefers_taught_joint_configuration(self):
        backend, robot = self.make()
        target = np.eye(4)
        taught = [11.0, 12.0, 13.0, 14.0, 15.0, 16.0]
        result = backend.move_to_reference(
            target, 'recover A', lambda unused: None, joints=taught)
        self.assertTrue(np.allclose(target, result))
        self.assertEqual([taught], robot.joint_targets)
        self.assertEqual([], robot.targets)

    def test_relative_b_uses_taught_joints_before_cartesian_correction(self):
        backend, robot = self.make()
        delta = np.eye(4)
        delta[0, 3] = 5.0
        nominal = np.eye(4)
        nominal[0, 3] = 4.0
        taught = [21.0, 22.0, 23.0, 24.0, 25.0, 26.0]
        result = backend.move_relative(
            delta, 'A-to-B', lambda unused: None,
            reference_transform=nominal, reference_joints=taught)
        self.assertEqual([taught], robot.joint_targets)
        self.assertEqual(1, len(robot.targets))
        self.assertTrue(np.allclose(result, robot.targets[0][0]))

    def test_align_uses_public_guard_and_result_contract(self):
        backend, unused_robot = self.make()
        guarded = []
        progressed = []
        result = backend.align(
            '/tmp/reference.npz', 5, lambda: False,
            lambda *values: progressed.append(values),
            lambda target: guarded.append(target))
        self.assertTrue(result.converged)
        self.assertEqual('icp_residual_converged', result.metrics['reason'])
        self.assertEqual(1, len(guarded))
        self.assertEqual(1, len(progressed))

    def test_move_to_reference_uses_absolute_taught_a_pose(self):
        backend, robot = self.make()
        target = np.eye(4)
        target[:3, 3] = [100.0, 200.0, 300.0]
        guarded = []

        result = backend.move_to_reference(
            target, 'recover A', lambda value: guarded.append(value.copy()))

        self.assertTrue(np.allclose(target, result))
        self.assertTrue(np.allclose(target, guarded[0]))
        self.assertTrue(np.allclose(target, robot.targets[0][0]))
        self.assertEqual('recover A', robot.targets[0][1])

    def test_estun_recovery_without_taught_joints_uses_controller_solve(self):
        backend, robot = self.make()
        target = np.eye(4)
        target[:3, 3] = [100.0, 200.0, 300.0]
        solved = []
        robot.movj_solved_pose = lambda value, label: (
            solved.append((np.asarray(value).copy(), label)) or True)

        result = backend.move_to_reference(
            target, 'recover A', lambda unused: None)

        self.assertTrue(np.allclose(target, result))
        self.assertEqual(1, len(solved))
        self.assertEqual('recover A', solved[0][1])
        self.assertEqual([], robot.targets)

    def test_estun_relative_target_uses_controller_solve(self):
        backend, robot = self.make()
        delta = np.eye(4)
        delta[1, 3] = 8.0
        solved = []
        robot.movj_solved_pose = lambda value, label: (
            solved.append((np.asarray(value).copy(), label)) or True)

        result = backend.move_relative(
            delta, 'A-to-B', lambda unused: None)

        self.assertEqual(1, len(solved))
        self.assertTrue(np.allclose(result, solved[0][0]))
        self.assertEqual('A-to-B', solved[0][1])
        self.assertEqual([], robot.targets)

    def test_drag_teaching_records_without_reinitializing_robot(self):
        backend, robot = self.make()
        backend.start_drag()
        self.assertTrue(robot.dragging)
        self.assertEqual(robot.MODE_BACKDRIVE, robot.mode)
        path = os.path.join(tempfile.mkdtemp(), 'reference.npz')
        reference = backend.record_reference_teaching(3, path)
        transform = backend.flange_transform_teaching()
        self.assertEqual(1, robot.init_calls)
        self.assertEqual(3, reference.frames)
        self.assertEqual((4, 4), transform.shape)
        backend.stop_drag_and_enable()
        self.assertFalse(robot.dragging)
        self.assertEqual(robot.MODE_ENABLED, robot.mode)

    def test_start_drag_recovers_stale_initialized_cache_from_disabled(self):
        backend, robot = self.make()
        backend.initialized = True
        robot.mode = 4
        state = backend.start_drag()
        self.assertEqual(1, robot.init_calls)
        self.assertTrue(backend.initialized)
        self.assertTrue(robot.dragging)
        self.assertEqual(robot.MODE_BACKDRIVE, state.mode)

    def test_enable_is_idempotent_when_robot_is_already_enabled(self):
        backend, robot = self.make()
        state = backend.enable_robot()
        self.assertEqual(robot.MODE_ENABLED, state.mode)
        self.assertEqual(0, robot.enable_calls)
        self.assertTrue(backend.last_enable_was_noop)

    def test_enable_calls_controller_when_robot_is_disabled(self):
        backend, robot = self.make()
        robot.mode = 4
        state = backend.enable_robot()
        self.assertEqual(robot.MODE_ENABLED, state.mode)
        self.assertEqual(1, robot.enable_calls)
        self.assertFalse(backend.last_enable_was_noop)


class AprilTagBackendTests(unittest.TestCase):
    def test_robot_init_does_not_cycle_an_already_enabled_controller(self):
        robot = object.__new__(AprilTagRobot)
        robot.speed = 15
        robot.dragging = False
        robot.log = mock.Mock()
        robot.get_mode = mock.Mock(return_value=robot.MODE_ENABLED)
        robot.apply_motion_safety = mock.Mock(return_value=True)
        robot.use_base_tool0 = mock.Mock(return_value=True)
        robot.call = mock.Mock()

        self.assertTrue(robot.initialize())

        robot.call.assert_not_called()
        robot.apply_motion_safety.assert_called_once_with()
        robot.use_base_tool0.assert_called_once_with()

    def test_locate_returns_stable_identity_and_location(self):
        tag = FakeTagNode()
        backend = AprilTagBackend(None, tag_factory=lambda unused: tag)
        result = backend.locate()
        self.assertEqual(0, result.identity.tag_id)
        self.assertEqual('tag36h11:0', result.identity.tag_frame)
        self.assertEqual(3, result.localization['frames'])
        self.assertEqual((4, 4), result.target.shape)

    def test_observation_prefers_taught_joint_configuration(self):
        tag = FakeTagNode()
        backend = AprilTagBackend(None, tag_factory=lambda unused: tag)
        taught = [31.0, 32.0, 33.0, 34.0, 35.0, 36.0]
        backend.move_to_observation(
            tag.robot.pose, lambda unused_stage, unused_target: None,
            joints=taught)
        self.assertEqual([taught], tag.robot.joint_targets)
        self.assertEqual([], tag.robot.targets)

    def test_pick_permission_is_bounded_to_pick_call(self):
        tag = FakeTagNode()
        backend = AprilTagBackend(None, tag_factory=lambda unused: tag)
        backend.prepare_pick({
            'flange_xyzrpy': list(tag.robot.pose),
            'base_flange_target': np.eye(4).tolist(),
        })
        stages = []
        backend.pick(lambda stage, target: stages.append(stage))
        self.assertTrue(tag.pick_saw_enabled)
        self.assertFalse(tag.execute_enabled)
        self.assertEqual(['before_contact'], stages)

    def test_per_point_offset_is_bounded_to_one_locate_call(self):
        tag = FakeTagNode()
        backend = AprilTagBackend(None, tag_factory=lambda unused: tag)
        result = backend.locate(
            tag_id=0, tag_offset_xyz_mm=[10.0, -20.0, 30.0])
        self.assertEqual(
            [10.0, -20.0, 30.0], result.target[:3, 3].tolist())
        self.assertEqual([0.0, 0.0, 0.0], tag.tag_to_tcp[:3, 3].tolist())

    def test_requested_tag_id_selects_matching_tag_frame(self):
        tag = FakeTagNode()
        backend = AprilTagBackend(None, tag_factory=lambda unused: tag)
        result = backend.locate(
            tag_id=7, tag_offset_xyz_mm=[0.0, 0.0, 0.0])
        self.assertEqual([7], tag.requested_tag_ids)
        self.assertEqual(7, result.identity.tag_id)
        self.assertEqual('tag36h11:7', result.identity.tag_frame)

    def test_observation_move_uses_recorded_flange_pose_and_guard(self):
        tag = FakeTagNode()
        backend = AprilTagBackend(None, tag_factory=lambda unused: tag)
        guarded = []
        target = backend.move_to_observation(
            [100, 200, 300, 0, 0, 90],
            lambda stage, value: guarded.append((stage, value.copy())))
        self.assertEqual('before_observation', guarded[0][0])
        self.assertEqual([100.0, 200.0, 300.0], target[:3, 3].tolist())
        self.assertEqual(1, tag.robot.init_calls)
        self.assertEqual(1, len(tag.robot.targets))


if __name__ == '__main__':
    unittest.main()
