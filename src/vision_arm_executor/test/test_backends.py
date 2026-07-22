import os
import tempfile
import unittest

import numpy as np

from vision_arm_executor.backends import AprilTagBackend, IcpBackend


class FakeRobot:
    MODE_ENABLED = 5
    MODE_BACKDRIVE = 6

    def __init__(self):
        self.dragging = False
        self.mode = 5
        self.pose = [10.0, 20.0, 30.0, 0.0, 0.0, 0.0]
        self.init_calls = 0
        self.targets = []

    def init(self):
        self.init_calls += 1
        return True

    def initialize(self):
        return self.init()

    def get_mode(self):
        return self.mode

    def get_tool(self, warn=False):
        return list(self.pose)

    def movj_pose(self, target, label):
        self.targets.append((np.asarray(target), label))
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
        self.last_localization = None
        self.pick_saw_enabled = False

    def _target_tag_id(self):
        return 0

    def locate(self):
        self.last_localization = {
            'frames': 3,
            'spread_mm': 0.2,
            'flange_xyzrpy': list(self.robot.pose),
            'base_flange_target': np.eye(4),
        }
        return np.eye(4)

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
        self.assertEqual(1, robot.init_calls)

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


class AprilTagBackendTests(unittest.TestCase):
    def test_locate_returns_stable_identity_and_location(self):
        tag = FakeTagNode()
        backend = AprilTagBackend(None, tag_factory=lambda unused: tag)
        result = backend.locate()
        self.assertEqual(0, result.identity.tag_id)
        self.assertEqual('tag36h11:0', result.identity.tag_frame)
        self.assertEqual(3, result.localization['frames'])
        self.assertEqual((4, 4), result.target.shape)

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


if __name__ == '__main__':
    unittest.main()
