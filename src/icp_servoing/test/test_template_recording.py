import os
import tempfile
import time
import unittest

import numpy as np

from icp_servoing.servoing import VisualServo


class FakeRobot:
    def __init__(self, poses):
        self.poses = list(poses)
        self.calls = 0

    def get_tool(self):
        index = min(self.calls, len(self.poses) - 1)
        self.calls += 1
        return list(self.poses[index])


class FakePointCloud:
    def __init__(self, points, fresh=True):
        self.latest_pc = points
        self.fresh = fresh
        self.wait_calls = 0

    def wait_fresh(self, timeout=2.0):
        self.wait_calls += 1
        return self.fresh


class TemplateRecordingTests(unittest.TestCase):
    @staticmethod
    def cloud():
        rng = np.random.default_rng(7)
        xy = rng.uniform(-0.35, 0.35, size=(6000, 2))
        z = (
            0.7
            + 0.03 * np.sin(xy[:, 0] * 10.0)
            + 0.02 * np.cos(xy[:, 1] * 9.0)
        )
        return np.column_stack((xy, z))

    def make_servo(self, poses, fresh=True):
        robot = FakeRobot(poses)
        servo = VisualServo(robot, np.eye(4))
        servo._pc_node = FakePointCloud(self.cloud(), fresh=fresh)
        return servo, robot

    def test_record_template_uses_exactly_two_pose_samples_and_saves(self):
        pose = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
        servo, robot = self.make_servo([pose, pose])
        started = time.perf_counter()
        self.assertTrue(servo.record_template(n_frames=3))
        elapsed = time.perf_counter() - started

        self.assertEqual(2, robot.calls)
        self.assertEqual(3, servo._pc_node.wait_calls)
        points, unused_tree, normals = servo._p2plane_ref
        self.assertEqual(points.shape, normals.shape)
        self.assertTrue(np.isfinite(normals).all())
        self.assertTrue(np.allclose(
            np.linalg.norm(normals, axis=1), 1.0, atol=1e-6))
        # This small synthetic template protects against accidentally
        # reintroducing the former per-point Python SVD minute-scale path.
        self.assertLess(elapsed, 5.0)

        with tempfile.TemporaryDirectory() as folder:
            path = servo.save_template(os.path.join(folder, 'template.npz'))
            self.assertTrue(os.path.isfile(path))
            with np.load(path) as saved:
                self.assertIn('p2_normals', saved.files)
                self.assertTrue(np.allclose(
                    saved['T_base_tool_ref'],
                    servo._T_base_tool_ref))

    def test_record_template_rejects_robot_motion_during_capture(self):
        servo, robot = self.make_servo([
            [100.0, 200.0, 300.0, 0.0, 0.0, 0.0],
            [103.0, 200.0, 300.0, 0.0, 0.0, 0.0],
        ])
        self.assertFalse(servo.record_template(n_frames=2))
        self.assertEqual(2, robot.calls)
        self.assertIsNone(servo._ref_pyramid)

    def test_capture_rejects_stale_point_cloud(self):
        pose = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
        servo, unused_robot = self.make_servo([pose, pose], fresh=False)
        self.assertIsNone(servo._capture_pc())

    def test_pose_reader_accepts_large_valid_position_change(self):
        first = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
        second = [500.0, 200.0, 300.0, 0.0, 0.0, 0.0]
        servo, unused_robot = self.make_servo([first, second])

        self.assertEqual(100.0, servo._read_tool_matrix()[0, 3])
        self.assertEqual(500.0, servo._read_tool_matrix()[0, 3])

    def test_icp_convergence_requires_one_qualified_frame(self):
        pose = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
        servo, unused_robot = self.make_servo([pose])
        self.assertEqual(4.0, servo.final_icp_trans_thresh_mm)
        self.assertEqual(1, servo.final_stable_frames)


if __name__ == '__main__':
    unittest.main()
