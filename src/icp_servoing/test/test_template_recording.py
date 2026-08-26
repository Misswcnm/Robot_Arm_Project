import os
import tempfile
import time
import unittest

import numpy as np

from icp_servoing.servoing import VisualServo, _tree_query


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
        self.capture_calls = []

    def capture_fresh_pointcloud(self, min_frames=1, timeout=2.0):
        self.capture_calls.append((min_frames, timeout))
        self.wait_calls += 1
        if not self.fresh:
            return None
        return self.latest_pc.copy()

    def wait_fresh(self, timeout=2.0):
        self.wait_calls += 1
        return self.fresh


class TemplateRecordingTests(unittest.TestCase):
    def test_tree_query_uses_legacy_n_jobs_when_workers_is_unavailable(self):
        class LegacyTree:
            def __init__(self):
                self.calls = []

            def query(self, points, **kwargs):
                self.calls.append(dict(kwargs))
                if 'workers' in kwargs:
                    raise TypeError(
                        "query() got an unexpected keyword argument 'workers'")
                return np.ones(len(points)), np.zeros(len(points), dtype=int)

        tree = LegacyTree()
        distance, index = _tree_query(tree, np.zeros((3, 3)), k=2)
        self.assertEqual((3,), distance.shape)
        self.assertEqual((3,), index.shape)
        self.assertEqual(
            [{'workers': -1, 'k': 2}, {'n_jobs': -1, 'k': 2}],
            tree.calls)

    def test_tree_query_can_fall_back_to_serial_query(self):
        class SerialTree:
            def __init__(self):
                self.calls = []

            def query(self, points, **kwargs):
                self.calls.append(dict(kwargs))
                for name in ('workers', 'n_jobs'):
                    if name in kwargs:
                        raise TypeError(
                            "query() got an unexpected keyword argument '%s'" %
                            name)
                return np.ones(len(points)), np.zeros(len(points), dtype=int)

        tree = SerialTree()
        _tree_query(tree, np.zeros((2, 3)), distance_upper_bound=0.1)
        self.assertEqual(
            [
                {'workers': -1, 'distance_upper_bound': 0.1},
                {'n_jobs': -1, 'distance_upper_bound': 0.1},
                {'distance_upper_bound': 0.1},
            ],
            tree.calls)

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
        self.assertEqual(
            [(1, 5.0), (1, 5.0), (1, 5.0)],
            servo._pc_node.capture_calls)
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

    def test_execution_capture_drains_three_nx_frames(self):
        pose = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
        servo, unused_robot = self.make_servo([pose])
        self.assertIsNotNone(servo._capture_pc())
        self.assertEqual([(3, 5.0)], servo._pc_node.capture_calls)

    def test_p2plane_refinement_rejects_large_jump_from_p2p_seed(self):
        pose = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
        servo, unused_robot = self.make_servo([pose])
        seed = np.eye(4)
        local = np.eye(4)
        local[0, 3] = 0.004
        divergent = np.eye(4)
        divergent[0, 3] = 0.008

        self.assertTrue(servo._p2plane_refinement_is_local(seed, local)[0])
        self.assertFalse(
            servo._p2plane_refinement_is_local(seed, divergent)[0])

    def test_motion_estimate_gap_uses_mm_for_servo_transforms(self):
        first = np.eye(4)
        second = np.eye(4)
        second[1, 3] = 4.0
        translation_mm, rotation_deg = VisualServo.transform_gap(
            first, second)
        self.assertAlmostEqual(4.0, translation_mm)
        self.assertAlmostEqual(0.0, rotation_deg)

    def test_quality_gate_enforces_its_overlap_argument(self):
        info = {'rmse': 0.005, 'inliers': 1000, 'overlap': 0.02}
        self.assertFalse(VisualServo.quality_ok(
            info, np.eye(4), min_overlap=0.03))

    def test_pose_reader_accepts_large_valid_position_change(self):
        first = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
        second = [500.0, 200.0, 300.0, 0.0, 0.0, 0.0]
        servo, unused_robot = self.make_servo([first, second])

        self.assertEqual(100.0, servo._read_tool_matrix()[0, 3])
        self.assertEqual(500.0, servo._read_tool_matrix()[0, 3])

    def test_icp_convergence_requires_two_qualified_frames(self):
        pose = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
        servo, unused_robot = self.make_servo([pose])
        self.assertEqual(4.0, servo.final_icp_trans_thresh_mm)
        self.assertEqual(2, servo.final_stable_frames)


if __name__ == '__main__':
    unittest.main()
