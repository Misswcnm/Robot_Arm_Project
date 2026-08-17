import json
import os
import tempfile
import unittest

from vision_arm_executor.station_store import (
    APRILTAG_POINT, ICP_POINT, StationStore)


class StationStoreTests(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp()
        self.store = StationStore(self.root)

    def test_mixed_points_share_one_manifest_and_one_label_sequence(self):
        self.store.save_apriltag(
            'map-1', 'station-1', 'P1',
            [1, 2, 3, 4, 5, 6], 0, [0, 0, 0])
        self.store.save_icp(
            'map-1', 'station-1', 'P2',
            {'record_id': 'a'}, {'record_id': 'b'})
        points, path = self.store.load('map-1', 'station-1')
        self.assertTrue(os.path.isfile(path))
        self.assertEqual(
            [('P1', APRILTAG_POINT), ('P2', ICP_POINT)],
            [(point['label'], point['point_type']) for point in points])
        self.assertEqual('P3', self.store.next_label('map-1', 'station-1'))

    def test_point_record_directory_groups_by_station_and_label(self):
        self.assertEqual(
            os.path.join(
                self.root, 'records', 'map-1_station-1_P1_1'),
            self.store.record_dir('map-1', 'station-1', 'P1', 1))

    def test_apriltag_persists_taught_joint_configuration(self):
        joints = [1, 2, 3, 4, 5, 6]
        self.store.save_apriltag(
            'map-1', 'station-1', 'P1',
            [10, 20, 30, 40, 50, 60], 0, [0, 0, 0], joints)
        points, unused_path = self.store.load('map-1', 'station-1')
        self.assertEqual(joints, points[0]['apriltag']['observe_joints'])

    def test_old_mode_manifest_is_rejected(self):
        path = self.store.manifest_path('map-1', 'station-1')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as stream:
            json.dump([{
                'label': 'P1', 'mode': 1,
                'icp_a': {'record_id': 'a'},
                'icp_b': {'record_id': 'b'},
            }], stream)
        with self.assertRaisesRegex(RuntimeError, 'point_type is required'):
            self.store.load('map-1', 'station-1')

    def test_corrupt_manifest_is_rejected_instead_of_overwritten(self):
        path = self.store.manifest_path('map-1', 'station-1')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as stream:
            stream.write('{broken')
        with self.assertRaisesRegex(RuntimeError, 'valid JSON'):
            self.store.save_apriltag(
                'map-1', 'station-1', 'P1',
                [1, 2, 3, 4, 5, 6], 0, [0, 0, 0])
        with open(path) as stream:
            self.assertEqual('{broken', stream.read())

    def test_mixed_station_returns_all_points_in_manifest_order(self):
        self.store.save_apriltag(
            'map-1', 'station-1', 'P1',
            [1, 2, 3, 4, 5, 6], 0, [0, 0, 0])
        self.store.save_icp(
            'map-1', 'station-1', 'P2',
            {'record_id': 'a'}, {'record_id': 'b'})
        unused_map, unused_pose, selected, unused_path = self.store.select({
            'mapid': 'map-1', 'poseid': 'station-1'})
        self.assertEqual(
            [('P1', APRILTAG_POINT), ('P2', ICP_POINT)],
            [(point['label'], point['point_type']) for point in selected])
        unused_map, unused_pose, selected, unused_path = self.store.select({
            'mapid': 'map-1', 'poseid': 'station-1', 'point_type': 0})
        self.assertEqual(['P1'], [point['label'] for point in selected])

    def test_label_and_point_type_mismatch_fails(self):
        self.store.save_apriltag(
            'map-1', 'station-1', 'P1',
            [1, 2, 3, 4, 5, 6], 0, [0, 0, 0])
        with self.assertRaisesRegex(RuntimeError, 'point_type mismatch'):
            self.store.select({
                'mapid': 'map-1', 'poseid': 'station-1',
                'label': 'P1', 'point_type': 1})


if __name__ == '__main__':
    unittest.main()
