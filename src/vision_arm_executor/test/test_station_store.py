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

    def test_backend_commands_are_the_only_group_identifiers(self):
        self.store.save_apriltag(
            'map-1', 'station-1', '1',
            [1, 2, 3, 4, 5, 6], 0, [0, 0, 0])
        self.store.save_icp(
            'map-1', 'station-1', 2,
            {'record_id': 'a'}, {'record_id': 'b'})
        points, path = self.store.load('map-1', 'station-1')
        self.assertTrue(os.path.isfile(path))
        self.assertEqual(
            [('1', APRILTAG_POINT), ('2', ICP_POINT)],
            [(point['command'], point['point_type']) for point in points])
        self.assertNotIn('label', points[0])
        self.assertEqual(
            ('map-1', 'station-1', '3'),
            self.store.teaching_identity({
                'mapid': 'map-1', 'poseid': 'station-1'}))

    def test_point_record_directory_uses_station_command_and_type(self):
        self.assertEqual(
            os.path.join(self.root, 'records', 'map-1_station-1_12_1'),
            self.store.record_dir('map-1', 'station-1', '12', 1))

    def test_apriltag_persists_taught_joint_configuration(self):
        joints = [1, 2, 3, 4, 5, 6]
        self.store.save_apriltag(
            'map-1', 'station-1', '1',
            [10, 20, 30, 40, 50, 60], 0, [0, 0, 0], joints)
        points, unused_path = self.store.load('map-1', 'station-1')
        self.assertEqual(joints, points[0]['reference']['observe_joints'])

    def test_missing_point_type_is_a_normal_group(self):
        path = self.store.manifest_path('map-1', 'station-1')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as stream:
            json.dump([{
                'command': '1',
                'work_points': [{
                    'work_id': 'W1', 'index': 1, 'command': 1,
                }],
            }], stream)
        points, unused_path = self.store.load('map-1', 'station-1')
        self.assertNotIn('point_type', points[0])

    def test_legacy_p_labels_are_read_as_command_numbers(self):
        path = self.store.manifest_path('map-1', 'station-1')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        legacy_record_dir = os.path.join(
            self.root, 'records', 'map-1_station-1_P2_1')
        os.makedirs(legacy_record_dir)
        with open(os.path.join(legacy_record_dir, 'icp_a.npz'), 'wb') as stream:
            stream.write(b'template')
        with open(path, 'w') as stream:
            json.dump([
                {'schema_version': 2, 'label': 'P1', 'point_type': 0,
                 'apriltag': {'tag_id': 0,
                              'observe_flange_pose': [0] * 6}},
                {'schema_version': 2, 'label': 'P2', 'point_type': 1,
                 'icp_a': {'record_id': 'a',
                           'record_dir': legacy_record_dir},
                 'icp_b': {'record_id': 'b',
                           'record_dir': legacy_record_dir}},
            ], stream)
        points, unused_path = self.store.load('map-1', 'station-1')
        self.assertEqual(['1', '2'], [point['command'] for point in points])
        self.assertEqual(0, len(points[0]['work_points']))
        self.assertEqual(
            'a', points[1]['reference']['record_id'])
        self.assertEqual(
            'b', points[1]['work_points'][0]['record_id'])
        unused_removed, unused_path, removed_dirs = (
            self.store.delete_station('map-1', 'station-1'))
        self.assertIn(legacy_record_dir, removed_dirs)
        self.assertFalse(os.path.exists(legacy_record_dir))

    def test_delete_by_command_removes_complete_icp_directory(self):
        self.store.save_group(
            'map-1', 'station-1', '1', ICP_POINT,
            {'record_id': 'ref'},
            [{'record_id': 'work', 'work_id': 'W1',
              'index': 1, 'command': 2}])
        directory = self.store.record_dir(
            'map-1', 'station-1', '1', ICP_POINT)
        os.makedirs(directory)
        for name in ('icp_a.npz', 'icp_a.json', 'work_1.json'):
            with open(os.path.join(directory, name), 'w') as stream:
                stream.write(name)
        removed, unused_path, removed_dirs = self.store.delete(
            'map-1', 'station-1', '1')
        self.assertEqual('1', removed['command'])
        self.assertIn(directory, removed_dirs)
        self.assertFalse(os.path.exists(directory))
        points, unused_path = self.store.load('map-1', 'station-1')
        self.assertEqual([], points)

    def test_delete_station_removes_all_numbered_groups(self):
        self.store.save_apriltag(
            'map-1', 'station-1', '1',
            [1, 2, 3, 4, 5, 6], 0, [0, 0, 0])
        self.store.save_icp(
            'map-1', 'station-1', '2',
            {'record_id': 'a'}, {'record_id': 'b'})
        directories = [
            self.store.record_dir('map-1', 'station-1', '1', 0),
            self.store.record_dir('map-1', 'station-1', '2', 1),
        ]
        for directory in directories:
            os.makedirs(directory)
        removed, unused_path, removed_dirs = self.store.delete_station(
            'map-1', 'station-1')
        self.assertEqual(['1', '2'], [point['command'] for point in removed])
        self.assertEqual(set(directories), set(removed_dirs))
        self.assertTrue(all(not os.path.exists(path) for path in directories))
        points, unused_path = self.store.load('map-1', 'station-1')
        self.assertEqual([], points)

    def test_corrupt_manifest_is_rejected_instead_of_overwritten(self):
        path = self.store.manifest_path('map-1', 'station-1')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as stream:
            stream.write('{broken')
        with self.assertRaisesRegex(RuntimeError, 'valid JSON'):
            self.store.save_apriltag(
                'map-1', 'station-1', '1',
                [1, 2, 3, 4, 5, 6], 0, [0, 0, 0])
        with open(path) as stream:
            self.assertEqual('{broken', stream.read())

    def test_mixed_station_returns_all_groups_in_manifest_order(self):
        self.store.save_apriltag(
            'map-1', 'station-1', '1',
            [1, 2, 3, 4, 5, 6], 0, [0, 0, 0])
        self.store.save_icp(
            'map-1', 'station-1', '2',
            {'record_id': 'a'}, {'record_id': 'b'})
        unused_map, unused_pose, selected, unused_path = self.store.select({
            'mapid': 'map-1', 'poseid': 'station-1'})
        self.assertEqual(
            [('1', APRILTAG_POINT), ('2', ICP_POINT)],
            [(point['command'], point['point_type']) for point in selected])
        unused_map, unused_pose, selected, unused_path = self.store.select({
            'mapid': 'map-1', 'poseid': 'station-1', 'point_type': 0})
        self.assertEqual(['1'], [point['command'] for point in selected])

    def test_task_command_and_point_type_mismatch_fails(self):
        self.store.save_apriltag(
            'map-1', 'station-1', '1',
            [1, 2, 3, 4, 5, 6], 0, [0, 0, 0])
        with self.assertRaisesRegex(RuntimeError, 'point_type mismatch'):
            self.store.select({
                'mapid': 'map-1', 'poseid': 'station-1',
                'task_command': '1', 'point_type': 1})


if __name__ == '__main__':
    unittest.main()
