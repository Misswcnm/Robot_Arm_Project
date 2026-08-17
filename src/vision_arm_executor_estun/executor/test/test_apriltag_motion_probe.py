import json

import numpy as np

from vision_arm_executor_estun.apriltag_motion_probe import (
    codroid_movj_preview,
    load_observation_joints,
    parse_fractions,
    parse_vector3,
)


def test_movj_preview_keeps_cartesian_target_and_joint_speed_fields():
    payload = codroid_movj_preview(
        [184.5, -236.0, 888.6, -89.9, -0.2, -100.6], 5)
    assert payload['type'] == 'movj'
    assert payload['target']['type'] == 'cpos'
    assert payload['target']['cpos']['x'] == 184.5
    assert payload['target']['cpos']['a'] == -89.9
    assert payload['target']['cpos']['poscfg']['mode'] == -1
    assert payload['speed']['sper'] == 5.0
    assert payload['speed']['stcp'] == 0.0
    assert payload['speed']['sori'] == 0.0


def test_load_observation_joints_from_station_list(tmp_path):
    path = tmp_path / 'station.json'
    path.write_text(json.dumps([{
        'label': 'P1',
        'apriltag': {'observe_joints': [1, 2, 3, 4, 5, 6]},
    }]), encoding='utf-8')
    np.testing.assert_allclose(
        load_observation_joints(path, 'P1'), [1, 2, 3, 4, 5, 6])


def test_parse_fractions_rejects_unsorted_values():
    assert parse_fractions('0.05,0.5,1') == [0.05, 0.5, 1.0]
    try:
        parse_fractions('0.5,0.25')
    except ValueError as error:
        assert 'strictly increasing' in str(error)
    else:
        raise AssertionError('unsorted fractions must fail')


def test_parse_vector3():
    np.testing.assert_allclose(parse_vector3('5,-2.5,0'), [5, -2.5, 0])
