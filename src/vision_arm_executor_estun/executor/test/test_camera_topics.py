import os
from unittest.mock import patch

from apriltag_pick.camera_topics import camera_topic, camera_topic_root


def test_foxy_uses_single_camera_topic_root():
    with patch.dict(os.environ, {'ROS_DISTRO': 'foxy'}, clear=True):
        assert camera_topic_root() == '/camera'
        assert camera_topic('color/image_raw') == '/camera/color/image_raw'
        assert camera_topic('depth/color/points') == (
            '/camera/depth/color/points')


def test_newer_ros_uses_camera_name_below_namespace():
    with patch.dict(os.environ, {'ROS_DISTRO': 'humble'}, clear=True):
        assert camera_topic_root() == '/camera/camera'


def test_explicit_topic_root_overrides_ros_distro():
    environment = {
        'ROS_DISTRO': 'foxy',
        'REALSENSE_TOPIC_ROOT': 'custom/d455/',
    }
    with patch.dict(os.environ, environment, clear=True):
        assert camera_topic_root() == '/custom/d455'
        assert camera_topic('color/camera_info') == (
            '/custom/d455/color/camera_info')
