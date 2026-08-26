"""Start the detector in background and keep pick_node attached to this terminal."""
import subprocess

from .camera_topics import camera_topic

from ament_index_python.packages import get_package_share_directory

from .pick_node import main as pick_main


def main():
    share = get_package_share_directory('apriltag_pick')
    detector = subprocess.Popen([
        'ros2', 'run', 'apriltag_ros', 'apriltag_node', '--ros-args',
        '-r', 'image_rect:=' + camera_topic('color/image_raw'),
        '-r', 'camera_info:=' + camera_topic('color/camera_info'),
        '-p', 'camera_topic:=' + camera_topic('color/image_raw'),
        '--params-file', f'{share}/config/tags.yaml',
    ])
    try:
        pick_main([
            '--ros-args', '--params-file', f'{share}/config/pick.yaml'])
    finally:
        detector.terminate()
        try:
            detector.wait(timeout=3)
        except subprocess.TimeoutExpired:
            detector.kill()
