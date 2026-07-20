from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('apriltag_pick')
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            parameters=[os.path.join(share, 'config', 'tags.yaml')],
            remappings=[
                ('image_rect', '/camera/camera/color/image_raw'),
                ('camera_info', '/camera/camera/color/camera_info'),
            ],
            output='screen',
        ),
        Node(
            package='apriltag_pick',
            executable='pick_node',
            name='apriltag_pick',
            parameters=[os.path.join(share, 'config', 'pick.yaml')],
            output='screen',
            emulate_tty=True,
        ),
    ])
