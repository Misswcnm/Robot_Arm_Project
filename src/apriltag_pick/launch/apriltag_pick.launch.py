from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('apriltag_pick')
    return LaunchDescription([
        Node(
            package='image_proc',
            executable='rectify_node',
            name='apriltag_color_rectify',
            namespace='/camera/camera/color',
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect'),
            ],
            parameters=[{
                'qos_overrides./camera/camera/color/image_raw.subscription.reliability': 'best_effort',
                'qos_overrides./camera/camera/color/image_raw.subscription.durability': 'volatile',
                'qos_overrides./camera/camera/color/camera_info.subscription.reliability': 'best_effort',
                'qos_overrides./camera/camera/color/camera_info.subscription.durability': 'volatile',
            }],
            output='screen',
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            parameters=[os.path.join(share, 'config', 'tags.yaml')],
            remappings=[
                ('image_rect', '/camera/camera/color/image_rect'),
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
