"""One-key ESTUN Codroid, D455, AprilTag and NX-compatible backend."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('vision_arm_executor_estun')
    bridge_share = get_package_share_directory('estun_codroid_bridge')
    vision_share = get_package_share_directory('vision_arm_executor')
    camera_share = get_package_share_directory('realsense2_camera')
    tag_share = get_package_share_directory('apriltag_pick')

    robot_ip = LaunchConfiguration('robot_ip')
    robot_port = LaunchConfiguration('robot_port')
    params = LaunchConfiguration('vision_params_file')
    start_driver = LaunchConfiguration('start_estun_driver')
    start_camera = LaunchConfiguration('start_camera')
    enable_pointcloud = LaunchConfiguration('enable_pointcloud')
    start_rviz = LaunchConfiguration('start_rviz')
    start_detector = LaunchConfiguration('start_apriltag_detector')
    start_gateway = LaunchConfiguration('start_nx_gateway')
    gateway_host = LaunchConfiguration('nx_gateway_host')
    gateway_port = LaunchConfiguration('nx_gateway_port')
    allowed_clients = LaunchConfiguration('nx_allowed_clients')
    codroid_request_timeout = LaunchConfiguration(
        'codroid_request_timeout_sec')
    codroid_motion_timeout = LaunchConfiguration(
        'codroid_motion_timeout_sec')

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.2.5'),
        DeclareLaunchArgument('robot_port', default_value='9000'),
        DeclareLaunchArgument('start_estun_driver', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('enable_pointcloud', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument(
            'start_apriltag_detector', default_value='true'),
        DeclareLaunchArgument('start_nx_gateway', default_value='true'),
        DeclareLaunchArgument(
            'vision_params_file',
            default_value=os.path.join(
                package_share, 'config', 'executor_estun.yaml')),
        DeclareLaunchArgument('nx_gateway_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('nx_gateway_port', default_value='8888'),
        DeclareLaunchArgument(
            'nx_allowed_clients', default_value='127.0.0.1/32'),
        DeclareLaunchArgument(
            'codroid_request_timeout_sec', default_value='5.0'),
        DeclareLaunchArgument(
            'codroid_motion_timeout_sec', default_value='30.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                bridge_share, 'launch', 'codroid_bridge.launch.py')),
            condition=IfCondition(start_driver),
            launch_arguments={
                'robot_ip': robot_ip,
                'robot_port': robot_port,
                'service_namespace': '/estun_codroid',
                'request_timeout_sec': codroid_request_timeout,
                'motion_request_timeout_sec': codroid_motion_timeout,
            }.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                camera_share, 'launch', 'rs_launch.py')),
            condition=IfCondition(start_camera),
            launch_arguments={
                'pointcloud.enable': enable_pointcloud,
            }.items()),

        Node(
            package='rviz2',
            executable='rviz2',
            name='estun_vision_rviz',
            output='screen',
            condition=IfCondition(start_rviz),
            arguments=[
                '-d', os.path.join(camera_share, 'launch', 'default.rviz'),
            ]),

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            output='screen',
            condition=IfCondition(start_detector),
            remappings=[
                ('image_rect', '/camera/camera/color/image_raw'),
                ('camera_info', '/camera/camera/color/camera_info'),
            ],
            parameters=[os.path.join(tag_share, 'config', 'tags.yaml')]),

        Node(
            package='vision_arm_executor_estun',
            executable='vision_arm_executor_estun',
            output='screen',
            emulate_tty=True,
            parameters=[params]),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'vision_arm_executor', 'nx_compat_gateway',
                '--host', gateway_host,
                '--port', gateway_port,
                '--allowed-clients', allowed_clients,
                '--route-file', os.path.join(
                    vision_share, 'config', 'nx_routes.json'),
                '--rpc-host', '127.0.0.1',
                '--rpc-port', '17881',
            ],
            output='screen',
            condition=IfCondition(start_gateway),
        ),
    ])
