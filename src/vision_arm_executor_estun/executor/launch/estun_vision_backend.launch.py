"""One-key ESTUN Codroid, D455, AprilTag and NX-compatible backend."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from apriltag_pick.camera_topics import camera_topic


def generate_launch_description():
    package_share = get_package_share_directory('vision_arm_executor_estun')
    bridge_share = get_package_share_directory('estun_codroid_bridge')
    vision_share = get_package_share_directory('vision_arm_executor')
    camera_share = get_package_share_directory('realsense2_camera')
    tag_share = get_package_share_directory('apriltag_pick')

    # realsense2_camera 4.51 (ROS 2 Foxy) uses the shorter profile
    # parameter names, while newer releases renamed them.  Select the names
    # exposed by the installed launch file so the same backend source can be
    # deployed on both systems.
    camera_launch = os.path.join(camera_share, 'launch', 'rs_launch.py')
    with open(camera_launch, encoding='utf-8') as stream:
        camera_launch_source = stream.read()
    color_profile_arg = (
        'rgb_camera.profile'
        if 'rgb_camera.profile' in camera_launch_source
        else 'rgb_camera.color_profile')
    depth_profile_arg = (
        'depth_module.profile'
        if 'depth_module.profile' in camera_launch_source
        else 'depth_module.depth_profile')

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

    camera_arguments = {
        'enable_color': 'true',
        'enable_depth': 'true',
        color_profile_arg: '640x480x15',
        depth_profile_arg: '640x480x15',
        # AprilTag uses the RGB stream's own CameraInfo and does not need
        # cross-sensor synchronization.  Foxy/D455 has been observed dropping
        # V4L frames when enable_sync is forced on.
        'enable_sync': 'false',
        'publish_tf': 'true',
        'pointcloud.enable': enable_pointcloud,
        # ICP consumes XYZ only. Disable RGB texturing so Foxy does not wait
        # for a colour frame in every depth frameset.
        'pointcloud.stream_filter': '0',
        'pointcloud.allow_no_texture_points': 'true',
    }
    # Foxy 4.51 exposes this launch argument and defaults it to the D455's
    # unsupported value 3 on this device.  Newer launch files omit it.
    if 'rgb_camera.power_line_frequency' in camera_launch_source:
        camera_arguments['rgb_camera.power_line_frequency'] = '1'

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.2.5'),
        DeclareLaunchArgument('robot_port', default_value='9000'),
        DeclareLaunchArgument('start_estun_driver', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('enable_pointcloud', default_value='true'),
        # RViz is a debugging tool. Keep it off in the production backend so
        # remote-desktop rendering cannot starve camera and detector threads.
        DeclareLaunchArgument('start_rviz', default_value='false'),
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
            PythonLaunchDescriptionSource(camera_launch),
            condition=IfCondition(start_camera),
            launch_arguments=camera_arguments.items()),

        TimerAction(
            period=3.0,
            actions=[Node(
                package='rviz2',
                executable='rviz2',
                name='estun_vision_rviz',
                output='screen',
                condition=IfCondition(start_rviz),
                arguments=[
                    '-d', os.path.join(
                        package_share, 'config', 'camera_foxy.rviz'),
                ])]),

        TimerAction(
            period=4.0,
            actions=[Node(
                package='apriltag_ros',
                executable='apriltag_node',
                name='apriltag',
                output='screen',
                condition=IfCondition(start_detector),
                remappings=[
                    ('image_rect', camera_topic('color/image_raw')),
                    ('camera_info', camera_topic('color/camera_info')),
                ],
                parameters=[
                    os.path.join(tag_share, 'config', 'tags.yaml')])]),

        TimerAction(
            period=5.0,
            actions=[Node(
                package='vision_arm_executor_estun',
                executable='vision_arm_executor_estun',
                output='screen',
                emulate_tty=True,
                parameters=[params])]),

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
