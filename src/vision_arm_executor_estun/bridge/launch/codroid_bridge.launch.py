from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.2.5'),
        DeclareLaunchArgument('robot_port', default_value='9000'),
        DeclareLaunchArgument('service_namespace', default_value='/estun_codroid'),
        DeclareLaunchArgument('request_timeout_sec', default_value='5.0'),
        DeclareLaunchArgument(
            'motion_request_timeout_sec', default_value='30.0'),
        Node(
            package='estun_codroid_bridge',
            executable='estun_codroid_bridge_node',
            name='estun_codroid_bridge',
            output='screen',
            parameters=[{
                'robot_ip': LaunchConfiguration('robot_ip'),
                'robot_port': ParameterValue(
                    LaunchConfiguration('robot_port'), value_type=int),
                'service_namespace': LaunchConfiguration('service_namespace'),
                'request_timeout_sec': ParameterValue(
                    LaunchConfiguration('request_timeout_sec'),
                    value_type=float),
                'motion_request_timeout_sec': ParameterValue(
                    LaunchConfiguration('motion_request_timeout_sec'),
                    value_type=float),
            }],
        ),
    ])
