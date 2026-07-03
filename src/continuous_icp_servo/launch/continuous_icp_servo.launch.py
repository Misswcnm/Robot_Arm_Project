from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='continuous_icp_servo',
            executable='continuous_icp_servo_node',
            name='continuous_icp_servo',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'pointcloud_topic': '/camera/camera/depth/color/points',
                'tool_topic': '/dobot_msgs_v4/msg/ToolVectorActual',
                'command_topic': '/continuous_icp_servo/command',
                'handeye_path': '/home/ylx/Robot_Arm_Project/scripts/handeye_chessboard_result.json',
                'icp_hz': 6.0,
                'servo_hz': 20.0,
                'max_step_mm': 2.0,
                'max_step_deg': 0.35,
                'max_icp_age_s': 0.35,
                'speed_percent': 10,
            }],
        )
    ])
