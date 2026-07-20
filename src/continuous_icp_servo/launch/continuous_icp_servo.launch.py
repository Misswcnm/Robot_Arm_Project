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
                'command_topic': '/continuous_icp_servo/command',
                'handeye_path': '/home/ylx/Robot_Arm_Project/scripts/'
                                'active_handeye_calibration.json',
                'icp_hz': 6.0,
                'servo_hz': 20.0,
                'getpose_hz': 30.0,
                'pose_sync_tolerance_s': 0.10,
                'max_pose_age_s': 0.25,
                'max_icp_age_s': 1.0,
                'min_icp_inliers': 500,
                'min_icp_overlap': 0.03,
                'max_icp_rmse_mm': 60.0,
                'max_icp_trans_mm': 500.0,
                'max_icp_rot_deg': 60.0,
                'point_to_plane_max_points': 80000,
                'speed_percent': 10,
            }],
        )
    ])
