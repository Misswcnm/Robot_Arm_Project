#!/bin/bash
cd ~/Robot_Arm_Project
source install/setup.bash
ros2 run continuous_icp_servo continuous_icp_servo_node --ros-args \
  -p pointcloud_topic:=/camera/camera/depth/color/points \
  -p joint_topic:=/joint_states_robot \
  -p command_topic:=/continuous_icp_servo/command \
  -p handeye_path:=/home/ylx/Robot_Arm_Project/scripts/active_handeye_calibration.json \
  -p icp_hz:=6.0 \
  -p servo_hz:=20.0 \
  -p getpose_hz:=30.0 \
  -p pose_sync_tolerance_s:=0.10 \
  -p max_pose_age_s:=0.25 \
  -p max_icp_age_s:=1.0 \
  -p min_icp_inliers:=500 \
  -p min_icp_overlap:=0.03 \
  -p max_icp_rmse_mm:=60.0 \
  -p max_icp_trans_mm:=500.0 \
  -p max_icp_rot_deg:=60.0 \
  -p point_to_plane_max_points:=80000 \
  -p speed_percent:=10
