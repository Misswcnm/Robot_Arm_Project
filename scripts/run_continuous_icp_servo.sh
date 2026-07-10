#!/bin/bash
cd ~/Robot_Arm_Project
source install/setup.bash
ros2 run continuous_icp_servo continuous_icp_servo_node --ros-args \
  -p pointcloud_topic:=/camera/camera/depth/color/points \
  -p tool_topic:=/dobot_msgs_v4/msg/ToolVectorActual \
  -p joint_topic:=/joint_states_robot \
  -p command_topic:=/continuous_icp_servo/command \
  -p handeye_path:=/home/ylx/Robot_Arm_Project/scripts/handeye_chessboard_result.json \
  -p icp_hz:=6.0 \
  -p servo_hz:=20.0 \
  -p max_step_mm:=1.0 \
  -p max_step_deg:=0.15 \
  -p max_icp_age_s:=2.0 \
  -p min_icp_overlap:=0.12 \
  -p max_icp_rmse_mm:=25.0 \
  -p max_icp_trans_mm:=120.0 \
  -p max_icp_rot_deg:=10.0 \
  -p speed_percent:=10
