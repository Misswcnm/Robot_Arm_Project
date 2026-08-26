#!/usr/bin/env bash
set -eo pipefail

workspace="${ROBOT_ARM_WORKSPACE:-/home/nvidia/Robot_Arm_Project}"
source /opt/ros/foxy/setup.bash
export PYTHONNOUSERSITE=1

# Source only the packages used by this backend. Avoid install/setup.bash:
# an unrelated stale package in a copied workspace must not prevent boot.
packages=(
  dobot_msgs_v4
  apriltag_msgs
  apriltag_ros
  estun_codroid_bridge
  icp_servoing
  apriltag_pick
  vision_arm_executor
  vision_arm_executor_estun
)
for package in "${packages[@]}"; do
  setup_file="${workspace}/install/${package}/share/${package}/package.bash"
  if [[ ! -f "${setup_file}" ]]; then
    echo "Missing installed ROS package: ${setup_file}" >&2
    exit 1
  fi
  source "${setup_file}"
done

cd "${workspace}"
exec ros2 launch vision_arm_executor_estun estun_vision_backend.launch.py \
  robot_ip:=192.168.2.5 \
  robot_port:=9000 \
  nx_allowed_clients:=127.0.0.1/32,192.168.2.15/32 \
  start_rviz:=false \
  "$@"
