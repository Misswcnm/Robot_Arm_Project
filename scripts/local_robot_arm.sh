#!/usr/bin/env bash
# Local ROS2 visual-stack helper. It deliberately never starts or edits application/master_node.
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ACTION="${1:-check}"
source_ros2() {
  # ROS setup scripts may inspect optional variables before defining them.
  source /opt/ros/humble/setup.bash
  set -u
}

case "$ACTION" in
  check)
    source_ros2
    command -v ros2
    command -v colcon
    test -f "$ROOT_DIR/scripts/active_handeye_calibration.json"
    test -f "$ROOT_DIR/scripts/tcp_calibration/tcp_calibration_20260714_095024.json"
    echo "ROS2 local prerequisites OK; no robot motion has been requested."
    ;;
  build)
    source_ros2
    cd "$ROOT_DIR"
    colcon build --base-paths src/icp_servoing src/apriltag_pick src/vision_arm_executor \
      --packages-select icp_servoing apriltag_pick vision_arm_executor
    ;;
  vision)
    source_ros2
    source "$ROOT_DIR/install/setup.bash"
    ros2 run vision_arm_executor vision_arm_executor --ros-args \
      --params-file "$ROOT_DIR/src/vision_arm_executor/config/executor.yaml"
    ;;
  all)
    exec "$ROOT_DIR/scripts/start_local_robot_arm_stack.sh" "${@:2}"
    ;;
  *)
    echo "usage: $0 {check|build|vision|all}" >&2; exit 2
    ;;
esac
