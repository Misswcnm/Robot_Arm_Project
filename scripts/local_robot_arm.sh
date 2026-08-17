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

source_workspace() {
  # Colcon setup files read optional tracing variables before defining them.
  set +u
  source "$ROOT_DIR/install/setup.bash"
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
    source_workspace
    python3 -c "import icp_servoing, apriltag_pick, vision_arm_executor"
    echo "ROS2 visual packages built and importable."
    ;;
  vision)
    source_ros2
    source_workspace
    ros2 run vision_arm_executor vision_arm_executor --ros-args \
      --params-file "$ROOT_DIR/src/vision_arm_executor/config/executor.yaml"
    ;;
  nx-gateway)
    source_ros2
    source_workspace
    if [[ -f "$ROOT_DIR/config/local_robot_arm.env" ]]; then
      source "$ROOT_DIR/config/local_robot_arm.env"
    fi
    ros2 run vision_arm_executor nx_compat_gateway \
      --route-file "${NX_ROUTE_FILE:-$ROOT_DIR/src/vision_arm_executor/config/nx_routes.json}"
    ;;
  all)
    exec "$ROOT_DIR/scripts/start_local_robot_arm_stack.sh" "${@:2}"
    ;;
  *)
    echo "usage: $0 {check|build|vision|nx-gateway|all}" >&2; exit 2
    ;;
esac
