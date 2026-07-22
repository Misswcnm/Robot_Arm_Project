#!/usr/bin/env bash
# Start the local CR5 + RealSense + step motor + visual RPC stack.
# This script never builds, launches, or edits application/.
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${ROBOT_ARM_ENV_FILE:-$PROJECT_DIR/config/local_robot_arm.env}"
DRY_RUN=false
HARDWARE_ONLY=false

for argument in "$@"; do
  case "$argument" in
    --dry-run) DRY_RUN=true ;;
    --hardware-only) HARDWARE_ONLY=true ;;
    -h|--help)
      echo "usage: $0 [--dry-run] [--hardware-only]"
      exit 0
      ;;
    *) echo "unknown argument: $argument" >&2; exit 2 ;;
  esac
done

if [[ -f "$ENV_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$ENV_FILE"
fi

# ROS setup scripts inspect optional variables, so enable nounset afterwards.
source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/install/setup.bash"
set -u

: "${IP_address:?set IP_address to the CR5 controller IP in config/local_robot_arm.env}"
: "${DOBOT_TYPE:?set DOBOT_TYPE (normally cr5) in config/local_robot_arm.env}"

VISION_RPC_HOST="${VISION_RPC_HOST:-127.0.0.1}"
VEHICLE_IP="${VEHICLE_IP:-127.0.0.1}"
VISION_RPC_AUTH_TOKEN="${VISION_RPC_AUTH_TOKEN:-}"
NX_COMPAT_ENABLED="${NX_COMPAT_ENABLED:-false}"
NX_COMPAT_HOST="${NX_COMPAT_HOST:-0.0.0.0}"
NX_COMPAT_PORT="${NX_COMPAT_PORT:-8888}"
NX_COMPAT_ALLOWED_CLIENTS="${NX_COMPAT_ALLOWED_CLIENTS:-$VEHICLE_IP/32}"
NX_ROUTE_FILE="${NX_ROUTE_FILE:-$PROJECT_DIR/src/vision_arm_executor/config/nx_routes.json}"
NX_VISION_RPC_HOST="${NX_VISION_RPC_HOST:-$VISION_RPC_HOST}"
NX_VISION_RPC_PORT="${NX_VISION_RPC_PORT:-17881}"
if [[ "$HARDWARE_ONLY" == false && "$VISION_RPC_HOST" != "127.0.0.1" && -z "$VISION_RPC_AUTH_TOKEN" ]]; then
  echo "VISION_RPC_AUTH_TOKEN is required when VISION_RPC_HOST is not 127.0.0.1" >&2
  exit 2
fi

ROS_LOG_DIR="${ROS_LOG_DIR:-$PROJECT_DIR/.runtime/roslog}"
export ROS_LOG_DIR IP_address DOBOT_TYPE VISION_RPC_HOST VEHICLE_IP
export VISION_RPC_AUTH_TOKEN
export NX_COMPAT_HOST NX_COMPAT_PORT NX_COMPAT_ALLOWED_CLIENTS NX_ROUTE_FILE
export NX_VISION_RPC_HOST NX_VISION_RPC_PORT
mkdir -p "$ROS_LOG_DIR"

required_packages=(cr_robot_ros2 realsense2_camera step_motor)
if [[ "$HARDWARE_ONLY" == false ]]; then
  required_packages+=(apriltag_ros apriltag_pick vision_arm_executor)
fi
for package in "${required_packages[@]}"; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "ROS2 package is not built or sourced: $package" >&2
    echo "Run: ./scripts/local_robot_arm.sh build" >&2
    exit 1
  fi
done
ros2 pkg executables step_motor | grep -q '^step_motor motor_node$'
if ! python3 -c "import icp_servoing, apriltag_pick, vision_arm_executor"; then
  echo "ROS2 visual Python modules are not importable." >&2
  echo "Run: ./scripts/local_robot_arm.sh build" >&2
  exit 1
fi

declare -a CHILD_PIDS=()
STOPPING=false

print_command() {
  local label="$1"
  shift
  printf '[%s]' "$label"
  printf ' %q' "$@"
  printf '\n'
}

start_component() {
  local label="$1"
  shift
  print_command "$label" "$@"
  if [[ "$DRY_RUN" == true ]]; then
    return
  fi
  "$@" &
  CHILD_PIDS+=("$!")
}

stop_children() {
  if [[ "$STOPPING" == true ]]; then
    return
  fi
  STOPPING=true
  if ((${#CHILD_PIDS[@]})); then
    echo "Stopping local robot-arm stack..."
    kill -INT "${CHILD_PIDS[@]}" 2>/dev/null || true
    wait "${CHILD_PIDS[@]}" 2>/dev/null || true
  fi
}
trap stop_children INT TERM EXIT

start_component CR5 \
  ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
start_component RealSense \
  ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
start_component StepMotor \
  ros2 run step_motor motor_node

if [[ "$HARDWARE_ONLY" == false ]]; then
  start_component AprilTagDetector \
    ros2 run apriltag_ros apriltag_node --ros-args \
      -r image_rect:=/camera/camera/color/image_raw \
      -r camera_info:=/camera/camera/color/camera_info \
      --params-file "$PROJECT_DIR/src/apriltag_pick/config/tags.yaml"
  start_component VisionExecutor \
    ros2 run vision_arm_executor vision_arm_executor --ros-args \
      --params-file "$PROJECT_DIR/src/vision_arm_executor/config/executor.yaml"
  if [[ "$NX_COMPAT_ENABLED" == true ]]; then
    if [[ ! -f "$NX_ROUTE_FILE" ]]; then
      echo "NX route file does not exist: $NX_ROUTE_FILE" >&2
      exit 1
    fi
    start_component NxCompatGateway \
      ros2 run vision_arm_executor nx_compat_gateway \
        --host "$NX_COMPAT_HOST" \
        --port "$NX_COMPAT_PORT" \
        --allowed-clients "$NX_COMPAT_ALLOWED_CLIENTS" \
        --route-file "$NX_ROUTE_FILE" \
        --rpc-host "$NX_VISION_RPC_HOST" \
        --rpc-port "$NX_VISION_RPC_PORT"
  fi
fi

if [[ "$DRY_RUN" == true ]]; then
  echo "Dry-run complete; no ROS node or robot motion was started."
  exit 0
fi

echo "Local robot-arm stack is running. Press Ctrl-C to stop all components."
wait -n "${CHILD_PIDS[@]}"
echo "A component exited; stopping the remaining stack." >&2
exit 1
