#!/bin/bash
set -e

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
project_root=$(cd "$script_dir/../.." && pwd)
motor_pid=""

cleanup()
{
  if [ -n "$motor_pid" ] && kill -0 "$motor_pid" 2>/dev/null; then
    kill "$motor_pid"
    wait "$motor_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [ ! -x "$script_dir/build/RobotControl" ]; then
  echo "Demo is not exist: $script_dir/build/RobotControl"
  exit 1
fi

source /opt/ros/foxy/setup.bash
export PYTHONNOUSERSITE=1
gripper_setup="$project_root/install_gripper/setup.bash"
if [ ! -f "$gripper_setup" ]; then
  gripper_setup="$project_root/install/setup.bash"
fi
if [ ! -f "$gripper_setup" ]; then
  echo "ROS2 workspace is not installed: $gripper_setup"
  exit 1
fi
source "$gripper_setup"
if ! ros2 pkg prefix step_motor >/dev/null 2>&1; then
  echo "step_motor is not available after sourcing $gripper_setup"
  exit 1
fi

if ros2 node list 2>/dev/null | grep -qx '/motor_node'; then
  echo "[STARTUP] /motor_node is already running"
  ros2 topic pub --once /motor_control step_motor/msg/Motor \
    "{id: 1, speed: 0, dir: 0, mode: 0, angle: 0, state: 2, sub_divide: 0}" \
    >/dev/null
else
  echo "[STARTUP] starting ROS2 gripper driver"
  ros2 run step_motor motor_node &
  motor_pid=$!
  motor_ready=false
  for _ in $(seq 1 30); do
    if ! kill -0 "$motor_pid" 2>/dev/null; then
      echo "[STARTUP] motor_node exited before becoming ready"
      exit 1
    fi
    if ros2 node list 2>/dev/null | grep -qx '/motor_node'; then
      motor_ready=true
      break
    fi
    sleep 0.2
  done
  if [ "$motor_ready" != true ]; then
    echo "[STARTUP] timeout waiting for /motor_node"
    exit 1
  fi
fi

echo "[STARTUP] gripper driver ready; starting RobotControl"
"$script_dir/build/RobotControl" "$@"
