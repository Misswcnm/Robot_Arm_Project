# Vision Arm Executor

`vision_arm_executor` is the ROS 2-only owner of CR5 visual operations. It listens only on
`127.0.0.1:17881` using UTF-8 JSON Lines; ROS 1 `master_node` calls it through
`master_node.vision_arm_rpc` and never imports `rclpy`.

Start safely (motion remains disabled after every restart):

```bash
cd ~/Robot_Arm_Project
colcon build --packages-select vision_arm_executor
source install/setup.bash
ros2 run vision_arm_executor vision_arm_executor --ros-args \
  --params-file src/vision_arm_executor/config/executor.yaml
```

Example request:

```bash
printf '{"request_id":"locate-1","action":"apriltag_locate","params":{},"dry_run":true}\n' | nc 127.0.0.1 17881
```

Responses use `accepted`, `running`, `succeeded`, `failed`, `timeout`, or `cancelled`.
`accepted` only means the task entered the queue. Query it with `task_status` and the original
request id. Run `execution_enable` explicitly before a non-dry-run movement action.

Runtime records are atomically saved under `data/vision_arm` by default. A/B records bind their
4x4 transforms; AprilTag caches have a TTL and are rejected after a hand-eye calibration change.
