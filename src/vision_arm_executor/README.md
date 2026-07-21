# Vision Arm Executor

`vision_arm_executor` is the ROS 2-only owner of CR5 visual operations. It uses UTF-8 JSON
Lines on TCP port `17881`. The default is loopback-only. For the vehicle-to-local-computer
wired connection, configure the local wired IP, the vehicle `/32` allowlist and an auth token;
the ROS 1 process must never import `rclpy`.

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

For a wired request, every message also carries the configured token:

```json
{
  "request_id": "station-01-20260721-001",
  "action": "vision_icp_align_and_move_b",
  "params": {"max_iters": 30},
  "timeout_sec": 120,
  "dry_run": false,
  "auth_token": "the-shared-random-token"
}
```

Responses use `accepted`, `running`, `succeeded`, `failed`, `timeout`, or `cancelled`.
`accepted` only means the task entered the queue. Query it with `task_status` and the original
request id. Each TCP connection handles one request and then closes; long-task polling opens a
new connection. Run `execution_enable` explicitly before a non-dry-run movement action.

Runtime records are atomically saved under `data/vision_arm` by default. ICP A stores a versioned
compressed point-cloud template; B binds to that exact A record, template checksum and hand-eye
checksum. AprilTag locate stores a versioned target cache. Pick rejects an expired cache, changed
Tag ID, changed hand-eye/TCP calibration, or robot motion after localization, and it never silently
re-detects the target.

Recommended operational actions:

- ICP commissioning: `vision_icp_record_a`, then standard drag/exit, then
  `vision_icp_record_b` with `manual_positioned=true`.
- ICP production: `vision_icp_align` or `vision_icp_align_and_move_b`.
- AprilTag production: `apriltag_locate`, then `apriltag_pick` using the cached target.
- AprilTag validation: locate, manually enter drag and touch the Tag center, then call
  `apriltag_validate`; exit drag using the standard recovery sequence afterwards.
- Safety and control: `health`, `arm_status`, `execution_enable`, `execution_disable`,
  `task_status`, and `task_cancel`.
