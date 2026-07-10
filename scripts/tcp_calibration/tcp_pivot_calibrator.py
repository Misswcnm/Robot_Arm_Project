#!/usr/bin/env python3
"""Interactive TCP pivot calibration using Dobot GetPose().

Drag the robot so the real tool tip touches one fixed point, record several
poses, then solve R_i * p_tcp + t_i = p_fixed.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import select
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from dobot_msgs_v4.srv import GetPose, StartDrag, StopDrag


@dataclass
class PoseSample:
    stamp_sec: float
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float


def rot_zyx_deg(rx: float, ry: float, rz: float) -> np.ndarray:
    """Dobot TCP RPY in degrees, composed as Rz * Ry * Rx."""
    ax, ay, az = math.radians(rx), math.radians(ry), math.radians(rz)
    cx, sx = math.cos(ax), math.sin(ax)
    cy, sy = math.cos(ay), math.sin(ay)
    cz, sz = math.cos(az), math.sin(az)

    rx_m = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    ry_m = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rz_m = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    return rz_m @ ry_m @ rx_m


def solve_pivot(samples: List[PoseSample]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return tcp offset in tool frame, fixed point in base frame, residuals."""
    if len(samples) < 4:
        raise ValueError("至少需要 4 组姿态，推荐 12 到 20 组")

    rows = []
    rhs = []
    eye = np.eye(3)
    for s in samples:
        r = rot_zyx_deg(s.rx, s.ry, s.rz)
        t = np.array([s.x, s.y, s.z], dtype=float)
        rows.append(np.hstack([r, -eye]))
        rhs.append(-t)

    a = np.vstack(rows)
    b = np.concatenate(rhs)
    x, *_ = np.linalg.lstsq(a, b, rcond=None)
    tcp_tool = x[:3]
    fixed_base = x[3:]

    residuals = []
    for s in samples:
        r = rot_zyx_deg(s.rx, s.ry, s.rz)
        t = np.array([s.x, s.y, s.z], dtype=float)
        tip_base = r @ tcp_tool + t
        residuals.append(float(np.linalg.norm(tip_base - fixed_base)))
    return tcp_tool, fixed_base, np.array(residuals)


class TcpPivotCalibrator(Node):
    def __init__(self) -> None:
        super().__init__("tcp_pivot_calibrator")

        self.declare_parameter("get_pose_service", "/dobot_bringup_ros2/srv/GetPose")
        self.declare_parameter("start_drag_service", "/dobot_bringup_ros2/srv/StartDrag")
        self.declare_parameter("stop_drag_service", "/dobot_bringup_ros2/srv/StopDrag")
        self.declare_parameter("stable_window", 15)
        self.declare_parameter("max_pos_std_mm", 0.25)
        self.declare_parameter("max_rot_std_deg", 0.15)

        self.get_pose_service = self.get_parameter("get_pose_service").value
        self.stable_window = int(self.get_parameter("stable_window").value)
        self.max_pos_std_mm = float(self.get_parameter("max_pos_std_mm").value)
        self.max_rot_std_deg = float(self.get_parameter("max_rot_std_deg").value)

        self.samples: List[PoseSample] = []
        self.last_solution = None

        self.get_pose_client = self.create_client(
            GetPose, self.get_pose_service
        )
        self.start_drag_client = self.create_client(
            StartDrag, self.get_parameter("start_drag_service").value
        )
        self.stop_drag_client = self.create_client(
            StopDrag, self.get_parameter("stop_drag_service").value
        )

        self.get_logger().info(f"使用 GetPose: {self.get_pose_service}")
        self.print_help()

    def call(self, client, request, timeout: float = 3.0):
        future = client.call_async(request)
        t0 = time.time()
        while not future.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            future.cancel()
            return None
        return future.result()

    def get_pose_sample(self) -> Optional[PoseSample]:
        if not self.get_pose_client.wait_for_service(timeout_sec=0.5):
            return None
        response = self.call(self.get_pose_client, GetPose.Request(), timeout=3.0)
        if response is None or response.res != 0:
            return None
        try:
            vals = [float(v) for v in response.robot_return.strip("{}").split(",")]
        except ValueError:
            return None
        if len(vals) != 6 or not np.isfinite(vals).all():
            return None
        if np.linalg.norm(vals[:3]) <= 1.0:
            return None
        now = self.get_clock().now().nanoseconds * 1e-9
        return PoseSample(now, *vals)

    def print_help(self) -> None:
        print("")
        print("TCP pivot 标定交互命令：")
        print("  d  进入拖拽模式 StartDrag")
        print("  e  退出拖拽模式 StopDrag")
        print("  r  记录当前稳定姿态")
        print("  u  删除上一条记录")
        print("  l  列出记录")
        print("  s  解算 TCP")
        print("  w  保存 samples/result 到 json")
        print("  h  帮助")
        print("  q  退出")
        print("")

    def stable_pose(self) -> Tuple[Optional[PoseSample], str]:
        window = []
        for _ in range(self.stable_window):
            sample = self.get_pose_sample()
            if sample is None:
                return None, "GetPose 无有效位姿"
            window.append(sample)
            time.sleep(0.04)

        arr = np.array([[p.x, p.y, p.z, p.rx, p.ry, p.rz] for p in window])
        pos_std = np.std(arr[:, :3], axis=0)
        rot_std = np.std(arr[:, 3:], axis=0)
        if float(np.max(pos_std)) > self.max_pos_std_mm:
            return None, f"位置未稳定 std={pos_std.round(3)} mm"
        if float(np.max(rot_std)) > self.max_rot_std_deg:
            return None, f"姿态未稳定 std={rot_std.round(3)} deg"

        mean = np.mean(arr, axis=0)
        stamp = window[-1].stamp_sec
        return PoseSample(stamp, *[float(v) for v in mean]), "ok"

    def record(self) -> None:
        pose, reason = self.stable_pose()
        if pose is None:
            self.get_logger().warn(f"不记录：{reason}")
            return
        self.samples.append(pose)
        self.get_logger().info(
            "记录 #%d xyz=[%.3f %.3f %.3f] rpy=[%.3f %.3f %.3f]"
            % (
                len(self.samples),
                pose.x,
                pose.y,
                pose.z,
                pose.rx,
                pose.ry,
                pose.rz,
            )
        )

    def list_samples(self) -> None:
        if not self.samples:
            print("还没有记录。")
            return
        for i, s in enumerate(self.samples, 1):
            print(
                f"{i:02d}: xyz=[{s.x:.3f} {s.y:.3f} {s.z:.3f}] "
                f"rpy=[{s.rx:.3f} {s.ry:.3f} {s.rz:.3f}]"
            )

    def solve(self) -> None:
        try:
            tcp_tool, fixed_base, residuals = solve_pivot(self.samples)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return

        self.last_solution = {
            "tcp_offset_tool_mm": tcp_tool.tolist(),
            "fixed_point_base_mm": fixed_base.tolist(),
            "residual_mean_mm": float(np.mean(residuals)),
            "residual_max_mm": float(np.max(residuals)),
            "residual_std_mm": float(np.std(residuals)),
            "sample_count": len(self.samples),
        }

        print("")
        print("TCP 标定结果：")
        print(
            "  tcp_offset_tool_mm = [%.3f, %.3f, %.3f]"
            % tuple(self.last_solution["tcp_offset_tool_mm"])
        )
        print(
            "  fixed_point_base_mm = [%.3f, %.3f, %.3f]"
            % tuple(self.last_solution["fixed_point_base_mm"])
        )
        print(
            "  residual mean/max/std = %.3f / %.3f / %.3f mm"
            % (
                self.last_solution["residual_mean_mm"],
                self.last_solution["residual_max_mm"],
                self.last_solution["residual_std_mm"],
            )
        )
        print("")

    def save(self) -> None:
        if self.last_solution is None:
            self.solve()
        if self.last_solution is None:
            return

        out_dir = os.path.dirname(os.path.abspath(__file__))
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(out_dir, f"tcp_calibration_{stamp}.json")
        payload = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "pose_source": "GetPose()",
            "method": "pivot_calibration",
            "pose_convention": "xyz mm, rx/ry/rz deg, rotation matrix R = Rz * Ry * Rx",
            "samples": [asdict(s) for s in self.samples],
            "result": self.last_solution,
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        self.get_logger().info(f"已保存：{path}")

    def call_empty_service(self, name: str, client, srv_cls) -> None:
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"{name} 服务不可用")
            return
        future = client.call_async(srv_cls.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            self.get_logger().warn(f"{name} 调用超时或失败")
            return
        self.get_logger().info(f"{name} res={future.result().res}")

    def handle_command(self, cmd: str) -> bool:
        cmd = cmd.strip().lower()
        if not cmd:
            return True
        if cmd == "d":
            self.call_empty_service("StartDrag", self.start_drag_client, StartDrag)
        elif cmd == "e":
            self.call_empty_service("StopDrag", self.stop_drag_client, StopDrag)
        elif cmd == "r":
            self.record()
        elif cmd == "u":
            if self.samples:
                removed = self.samples.pop()
                self.last_solution = None
                self.get_logger().warn(
                    "删除上一条 xyz=[%.3f %.3f %.3f] rpy=[%.3f %.3f %.3f]"
                    % (removed.x, removed.y, removed.z, removed.rx, removed.ry, removed.rz)
                )
        elif cmd == "l":
            self.list_samples()
        elif cmd == "s":
            self.solve()
        elif cmd == "w":
            self.save()
        elif cmd == "h":
            self.print_help()
        elif cmd == "q":
            return False
        else:
            self.get_logger().warn(f"未知命令：{cmd}，输入 h 查看帮助")
        return True


def main() -> None:
    parser = argparse.ArgumentParser(description="Dobot TCP pivot calibration")
    parser.add_argument("--spin-timeout", type=float, default=0.02)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = TcpPivotCalibrator()
    running = True
    try:
        while rclpy.ok() and running:
            rclpy.spin_once(node, timeout_sec=args.spin_timeout)
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if readable:
                running = node.handle_command(sys.stdin.readline())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
