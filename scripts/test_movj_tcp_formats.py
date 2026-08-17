#!/usr/bin/env python3
"""Test Dobot CR/CR5 MovJ TCP command formats on port 29999.

Default mode is safe: it only reads RobotMode/GetPose/GetAngle/GetErrorID and
prints candidate MovJ commands. Add --execute to actually send MovJ commands.

Recommended first run:
  python3 scripts/test_movj_tcp_formats.py --host 192.168.1.6

Safe motion-format test using the current TCP pose as target:
  python3 scripts/test_movj_tcp_formats.py --host 192.168.1.6 --execute --current
"""

from __future__ import annotations

import argparse
import re
import socket
import time
from dataclasses import dataclass


DEFAULT_HOST = "192.168.1.6"
DEFAULT_PORT = 29999


@dataclass
class TcpReply:
    command: str
    response: str
    error_id: int | None
    ok: bool


def send_command(host: str, port: int, command: str, timeout: float) -> TcpReply:
    payload = (command.strip() + "\n").encode("utf-8")
    response = ""
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            sock.sendall(payload)
            chunks: list[bytes] = []
            deadline = time.time() + timeout
            while time.time() < deadline:
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    break
                if not data:
                    break
                chunks.append(data)
                if b";" in data:
                    break
            response = b"".join(chunks).decode("utf-8", errors="replace").strip()
    except OSError as exc:
        response = f"SOCKET_ERROR: {exc}"

    error_id = parse_error_id(response)
    return TcpReply(command=command, response=response, error_id=error_id, ok=(error_id == 0))


def parse_error_id(response: str) -> int | None:
    match = re.match(r"\s*(-?\d+)\s*,", response)
    if not match:
        return None
    return int(match.group(1))


def parse_brace_numbers(response: str, expected: int) -> list[float] | None:
    match = re.search(r"\{([^{}]+)\}", response)
    if not match:
        return None
    try:
        vals = [float(v.strip()) for v in match.group(1).split(",")]
    except ValueError:
        return None
    return vals if len(vals) == expected else None


def fmt_pose(pose: list[float]) -> str:
    return ",".join(f"{v:.3f}" for v in pose)


def fmt_joint(joints: list[float]) -> str:
    return ",".join(f"{v:.3f}" for v in joints)


def build_movj_variants(pose: list[float], joints: list[float] | None,
                        a: int, v: int, cp: int) -> list[tuple[str, str]]:
    p = fmt_pose(pose)
    variants = [
        ("v4_pose_only",
         f"MovJ(pose={{{p}}})"),
        ("v4_pose_full_named",
         f"MovJ(pose={{{p}}},user=0,tool=0,a={a},v={v},cp={cp})"),
        ("v4_pose_speed_no_frame",
         f"MovJ(pose={{{p}}},a={a},v={v},cp={cp})"),
        ("legacy_pose_positional_only",
         f"MovJ({p})"),
        ("legacy_pose_positional_with_speed",
         f"MovJ({p},a={a},v={v},cp={cp})"),
        ("legacy_pose_positional_with_frame_speed",
         f"MovJ({p},user=0,tool=0,a={a},v={v},cp={cp})"),
        ("legacy_pose_positional_numeric_tail",
         f"MovJ({p},0,0,{a},{v},{cp})"),
    ]
    if joints:
        j = fmt_joint(joints)
        variants.extend([
            ("jointmovj_positional_only",
             f"JointMovJ({j})"),
            ("jointmovj_positional_with_speed",
             f"JointMovJ({j},a={a},v={v},cp={cp})"),
            ("v4_joint_named_full",
             f"MovJ(joint={{{j}}},user=0,tool=0,a={a},v={v},cp={cp})"),
        ])
    return variants


def print_reply(name: str, reply: TcpReply) -> None:
    status = "OK" if reply.ok else f"ERR={reply.error_id}"
    print(f"\n[{name}] {status}")
    print(f"  >> {reply.command}")
    print(f"  << {reply.response or '<no response>'}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Probe Dobot MovJ TCP command formats.")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--delay", type=float, default=0.3)
    parser.add_argument("--a", type=int, default=20)
    parser.add_argument("--v", type=int, default=20)
    parser.add_argument("--cp", type=int, default=0)
    parser.add_argument(
        "--pose", nargs=6, type=float, metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        help="Target pose in mm/deg. If omitted, --current uses GetPose().")
    parser.add_argument(
        "--current", action="store_true",
        help="Use current GetPose() as MovJ target. Safest way to test syntax.")
    parser.add_argument(
        "--execute", action="store_true",
        help="Actually send MovJ candidates. Without this, MovJ commands are only printed.")
    args = parser.parse_args()

    print(f"TCP target: {args.host}:{args.port}")
    print("Preflight commands:")
    preflight = {}
    for name, cmd in [
        ("robot_mode", "RobotMode()"),
        ("get_pose", "GetPose()"),
        ("get_angle", "GetAngle()"),
        ("get_error_id", "GetErrorID()"),
    ]:
        reply = send_command(args.host, args.port, cmd, args.timeout)
        preflight[name] = reply
        print_reply(name, reply)
        time.sleep(args.delay)

    pose = args.pose
    if pose is None and args.current:
        pose = parse_brace_numbers(preflight["get_pose"].response, 6)
        if pose is None:
            print("\nERROR: --current requested but GetPose() did not return 6 values.")
            return 2
    if pose is None:
        print("\nNo --pose/--current supplied. MovJ candidates will not be built.")
        print("Example:")
        print("  python3 scripts/test_movj_tcp_formats.py --host 192.168.1.6 --current --execute")
        return 0

    joints = parse_brace_numbers(preflight["get_angle"].response, 6)
    candidates = build_movj_variants(pose, joints, args.a, args.v, args.cp)

    print("\nMovJ candidates:")
    for name, cmd in candidates:
        print(f"  [{name}] {cmd}")

    if not args.execute:
        print("\nDry run only. Add --execute to send these MovJ commands.")
        return 0

    print("\nExecuting MovJ candidates. A response with ErrorID=0 means syntax/queue accepted.")
    print("If pose={...} gives -30001 but MovJ(x,y,z,rx,ry,rz) gives 0, this controller uses legacy positional format.")
    for name, cmd in candidates:
        reply = send_command(args.host, args.port, cmd, args.timeout)
        print_reply(name, reply)
        time.sleep(args.delay)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
