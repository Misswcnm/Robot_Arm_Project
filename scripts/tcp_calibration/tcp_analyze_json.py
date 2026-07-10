#!/usr/bin/env python3
"""Analyze an existing TCP calibration JSON and print residual diagnostics."""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path

import numpy as np


def rot_zyx_deg(rx: float, ry: float, rz: float) -> np.ndarray:
    ax, ay, az = math.radians(rx), math.radians(ry), math.radians(rz)
    cx, sx = math.cos(ax), math.sin(ax)
    cy, sy = math.cos(ay), math.sin(ay)
    cz, sz = math.cos(az), math.sin(az)
    rx_m = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    ry_m = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rz_m = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    return rz_m @ ry_m @ rx_m


def solve(samples):
    rows = []
    rhs = []
    eye = np.eye(3)
    for s in samples:
        r = rot_zyx_deg(s["rx"], s["ry"], s["rz"])
        t = np.array([s["x"], s["y"], s["z"]], dtype=float)
        rows.append(np.hstack([r, -eye]))
        rhs.append(-t)
    a = np.vstack(rows)
    b = np.concatenate(rhs)
    x, *_ = np.linalg.lstsq(a, b, rcond=None)
    tcp = x[:3]
    fixed = x[3:]
    residuals = []
    tips = []
    for s in samples:
        r = rot_zyx_deg(s["rx"], s["ry"], s["rz"])
        t = np.array([s["x"], s["y"], s["z"]], dtype=float)
        tip = r @ tcp + t
        tips.append(tip)
        residuals.append(float(np.linalg.norm(tip - fixed)))
    return tcp, fixed, np.array(residuals), np.array(tips)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("json_path")
    parser.add_argument("--drop", nargs="*", type=int, default=[], help="1-based sample ids to drop")
    args = parser.parse_args()

    data = json.loads(Path(args.json_path).read_text(encoding="utf-8"))
    samples_all = data["samples"]
    drop = {i - 1 for i in args.drop}
    samples = [s for i, s in enumerate(samples_all) if i not in drop]

    tcp, fixed, residuals, _ = solve(samples)
    print(f"samples used: {len(samples)} / {len(samples_all)}  dropped: {args.drop}")
    print("tcp_offset_tool_mm = [%.3f, %.3f, %.3f]" % tuple(tcp))
    print("fixed_point_base_mm = [%.3f, %.3f, %.3f]" % tuple(fixed))
    print(
        "residual mean/max/std = %.3f / %.3f / %.3f mm"
        % (float(np.mean(residuals)), float(np.max(residuals)), float(np.std(residuals)))
    )
    print("")
    print("per-sample residual:")
    used_index = [i for i in range(len(samples_all)) if i not in drop]
    for source_i, s, r in zip(used_index, samples, residuals):
        print(
            "%02d  res=%6.3f mm  xyz=[%8.3f %8.3f %8.3f]  rpy=[%8.3f %8.3f %8.3f]"
            % (source_i + 1, r, s["x"], s["y"], s["z"], s["rx"], s["ry"], s["rz"])
        )

    if len(samples_all) >= 5 and not args.drop:
        print("")
        print("best leave-one-out:")
        loo = []
        for k in range(len(samples_all)):
            ss = [s for i, s in enumerate(samples_all) if i != k]
            tcp2, _, res2, _ = solve(ss)
            loo.append((float(np.mean(res2)), float(np.max(res2)), k + 1, tcp2))
        for mean, maxr, idx, tcp2 in sorted(loo)[:5]:
            print(
                "drop %02d: mean=%.3f max=%.3f tcp=[%.3f %.3f %.3f]"
                % (idx, mean, maxr, tcp2[0], tcp2[1], tcp2[2])
            )

        print("")
        print("best drop-two:")
        pairs = []
        for ks in itertools.combinations(range(len(samples_all)), 2):
            ss = [s for i, s in enumerate(samples_all) if i not in ks]
            tcp2, _, res2, _ = solve(ss)
            pairs.append((float(np.mean(res2)), float(np.max(res2)), tuple(k + 1 for k in ks), tcp2))
        for mean, maxr, ids, tcp2 in sorted(pairs)[:5]:
            print(
                "drop %s: mean=%.3f max=%.3f tcp=[%.3f %.3f %.3f]"
                % (ids, mean, maxr, tcp2[0], tcp2[1], tcp2[2])
            )


if __name__ == "__main__":
    main()
