#!/usr/bin/env python3
"""Shift the demo paths so they start at the robot's current pose.

The demo paths in src/path_sender/data are absolute coordinates that start at
(0, 0) facing +x. On the H1 the odometry frame (FAST-LIO /Odometry) has its
origin wherever FAST-LIO started, so the raw files would send the robot to some
unrelated spot. This reads one /Odometry message and writes rotated/translated
copies to h1_paths/, plus the matching goal yaws. It sends no motion commands.

Usage (after `source ~/znav_env.sh`):
    python3 scripts/h1_paths.py [--scale 0.5] [--odom /Odometry]
"""
import argparse
import math
import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src", "path_sender", "data")
DST = os.path.join(ROOT, "h1_paths")
GOAL_YAWS = [1.57, 3.14, 0.0]  # same as path_sender.launch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--odom", default="/Odometry")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="shrink the paths, e.g. 0.5 -> about 3 m x 1 m")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("h1_paths_once")
    got = []
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Odometry, args.odom, got.append, qos)
    end = node.get_clock().now().nanoseconds + 15e9
    while not got and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        raise SystemExit(f"no message on {args.odom} within 15 s")

    p = got[0].pose.pose
    q = p.orientation
    yaw0 = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    x0, y0, z0 = p.position.x, p.position.y, p.position.z
    c, s = math.cos(yaw0), math.sin(yaw0)

    os.makedirs(DST, exist_ok=True)
    for i in (1, 2, 3):
        out = []
        with open(os.path.join(SRC, f"path_{i}.txt")) as f:
            for line in f:
                v = line.split()
                if len(v) < 3:
                    continue
                x, y = float(v[0]) * args.scale, float(v[1]) * args.scale
                out.append(f"{x0 + c * x - s * y:.4f} {y0 + s * x + c * y:.4f} {z0:.4f}\n")
        with open(os.path.join(DST, f"path_{i}.txt"), "w") as f:
            f.writelines(out)

    yaws = [math.atan2(math.sin(g + yaw0), math.cos(g + yaw0)) for g in GOAL_YAWS]
    print(f"robot pose: x={x0:.2f} y={y0:.2f} yaw={math.degrees(yaw0):.1f} deg")
    print(f"paths written to {DST} (scale {args.scale}); "
          f"robot will drive forward {6 * args.scale:.1f} m and left {2 * args.scale:.1f} m")
    with open(os.path.join(DST, "path_sender.yaml"), "w") as f:
        f.write("pathSender:\n  ros__parameters:\n")
        for i in (1, 2, 3):
            f.write(f"    pathFile{i}: {os.path.join(DST, f'path_{i}.txt')}\n")
            f.write(f"    goalYaw{i}: {yaws[i - 1]:.4f}\n")
        f.write("    goalDelay: 1.0\n    initDelay: 1.0\n    looping: false\n")
    print(f"pathSender params: {os.path.join(DST, 'path_sender.yaml')} (looping: false)")


if __name__ == "__main__":
    main()
