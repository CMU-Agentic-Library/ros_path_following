#!/usr/bin/env python3
"""One path: forward --dist m, U-turn (semicircle, radius --radius) to the
left or right, then back --dist m. Built from the robot's current /Odometry pose.

Without --send it only writes h1_paths_uturn/path.txt and prints a summary.
With --send it publishes the path once on --topic (pathFollower's path topic),
which makes the robot move if pathFollower and the chassis chain are running.

Usage (after `source ~/znav_env.sh`):
    python3 scripts/h1_uturn.py [--dist 1.0] [--radius 0.25] [--side left] [--send]
"""
import argparse
import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.qos import QoSProfile, ReliabilityPolicy

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DST = os.path.join(ROOT, "h1_paths_uturn")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--odom", default="/Odometry")
    ap.add_argument("--topic", default="/follow_path")
    ap.add_argument("--dist", type=float, default=1.0)
    ap.add_argument("--radius", type=float, default=0.25)
    ap.add_argument("--side", choices=("left", "right"), default="left")
    ap.add_argument("--step", type=float, default=0.05)
    ap.add_argument("--send", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("h1_uturn")
    got = []
    node.create_subscription(Odometry, args.odom, got.append,
                             QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
    clock = node.get_clock()
    end = clock.now().nanoseconds + 15e9
    while not got and clock.now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not got:
        raise SystemExit(f"no message on {args.odom} within 15 s")

    p = got[-1].pose.pose
    q = p.orientation
    yaw0 = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    x0, y0, z0 = p.position.x, p.position.y, p.position.z
    sgn = 1.0 if args.side == "left" else -1.0
    r, d, st = args.radius, args.dist, args.step

    # path in the robot frame (forward = +x, left = +y)
    local = []
    n = max(1, round(d / st))
    local += [(d * i / n, 0.0) for i in range(n + 1)]
    m = max(4, round(math.pi * r / st))
    for i in range(1, m + 1):
        a = -math.pi / 2 + math.pi * i / m
        local.append((d + r * math.cos(a), sgn * (r + r * math.sin(a))))
    local += [(d - d * i / n, sgn * 2 * r) for i in range(1, n + 1)]

    c, s = math.cos(yaw0), math.sin(yaw0)
    pts = [(x0 + c * u - s * v, y0 + s * u + c * v) for u, v in local]
    goal_yaw = math.atan2(math.sin(yaw0 + math.pi), math.cos(yaw0 + math.pi))

    os.makedirs(DST, exist_ok=True)
    with open(os.path.join(DST, "path.txt"), "w") as f:
        f.writelines(f"{x:.4f} {y:.4f} {z0:.4f}\n" for x, y in pts)
    print(f"robot pose: x={x0:.2f} y={y0:.2f} yaw={math.degrees(yaw0):.1f} deg")
    print(f"path: {len(pts)} points, forward {d} m, U-turn {args.side} r={r} m, back {d} m; "
          f"ends at ({pts[-1][0]:.2f},{pts[-1][1]:.2f}) facing {math.degrees(goal_yaw):.0f} deg")
    print(f"saved {os.path.join(DST, 'path.txt')}")

    if args.send:
        pub = node.create_publisher(Path, args.topic, 5)
        end = clock.now().nanoseconds + 10e9
        while pub.get_subscription_count() == 0 and clock.now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            raise SystemExit(f"nobody subscribes to {args.topic}; is pathFollower running?")
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = got[-1].header.stamp
        for x, y in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        msg.poses[-1].pose.orientation.z = math.sin(goal_yaw / 2)
        msg.poses[-1].pose.orientation.w = math.cos(goal_yaw / 2)
        for _ in range(5):  # let discovery settle so the one message is delivered
            rclpy.spin_once(node, timeout_sec=0.1)
        pub.publish(msg)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        print(f"sent path on {args.topic}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
