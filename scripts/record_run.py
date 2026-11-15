#!/usr/bin/env python3
"""Record a path-following run and plot planned paths vs. the actual track.

Subscribes to odometry (FAST-LIO /Odometry) and the path topic the follower
uses (/follow_path), and logs /goal_reach events. Every few seconds, and again
on Ctrl-C/SIGTERM, it writes:
    <out>/track.csv   t, x, y, yaw of the robot
    <out>/paths.csv   path_id, x, y of every path received
    <out>/run.png     planned paths (dashed) + actual track (solid)
It sends no commands.

Usage (after `source ~/znav_env.sh`):
    python3 scripts/record_run.py [--out runs/<timestamp>]
"""
import argparse
import csv
import math
import os
import signal
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import rclpy  # noqa: E402
from nav_msgs.msg import Odometry, Path  # noqa: E402
from std_msgs.msg import Empty  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Recorder:
    def __init__(self, node, out):
        self.out = out
        self.track = []      # (t, x, y, yaw)
        self.paths = []      # list of [(x, y), ...]
        self.goals = []      # (x, y) where /goal_reach fired
        self.t0 = None
        node.create_subscription(Odometry, args.odom, self.on_odom, 10)
        node.create_subscription(Path, args.path, self.on_path, 10)
        node.create_subscription(Empty, "/goal_reach", self.on_goal, 10)

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        p = m.pose.pose
        self.track.append((t - self.t0, p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_path(self, m):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in m.poses]
        if pts and (not self.paths or pts != self.paths[-1]):
            self.paths.append(pts)

    def on_goal(self, _):
        if self.track:
            self.goals.append(self.track[-1][1:3])

    def save(self):
        os.makedirs(self.out, exist_ok=True)
        with open(os.path.join(self.out, "track.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "yaw"])
            w.writerows(self.track)
        with open(os.path.join(self.out, "paths.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["path_id", "x", "y"])
            for i, pts in enumerate(self.paths, 1):
                w.writerows((i, x, y) for x, y in pts)

        fig, ax = plt.subplots(figsize=(8, 6), dpi=150)
        colors = ["#1f77b4", "#2ca02c", "#9467bd", "#8c564b"]
        for i, pts in enumerate(self.paths):
            xs, ys = zip(*pts)
            ax.plot(xs, ys, "--", color=colors[i % len(colors)], lw=1.5,
                    label=f"planned path {i + 1}")
        if self.track:
            _, xs, ys, yaws = zip(*self.track)
            ax.plot(xs, ys, "-", color="#d62728", lw=2, label="actual track")
            ax.plot(xs[0], ys[0], "o", color="black", label="start")
            ax.plot(xs[-1], ys[-1], "s", color="#d62728", label="end")
            step = max(1, len(xs) // 25)
            ax.quiver(xs[::step], ys[::step],
                      [math.cos(a) for a in yaws[::step]],
                      [math.sin(a) for a in yaws[::step]],
                      color="#d62728", alpha=0.5, scale=30, width=0.003)
        for gx, gy in self.goals:
            ax.plot(gx, gy, "*", color="gold", ms=14, mec="black")
        dur = self.track[-1][0] if self.track else 0.0
        ax.set_title(f"H1 path following  ({len(self.goals)} goals reached, {dur:.1f} s)")
        ax.set_xlabel("x [m] (odom frame)")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.3)
        ax.legend(loc="best", fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(self.out, "run.png"))
        plt.close(fig)


ap = argparse.ArgumentParser()
ap.add_argument("--out", default=os.path.join(ROOT, "runs", time.strftime("%Y%m%d_%H%M%S")))
ap.add_argument("--odom", default="/Odometry")
ap.add_argument("--path", default="/follow_path")
args = ap.parse_args()

rclpy.init()
node = rclpy.create_node("run_recorder")
rec = Recorder(node, args.out)
stop = []
signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
signal.signal(signal.SIGINT, lambda *_: stop.append(1))
print(f"recording to {args.out}", flush=True)
last = time.time()
while not stop:
    rclpy.spin_once(node, timeout_sec=0.1)
    if time.time() - last > 5:
        rec.save()
        last = time.time()
rec.save()
print(f"saved {len(rec.track)} poses, {len(rec.paths)} paths -> {args.out}/run.png", flush=True)
node.destroy_node()
rclpy.shutdown()
