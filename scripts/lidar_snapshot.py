#!/usr/bin/env python3
"""Save a picture of the point cloud around the robot.

Accumulates a few seconds of point cloud, grabs the robot pose from /Odometry, and writes a top-down view (colored by height) and
a side view, with the robot and a path file (--path) overlaid. Sends nothing.

Usage (after `source ~/znav_env.sh`):
    python3 scripts/lidar_snapshot.py [--secs 3] [--range 8] [--out lidar.png]

Default source is the raw lidar /livox/lidar (sensor frame), moved into the
odom frame with the latest /Odometry pose. Use
`--cloud /cloud_registered --world` if FAST-LIO publishes its odom-frame cloud.
"""
import argparse
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cloud", default="/livox/lidar")
    ap.add_argument("--world", action="store_true",
                    help="cloud is already in the odom frame (e.g. /cloud_registered)")
    ap.add_argument("--odom", default="/Odometry")
    ap.add_argument("--secs", type=float, default=3.0)
    ap.add_argument("--range", type=float, default=8.0, help="half-width of view [m]")
    ap.add_argument("--zmin", type=float, default=-1.5, help="drop points below robot z + zmin")
    ap.add_argument("--zmax", type=float, default=2.0, help="drop points above robot z + zmax")
    ap.add_argument("--path", default=os.path.join(ROOT, "h1_paths_uturn", "path.txt"),
                    help="path file (x y z per line) to overlay; empty to skip")
    ap.add_argument("--out", default=os.path.join(ROOT, "runs", "lidar.png"))
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("lidar_snapshot")
    clouds, odom = [], []
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

    def on_cloud(msg):
        rec = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = np.stack([rec["x"], rec["y"], rec["z"]], axis=-1).astype(np.float64).reshape(-1, 3)
        if not args.world:
            if not odom:
                return
            pp = odom[-1].pose.pose
            pts = pts @ quat_to_rot(pp.orientation).T + [pp.position.x, pp.position.y, pp.position.z]
        clouds.append(pts.astype(np.float32))

    node.create_subscription(PointCloud2, args.cloud, on_cloud, qos)
    node.create_subscription(Odometry, args.odom, odom.append, qos)
    clock = node.get_clock()
    end = clock.now().nanoseconds + args.secs * 1e9
    hard_end = end + 10e9
    while clock.now().nanoseconds < end or ((not clouds or not odom) and clock.now().nanoseconds < hard_end):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not clouds:
        raise SystemExit(f"no message on {args.cloud}")
    if not odom:
        raise SystemExit(f"no message on {args.odom}")

    p = odom[-1].pose.pose
    q = p.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    rx, ry, rz = p.position.x, p.position.y, p.position.z

    pts = np.concatenate(clouds)
    m = ((np.abs(pts[:, 0] - rx) < args.range) & (np.abs(pts[:, 1] - ry) < args.range)
         & (pts[:, 2] > rz + args.zmin) & (pts[:, 2] < rz + args.zmax))
    pts = pts[m]
    print(f"{len(clouds)} frames, {len(pts)} points in view; robot x={rx:.2f} y={ry:.2f} "
          f"yaw={math.degrees(yaw):.1f} deg")

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(15, 7), gridspec_kw={"width_ratios": [1, 1]})
    sc = ax.scatter(pts[:, 0], pts[:, 1], c=pts[:, 2] - rz, s=0.3, cmap="viridis",
                    vmin=args.zmin, vmax=args.zmax)
    fig.colorbar(sc, ax=ax, label="height above robot origin [m]", shrink=0.8)

    if args.path and os.path.exists(args.path):
        xy = np.loadtxt(args.path, usecols=(0, 1), ndmin=2)
        ax.plot(xy[:, 0], xy[:, 1], "--", color="tab:red", lw=1.5, label="path")

    ax.plot(rx, ry, "o", color="red", ms=9, label="robot")
    ax.arrow(rx, ry, 0.6 * math.cos(yaw), 0.6 * math.sin(yaw), color="red", width=0.04)
    for r in (1, 2, 4):
        ax.add_patch(plt.Circle((rx, ry), r, fill=False, ls=":", color="gray", lw=0.8))
        ax.text(rx + r * 0.71, ry + r * 0.71, f"{r} m", color="gray", fontsize=8)
    ax.set_xlim(rx - args.range, rx + args.range)
    ax.set_ylim(ry - args.range, ry + args.range)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m] (odom frame)")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Top view: {args.cloud}, {args.secs:.0f} s")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)

    # side view along the robot's heading: forward distance vs height
    c, s = math.cos(yaw), math.sin(yaw)
    fwd = (pts[:, 0] - rx) * c + (pts[:, 1] - ry) * s
    left = -(pts[:, 0] - rx) * s + (pts[:, 1] - ry) * c
    band = np.abs(left) < 1.0
    ax2.scatter(fwd[band], pts[band, 2] - rz, s=0.5, c="k")
    ax2.plot(0, 0, "o", color="red", ms=9)
    ax2.set_xlim(-args.range, args.range)
    ax2.set_ylim(args.zmin, args.zmax)
    ax2.set_xlabel("forward from robot [m] (points within ±1 m to the side)")
    ax2.set_ylabel("height [m]")
    ax2.set_title("Side view along robot heading")
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fig.savefig(args.out, dpi=120)
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
