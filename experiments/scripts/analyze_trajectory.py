#!/usr/bin/env python3
"""Analyze trajectory quality from a ROS2 bag.

Extracts PoseStamped or TF messages and computes:
  - Planarity score (PCA eigenvalue ratio λ3/λ1)
  - Cumulative rotation [deg]
  - Number of LiDAR scans
  - Total duration [s]

Usage:
  analyze_trajectory.py <bag_path> [output.json]
    [--pose-topic /sensing/gnss/pose]
    [--pc-topic /sensing/lidar/front/pointcloud_raw_ex]
    [--tf-parent map] [--tf-child base_link]
    [--use-tf]
"""
import argparse
import json
import math
import pathlib
import sys

import numpy as np

# ROS 2 imports (require sourced environment)
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2


def open_reader(bag_path: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_opts = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_opts, converter_opts)
    return reader


def quat_to_rotvec(qw, qx, qy, qz):
    """Convert quaternion to rotation vector."""
    norm = math.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm
    sin_half = math.sqrt(max(0.0, 1.0 - qw**2))
    if sin_half < 1e-10:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.acos(max(-1.0, min(1.0, qw)))
    return (qx / sin_half * angle, qy / sin_half * angle, qz / sin_half * angle)


def rotation_angle_between(q1, q2):
    """Rotation angle [rad] between two quaternions (w,x,y,z)."""
    # relative rotation: q_rel = q1^-1 * q2
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 + x1*x2 + y1*y2 + z1*z2
    w = max(-1.0, min(1.0, w))
    return 2.0 * math.acos(abs(w))  # abs because q == -q


def extract_poses_pose_stamped(bag_path: str, topic: str):
    """Return list of (timestamp_s, x, y, z, qw, qx, qy, qz)."""
    reader = open_reader(bag_path)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    if topic not in topic_types:
        print(f"[WARN] Topic '{topic}' not found in bag. Available: {list(topic_types.keys())}",
              file=sys.stderr)
        return []

    filter_topics = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(filter_topics)

    poses = []
    while reader.has_next():
        t, data, ts = reader.read_next()
        msg = deserialize_message(data, PoseStamped)
        p = msg.pose.position
        o = msg.pose.orientation
        poses.append((ts * 1e-9, p.x, p.y, p.z, o.w, o.x, o.y, o.z))

    return poses


def extract_poses_odometry(bag_path: str, topic: str):
    """Return list of (timestamp_s, x, y, z, qw, qx, qy, qz) from Odometry."""
    reader = open_reader(bag_path)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    if topic not in topic_types:
        print(f"[WARN] Topic '{topic}' not found in bag. Available: {list(topic_types.keys())}",
              file=sys.stderr)
        return []

    filter_topics = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(filter_topics)

    poses = []
    while reader.has_next():
        t, data, ts = reader.read_next()
        msg = deserialize_message(data, Odometry)
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        poses.append((stamp, p.x, p.y, p.z, o.w, o.x, o.y, o.z))

    return poses


def extract_poses_tf(bag_path: str, tf_topic: str, parent_frame: str, child_frame: str):
    """Return list of (timestamp_s, x, y, z, qw, qx, qy, qz) from TF."""
    reader = open_reader(bag_path)
    filter_topics = rosbag2_py.StorageFilter(topics=[tf_topic])
    reader.set_filter(filter_topics)

    poses = []
    while reader.has_next():
        t, data, ts = reader.read_next()
        msg = deserialize_message(data, TFMessage)
        for tf in msg.transforms:
            if tf.header.frame_id == parent_frame and tf.child_frame_id == child_frame:
                tr = tf.transform.translation
                ro = tf.transform.rotation
                stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                poses.append((stamp, tr.x, tr.y, tr.z, ro.w, ro.x, ro.y, ro.z))

    poses.sort(key=lambda x: x[0])
    return poses


def count_scans(bag_path: str, pc_topic: str) -> int:
    reader = open_reader(bag_path)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if pc_topic not in topic_types:
        return 0

    filter_topics = rosbag2_py.StorageFilter(topics=[pc_topic])
    reader.set_filter(filter_topics)

    count = 0
    while reader.has_next():
        reader.read_next()
        count += 1
    return count


def analyze(poses: list, n_scans: int) -> dict:
    if len(poses) < 3:
        return {"error": "Too few poses for analysis"}

    positions = np.array([[p[1], p[2], p[3]] for p in poses])
    quats     = [(p[4], p[5], p[6], p[7]) for p in poses]

    # Planarity score via PCA on positions
    centered = positions - positions.mean(axis=0)
    cov = centered.T @ centered / len(centered)
    eigenvalues = np.linalg.eigvalsh(cov)
    eigenvalues = np.sort(eigenvalues)[::-1]  # descending
    planarity_score = float(eigenvalues[2] / eigenvalues[0]) if eigenvalues[0] > 1e-10 else 0.0

    # Cumulative rotation
    total_rot_rad = 0.0
    for i in range(1, len(quats)):
        total_rot_rad += rotation_angle_between(quats[i-1], quats[i])

    duration_s = poses[-1][0] - poses[0][0]

    return {
        "n_poses":          len(poses),
        "n_scans":          n_scans,
        "duration_s":       round(duration_s, 2),
        "planarity_score":  round(planarity_score, 6),
        "eigenvalues":      [round(float(e), 4) for e in eigenvalues],
        "cumulative_rot_deg": round(math.degrees(total_rot_rad), 2),
        "path_length_m":    round(float(np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))), 2),
        "observation": (
            "SUFFICIENT (non-planar motion detected)"
            if planarity_score > 0.01
            else "INSUFFICIENT (planar motion — translation accuracy may be degraded)"
        ),
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("bag_path")
    p.add_argument("output_json", nargs="?")
    p.add_argument("--pose-topic", default="/sensing/gnss/pose")
    p.add_argument("--pc-topic",   default="/sensing/lidar/front/pointcloud_raw_ex")
    p.add_argument("--tf-parent",  default="map")
    p.add_argument("--tf-child",   default="base_link")
    p.add_argument("--use-tf",     action="store_true")
    p.add_argument("--use-odom",   action="store_true",
                   help="Read nav_msgs/msg/Odometry from --pose-topic instead of PoseStamped")
    args = p.parse_args()

    print(f"Reading bag: {args.bag_path}", file=sys.stderr)

    if args.use_tf:
        poses = extract_poses_tf(args.bag_path, "/tf", args.tf_parent, args.tf_child)
    elif args.use_odom:
        poses = extract_poses_odometry(args.bag_path, args.pose_topic)
    else:
        poses = extract_poses_pose_stamped(args.bag_path, args.pose_topic)

    if not poses:
        print("[ERROR] No poses extracted.", file=sys.stderr)
        sys.exit(1)

    n_scans = count_scans(args.bag_path, args.pc_topic)
    result = analyze(poses, n_scans)

    if args.output_json:
        pathlib.Path(args.output_json).write_text(json.dumps(result, indent=2))
        print(f"Saved: {args.output_json}", file=sys.stderr)
    else:
        print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
