#!/usr/bin/env python3
"""Apply a rigid-body offset to PoseStamped messages in a ROS2 bag.

Purpose: Create a "perturbed" bag to test the optimizer's recovery ability.
Each pose P_i is replaced by P_i composed with delta_T (right-multiplication):
  P_i' = P_i * delta_T

This simulates a change in the LiDAR-to-odom calibration by delta_T.
The optimizer should then recover a result that differs from the original by delta_T.

Usage:
  inject_offset_pose.py <input_bag> <output_bag>
    --dx DX --dy DY --dz DZ
    --drx DRX --dry DRY --drz DRZ
    [--pose-topic /sensing/gnss/pose]

delta rotation: rotation vector (axis * angle) in radians
"""
import argparse
import math
import pathlib
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from geometry_msgs.msg import PoseStamped


def rotvec_to_quat(rx: float, ry: float, rz: float):
    """Convert rotation vector to quaternion (w, x, y, z)."""
    theta = math.sqrt(rx**2 + ry**2 + rz**2)
    if theta < 1e-10:
        return (1.0, 0.0, 0.0, 0.0)
    half = theta / 2.0
    s = math.sin(half) / theta
    return (math.cos(half), rx * s, ry * s, rz * s)


def quat_multiply(q1, q2):
    """Hamilton product q1 * q2 (w, x, y, z)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )


def quat_rotate_vector(q, v):
    """Rotate 3-vector v by quaternion q (w, x, y, z)."""
    w, x, y, z = q
    # Rotation matrix from quaternion
    R = np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),       2*(x*z + w*y)      ],
        [    2*(x*y + w*z),   1 - 2*(x*x + z*z),   2*(y*z - w*x)      ],
        [    2*(x*z - w*y),       2*(y*z + w*x),   1 - 2*(x*x + y*y)  ],
    ])
    return R @ np.array(v)


def apply_delta_to_pose(msg: PoseStamped, delta_t: tuple, delta_q: tuple) -> PoseStamped:
    """
    Right-multiply the pose by delta: P' = P * delta.
    delta_t: (dx, dy, dz) in local (body) frame
    delta_q: (w, x, y, z) delta rotation
    """
    p = msg.pose.position
    o = msg.pose.orientation
    q_orig = (o.w, o.x, o.y, o.z)

    # New rotation: q' = q_orig * delta_q
    q_new = quat_multiply(q_orig, delta_q)

    # New translation: t' = t_orig + R(q_orig) * delta_t
    delta_t_world = quat_rotate_vector(q_orig, delta_t)
    new_x = p.x + delta_t_world[0]
    new_y = p.y + delta_t_world[1]
    new_z = p.z + delta_t_world[2]

    new_msg = PoseStamped()
    new_msg.header = msg.header
    new_msg.pose.position.x = new_x
    new_msg.pose.position.y = new_y
    new_msg.pose.position.z = new_z
    # Normalize quaternion
    norm = math.sqrt(sum(v**2 for v in q_new))
    new_msg.pose.orientation.w = q_new[0] / norm
    new_msg.pose.orientation.x = q_new[1] / norm
    new_msg.pose.orientation.y = q_new[2] / norm
    new_msg.pose.orientation.z = q_new[3] / norm

    return new_msg


def inject(input_bag: str, output_bag: str, pose_topic: str,
           dx: float, dy: float, dz: float,
           drx: float, dry: float, drz: float):

    delta_q = rotvec_to_quat(drx, dry, drz)
    delta_t = (dx, dy, dz)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    topic_types = reader.get_all_topics_and_types()
    pose_type = None
    for t in topic_types:
        if t.name == pose_topic:
            pose_type = t.type

    if pose_type is None:
        print(f"[WARN] '{pose_topic}' not found. Topics: {[t.name for t in topic_types]}",
              file=sys.stderr)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=output_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for t in topic_types:
        writer.create_topic(t)

    n_modified = 0
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == pose_topic and pose_type == "geometry_msgs/msg/PoseStamped":
            msg = deserialize_message(data, PoseStamped)
            msg = apply_delta_to_pose(msg, delta_t, delta_q)
            data = serialize_message(msg)
            n_modified += 1
        writer.write(topic, data, timestamp)

    print(f"Modified {n_modified} PoseStamped messages on '{pose_topic}'.", file=sys.stderr)
    print(f"Output: {output_bag}", file=sys.stderr)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("input_bag")
    p.add_argument("output_bag")
    p.add_argument("--dx",  type=float, default=0.0)
    p.add_argument("--dy",  type=float, default=0.0)
    p.add_argument("--dz",  type=float, default=0.0)
    p.add_argument("--drx", type=float, default=0.0)
    p.add_argument("--dry", type=float, default=0.0)
    p.add_argument("--drz", type=float, default=0.0)
    p.add_argument("--pose-topic", default="/sensing/gnss/pose")
    args = p.parse_args()

    print(
        f"Injecting offset: dx={args.dx} dy={args.dy} dz={args.dz} "
        f"drx={args.drx} dry={args.dry} drz={args.drz}",
        file=sys.stderr
    )
    inject(
        args.input_bag, args.output_bag, args.pose_topic,
        args.dx, args.dy, args.dz,
        args.drx, args.dry, args.drz,
    )


if __name__ == "__main__":
    main()
