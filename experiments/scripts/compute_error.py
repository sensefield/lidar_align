#!/usr/bin/env python3
"""Compute calibration error metrics between estimated and ground-truth transforms.

Usage:
  compute_error.py <estimated.json> <ground_truth.json> [output.json]

Outputs e_t [m], e_r [deg], e_tau [ms].
"""
import json
import sys
import math
import pathlib
import argparse

import numpy as np


def rotvec_to_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
    """Convert rotation vector (axis*angle) to 3x3 rotation matrix."""
    theta = math.sqrt(rx**2 + ry**2 + rz**2)
    if theta < 1e-10:
        return np.eye(3)
    k = np.array([rx, ry, rz]) / theta
    K = np.array([
        [    0, -k[2],  k[1]],
        [ k[2],     0, -k[0]],
        [-k[1],  k[0],     0],
    ])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


def to_se3(d: dict) -> np.ndarray:
    """Build 4x4 SE(3) matrix from dict with x,y,z,rx,ry,rz."""
    T = np.eye(4)
    T[:3, :3] = rotvec_to_matrix(d["rx"], d["ry"], d["rz"])
    T[:3,  3] = [d["x"], d["y"], d["z"]]
    return T


def rotation_angle_deg(R: np.ndarray) -> float:
    """Extract rotation angle [deg] from 3x3 rotation matrix."""
    trace = float(np.trace(R))
    cos_a = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.degrees(math.acos(cos_a))


def compute_error(est: dict, gt: dict) -> dict:
    T_est = to_se3(est)
    T_gt  = to_se3(gt)

    # Residual: delta = T_GT^-1 * T_est
    T_delta = np.linalg.inv(T_gt) @ T_est

    e_t = float(np.linalg.norm(T_delta[:3, 3]))
    e_r = rotation_angle_deg(T_delta[:3, :3])

    e_tau = None
    if est.get("time_offset_s") is not None and gt.get("time_offset_s") is not None:
        e_tau = abs(float(est["time_offset_s"]) - float(gt["time_offset_s"])) * 1000.0

    return {
        "e_t_m":   round(e_t, 6),
        "e_r_deg": round(e_r, 4),
        "e_tau_ms": round(e_tau, 3) if e_tau is not None else None,
        "delta_x":  round(float(T_delta[0, 3]), 6),
        "delta_y":  round(float(T_delta[1, 3]), 6),
        "delta_z":  round(float(T_delta[2, 3]), 6),
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("estimated_json")
    p.add_argument("ground_truth_json")
    p.add_argument("output_json", nargs="?")
    args = p.parse_args()

    est = json.loads(pathlib.Path(args.estimated_json).read_text())
    gt  = json.loads(pathlib.Path(args.ground_truth_json).read_text())

    result = compute_error(est, gt)

    if args.output_json:
        pathlib.Path(args.output_json).write_text(json.dumps(result, indent=2))
        print(f"Saved: {args.output_json}", file=sys.stderr)
    else:
        print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
