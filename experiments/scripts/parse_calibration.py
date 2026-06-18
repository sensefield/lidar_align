#!/usr/bin/env python3
"""Parse lidar_align calibration.txt into a JSON file.

Usage:
  parse_calibration.py <calibration.txt> [output.json]
  parse_calibration.py <calibration.txt> [output.json] [--log <stdout_log>]
"""
import re
import json
import sys
import pathlib
import argparse


def parse(calib_path: str, log_path: str | None = None) -> dict:
    text = pathlib.Path(calib_path).read_text()

    # Vector [x, y, z, rx, ry, rz]
    m = re.search(
        r"Transformation Vector \(x,y,z,rx,ry,rz\).*?\n\[([^\]]+)\]",
        text, re.DOTALL
    )
    if not m:
        raise ValueError(f"Cannot parse transformation vector from {calib_path}")
    vals = [float(v.strip()) for v in m.group(1).split(",")]

    # Quaternion [w, x, y, z]
    m_q = re.search(
        r"Quaternion \(w,x,y,z\).*?\n\[([^\]]+)\]",
        text, re.DOTALL
    )
    quat = [float(v.strip()) for v in m_q.group(1).split(",")] if m_q else [None] * 4

    # Translation vector (redundant with vals but handy for sanity check)
    m_t = re.search(
        r"Translation Vector \(x,y,z\).*?\n\[([^\]]+)\]",
        text, re.DOTALL
    )
    trans = [float(v.strip()) for v in m_t.group(1).split(",")] if m_t else None

    # Time offset
    m_tau = re.search(
        r"Time offset that must be added.*?\n([-\d.eE+]+)",
        text, re.DOTALL
    )
    time_offset = float(m_tau.group(1)) if m_tau else None

    # Final KNN cost from stdout log (last "Error:" value in the log)
    knn_cost = None
    if log_path and pathlib.Path(log_path).exists():
        log_text = pathlib.Path(log_path).read_text()
        costs = re.findall(r"Error:\s+([\d.eE+\-]+)", log_text)
        if costs:
            knn_cost = float(costs[-1])

    result = {
        "x":  vals[0], "y":  vals[1], "z":  vals[2],
        "rx": vals[3], "ry": vals[4], "rz": vals[5],
        "qw": quat[0], "qx": quat[1], "qy": quat[2], "qz": quat[3],
        "time_offset_s": time_offset,
        "knn_cost": knn_cost,
    }
    if trans:
        assert abs(result["x"] - trans[0]) < 1e-4, "Translation mismatch in calibration.txt"

    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument("calib_txt", help="Path to calibration.txt")
    p.add_argument("output_json", nargs="?", help="Output JSON path (default: stdout)")
    p.add_argument("--log", default=None, help="Path to stdout log for KNN cost extraction")
    args = p.parse_args()

    result = parse(args.calib_txt, args.log)

    if args.output_json:
        pathlib.Path(args.output_json).write_text(json.dumps(result, indent=2))
        print(f"Saved: {args.output_json}", file=sys.stderr)
    else:
        print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
