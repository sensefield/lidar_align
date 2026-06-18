#!/usr/bin/env python3
"""Aggregate all experiment results and generate a Markdown report.

Usage:
  generate_report.py <results_dir> [output.md]
"""
import argparse
import json
import math
import pathlib
import statistics
import sys
from datetime import datetime


def load_jsons(directory: pathlib.Path, pattern: str) -> list[dict]:
    files = sorted(directory.glob(pattern))
    results = []
    for f in files:
        try:
            d = json.loads(f.read_text())
            d["_file"] = str(f.name)
            results.append(d)
        except Exception as e:
            print(f"[WARN] Cannot parse {f}: {e}", file=sys.stderr)
    return results


def fmt(v, digits=4):
    if v is None:
        return "—"
    return f"{v:.{digits}f}"


def stats_row(values: list[float]) -> str:
    if not values:
        return "—"
    mean = statistics.mean(values)
    std  = statistics.stdev(values) if len(values) > 1 else 0.0
    return f"{mean:.4f} ± {std:.4f}"


def section_exp1(results_dir: pathlib.Path) -> str:
    d = results_dir / "exp1_repeatability"
    calibs = load_jsons(d, "calibration_*.json")

    lines = ["## 実験 1: 再現性テスト (N 回繰り返し)\n"]
    if not calibs:
        lines.append("_結果なし_\n")
        return "\n".join(lines)

    lines.append("| Run | x [m] | y [m] | z [m] | rx [rad] | ry [rad] | rz [rad] | τ [ms] |")
    lines.append("|-----|-------|-------|-------|----------|----------|----------|--------|")
    for c in calibs:
        tau_ms = f"{c['time_offset_s']*1000:.2f}" if c.get('time_offset_s') is not None else "—"
        lines.append(
            f"| {c['_file']} | {fmt(c['x'])} | {fmt(c['y'])} | {fmt(c['z'])} "
            f"| {fmt(c['rx'])} | {fmt(c['ry'])} | {fmt(c['rz'])} | {tau_ms} |"
        )

    if len(calibs) > 1:
        lines.append("")
        lines.append("**標本標準偏差:**\n")
        lines.append("| σ_x [m] | σ_y [m] | σ_z [m] | σ_rx [rad] | σ_ry [rad] | σ_rz [rad] | σ_τ [ms] |")
        lines.append("|---------|---------|---------|------------|------------|------------|----------|")
        stds = []
        for key in ["x", "y", "z", "rx", "ry", "rz"]:
            vals = [c[key] for c in calibs if c.get(key) is not None]
            stds.append(f"{statistics.stdev(vals):.5f}" if len(vals) > 1 else "—")
        tau_vals = [c["time_offset_s"]*1000 for c in calibs if c.get("time_offset_s") is not None]
        std_tau = f"{statistics.stdev(tau_vals):.3f}" if len(tau_vals) > 1 else "—"
        lines.append("| " + " | ".join(stds) + f" | {std_tau} |")

    return "\n".join(lines) + "\n"


def section_exp2(results_dir: pathlib.Path) -> str:
    d = results_dir / "exp2_perturbation"
    errors = load_jsons(d, "error_*.json")
    params = load_jsons(d, "params_*.json")

    lines = ["## 実験 2: オフセット注入テスト\n"]
    if not errors:
        lines.append("_結果なし_\n")
        return "\n".join(lines)

    lines.append("| ケース | 注入 Δx [m] | 注入 Δrz [rad] | 注入 Δτ [ms] | 回収 e_t [m] | 回収 e_r [°] |")
    lines.append("|--------|------------|----------------|--------------|-------------|-------------|")
    for e, par in zip(errors, params if params else [{}] * len(errors)):
        lines.append(
            f"| {e['_file']} "
            f"| {fmt(par.get('delta_x', None))} "
            f"| {fmt(par.get('delta_rz', None))} "
            f"| {fmt(par.get('delta_tau_ms', None))} "
            f"| {fmt(e.get('e_t_m'))} "
            f"| {fmt(e.get('e_r_deg'))} |"
        )

    return "\n".join(lines) + "\n"


def section_exp3(results_dir: pathlib.Path) -> str:
    d = results_dir / "exp3_sensitivity"
    calibs = load_jsons(d, "calibration_*.json")
    params = load_jsons(d, "params_*.json")

    lines = ["## 実験 3: パラメータ感度分析\n"]
    if not calibs:
        lines.append("_結果なし_\n")
        return "\n".join(lines)

    lines.append("| パラメータ | 値 | x [m] | y [m] | z [m] | rz [rad] | τ [ms] | KNNコスト | 実行時間 [s] |")
    lines.append("|-----------|---|-------|-------|-------|----------|--------|-----------|------------|")
    for c, par in zip(calibs, params if params else [{}] * len(calibs)):
        param_name = par.get("param_name", "—")
        param_val  = par.get("param_value", "—")
        tau_ms = f"{c['time_offset_s']*1000:.2f}" if c.get('time_offset_s') is not None else "—"
        elapsed = f"{par.get('elapsed_s', '—')}"
        lines.append(
            f"| {param_name} | {param_val} "
            f"| {fmt(c['x'])} | {fmt(c['y'])} | {fmt(c['z'])} "
            f"| {fmt(c['rz'])} | {tau_ms} | {fmt(c.get('knn_cost'), 2)} | {elapsed} |"
        )

    return "\n".join(lines) + "\n"


def section_exp4(results_dir: pathlib.Path) -> str:
    d = results_dir / "exp4_motion"
    trajs = load_jsons(d, "trajectory_*.json")
    calibs = load_jsons(d, "calibration_*.json")

    lines = ["## 実験 4: 運動パターン依存性\n"]
    if not trajs:
        lines.append("_結果なし_\n")
        return "\n".join(lines)

    lines.append("**軌跡品質:**\n")
    lines.append("| 区間 | スキャン数 | 時間 [s] | 非平面性スコア | 累積回転 [°] | 判定 |")
    lines.append("|------|----------|---------|--------------|------------|------|")
    for t in trajs:
        lines.append(
            f"| {t['_file']} | {t.get('n_scans', '—')} "
            f"| {t.get('duration_s', '—')} "
            f"| {fmt(t.get('planarity_score'), 6)} "
            f"| {fmt(t.get('cumulative_rot_deg'), 1)} "
            f"| {t.get('observation', '—')} |"
        )

    if calibs:
        lines.append("")
        lines.append("**各区間のキャリブレーション結果:**\n")
        lines.append("| 区間 | x [m] | y [m] | z [m] | rz [rad] | KNNコスト |")
        lines.append("|------|-------|-------|-------|----------|-----------|")
        for c in calibs:
            lines.append(
                f"| {c['_file']} | {fmt(c['x'])} | {fmt(c['y'])} | {fmt(c['z'])} "
                f"| {fmt(c['rz'])} | {fmt(c.get('knn_cost'), 2)} |"
            )

    return "\n".join(lines) + "\n"


def section_exp5(results_dir: pathlib.Path) -> str:
    d = results_dir / "exp5_initial_guess"
    calibs_2stage  = load_jsons(d, "calibration_2stage_*.json")
    calibs_local   = load_jsons(d, "calibration_local_*.json")

    lines = ["## 実験 5: 初期値依存性テスト\n"]
    if not calibs_2stage and not calibs_local:
        lines.append("_結果なし_\n")
        return "\n".join(lines)

    def summary(calibs, label):
        if not calibs:
            return f"_{label}: 結果なし_\n"
        rows = [f"**{label}**\n"]
        rows.append("| Run | x [m] | y [m] | z [m] | rz [rad] | KNNコスト |")
        rows.append("|-----|-------|-------|-------|----------|-----------|")
        for c in calibs:
            rows.append(
                f"| {c['_file']} | {fmt(c['x'])} | {fmt(c['y'])} | {fmt(c['z'])} "
                f"| {fmt(c['rz'])} | {fmt(c.get('knn_cost'), 2)} |"
            )
        if len(calibs) > 1:
            rows.append("")
            rows.append("標準偏差:")
            stds = []
            for key in ["x", "y", "z", "rz"]:
                vals = [c.get(key) for c in calibs if c.get(key) is not None]
                stds.append(f"σ_{key} = {statistics.stdev(vals):.5f}" if len(vals) > 1 else f"σ_{key} = —")
            rows.append(", ".join(stds))
        return "\n".join(rows) + "\n"

    lines.append(summary(calibs_2stage, "2段階最適化 (local=false)"))
    lines.append(summary(calibs_local,  "ローカルのみ (local=true)"))

    return "\n".join(lines) + "\n"


def generate(results_dir: str) -> str:
    rd = pathlib.Path(results_dir)
    now = datetime.now().strftime("%Y-%m-%d %H:%M")

    sections = [
        f"# lidar_align 精度評価レポート\n\n生成日時: {now}\n",
        section_exp1(rd),
        section_exp2(rd),
        section_exp3(rd),
        section_exp4(rd),
        section_exp5(rd),
    ]
    return "\n---\n\n".join(sections)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("results_dir")
    p.add_argument("output_md", nargs="?")
    args = p.parse_args()

    report = generate(args.results_dir)

    if args.output_md:
        pathlib.Path(args.output_md).write_text(report)
        print(f"Saved: {args.output_md}", file=sys.stderr)
    else:
        print(report)


if __name__ == "__main__":
    main()
