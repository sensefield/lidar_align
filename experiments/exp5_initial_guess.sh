#!/usr/bin/env bash
# 実験 5: 初期値依存性テスト
# Usage: exp5_initial_guess.sh <bag_path> [N=20]

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/scripts/lib_common.sh"
ros_source

BAG_PATH="${1:?Usage: exp5_initial_guess.sh <bag_path> [N]}"
N="${2:-20}"
OUT_DIR="${RESULTS_DIR}/exp5_initial_guess"
mkdir -p "${OUT_DIR}"

if [[ ! -e "${BAG_PATH}" ]]; then
  echo "[ERROR] Bag not found: ${BAG_PATH}" >&2; exit 1
fi

log_header "実験 5: 初期値依存性テスト (N=${N})"
echo "Bag: ${BAG_PATH}"

# ランダム初期値を N 個生成
python3 - <<PYEOF
import random, json, yaml, pathlib

random.seed(42)
cfg = yaml.safe_load(pathlib.Path("${BASE_CONFIG}").read_text())
base = cfg["lidar_align"]["ros__parameters"]["inital_guess"]
bx, by, bz, brx, bry, brz = base

guesses = []
for _ in range(${N}):
    g = [
        bx  + random.uniform(-0.3, 0.3),
        by  + random.uniform(-0.3, 0.3),
        bz  + random.uniform(-0.3, 0.3),
        brx + random.uniform(-0.1, 0.1),
        bry + random.uniform(-0.1, 0.1),
        brz + random.uniform(-0.1, 0.1),
    ]
    guesses.append(g)

out = pathlib.Path("${OUT_DIR}/initial_guesses.json")
out.write_text(json.dumps(guesses, indent=2))
print(f"Generated {len(guesses)} random initial guesses")
PYEOF

# 各初期値で 2段階 と ローカルのみ を実行
python3 - <<'PYEOF'
import json, subprocess, pathlib, shlex

guesses = json.loads(pathlib.Path("${OUT_DIR}/initial_guesses.json").read_text())

for i, g in enumerate(guesses, 1):
    g_str = "[" + ", ".join(f"{v:.4f}" for v in g) + "]"
    for mode, local_flag in [("2stage", "false"), ("local_only", "true")]:
        run_id = f"{mode}_{i:02d}"
        # --ros-args で inital_guess 配列を渡す
        cmd = [
            "bash", "-c",
            f'source "${SCRIPT_DIR}/scripts/lib_common.sh" && ros_source && '
            f'run_calibration "{BAG_PATH}" "{OUT_DIR}" "{run_id}" '
            f'"local:={local_flag}" '
            f'"inital_guess:={g_str}"'
        ]
        print(f"[EXP7] {run_id}: local={local_flag}, guess={g_str[:40]}...")
        result = subprocess.run(cmd, capture_output=False)
        if result.returncode != 0:
            print(f"[WARN] {run_id} failed")
PYEOF

# 収束先の統計比較
python3 - <<'PYEOF'
import json, statistics, pathlib

out = pathlib.Path("${OUT_DIR}")
print("\n=== 収束統計 ===")
for mode in ["2stage", "local_only"]:
    calibs = sorted(out.glob(f"calibration_{mode}_*.json"))
    data = [json.loads(f.read_text()) for f in calibs]
    if not data:
        print(f"\n[{mode}]: 結果なし"); continue
    print(f"\n[{mode}] (N={len(data)})")
    print(f"  {'param':<6} {'mean':>10} {'σ':>10}")
    for k in ["x", "y", "z", "rx", "ry", "rz"]:
        vals = [d[k] for d in data if d.get(k) is not None]
        if vals and len(vals) > 1:
            print(f"  {k:<4} {statistics.mean(vals):>10.5f} {statistics.stdev(vals):>10.5f}")
PYEOF

echo ""
echo "Results: ${OUT_DIR}"
