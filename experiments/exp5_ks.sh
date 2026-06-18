#!/usr/bin/env bash
# 実験 5 (KS): 初期値依存性テスト — kinematic_state pose ソース
# Usage: exp5_ks.sh <bag_path> [N=5]

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/scripts/lib_common_ks.sh"
ros_source

BAG_PATH="${1:?Usage: exp5_ks.sh <bag_path> [N]}"
N="${2:-5}"
OUT_DIR="${RESULTS_DIR}/exp5_initial_guess_ks"
mkdir -p "${OUT_DIR}"

if [[ ! -e "${BAG_PATH}" ]]; then
  echo "[ERROR] Bag not found: ${BAG_PATH}" >&2; exit 1
fi

log_header "実験 5 (KS): 初期値依存性テスト (N=${N})"
echo "Bag        : ${BAG_PATH}"
echo "PoseSource : /localization/kinematic_state (Odometry)"

python3 - "${OUT_DIR}" "${N}" "${BASE_CONFIG}" <<'PYEOF'
import random, json, yaml, pathlib, sys

out_dir = pathlib.Path(sys.argv[1])
N = int(sys.argv[2])
cfg_path = sys.argv[3]

random.seed(42)
cfg = yaml.safe_load(pathlib.Path(cfg_path).read_text())
base = cfg["lidar_align"]["ros__parameters"]["inital_guess"]
bx, by, bz, brx, bry, brz = base

guesses = []
for _ in range(N):
    g = [
        bx  + random.uniform(-0.3, 0.3),
        by  + random.uniform(-0.3, 0.3),
        bz  + random.uniform(-0.3, 0.3),
        brx + random.uniform(-0.1, 0.1),
        bry + random.uniform(-0.1, 0.1),
        brz + random.uniform(-0.1, 0.1),
    ]
    guesses.append(g)

out = out_dir / "initial_guesses.json"
out.write_text(json.dumps(guesses, indent=2))
print(f"Generated {len(guesses)} random initial guesses")
for i, g in enumerate(guesses, 1):
    print(f"  {i}: {[round(v,4) for v in g]}")
PYEOF

GUESSES_JSON="${OUT_DIR}/initial_guesses.json"

# ローカルのみ (local=true) で N 通り実行
python3 - "${BAG_PATH}" "${OUT_DIR}" "${GUESSES_JSON}" "${SCRIPT_DIR}" <<'PYEOF'
import json, subprocess, pathlib, sys

bag_path  = sys.argv[1]
out_dir   = sys.argv[2]
guesses_f = sys.argv[3]
script_dir = sys.argv[4]

guesses = json.loads(pathlib.Path(guesses_f).read_text())

for i, g in enumerate(guesses, 1):
    g_str = "[" + ", ".join(f"{v:.4f}" for v in g) + "]"
    for mode, local_flag in [("2stage", "false"), ("local_only", "true")]:
        run_id = f"{mode}_{i:02d}"
        cmd = [
            "bash", "-c",
            f'source "{script_dir}/scripts/lib_common_ks.sh" && ros_source && '
            f'run_calibration "{bag_path}" "{out_dir}" "{run_id}" '
            f'"use_n_scans:=500" '
            f'"local:={local_flag}" '
            f'"inital_guess:={g_str}"'
        ]
        print(f"[EXP5-KS] {run_id}: local={local_flag}, guess={g_str[:50]}...")
        result = subprocess.run(cmd, capture_output=False)
        if result.returncode != 0:
            print(f"[WARN] {run_id} failed")
PYEOF

python3 - "${OUT_DIR}" <<'PYEOF'
import json, statistics, pathlib, sys

out = pathlib.Path(sys.argv[1])
print("\n=== 収束統計 (KS) ===")
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
