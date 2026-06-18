#!/usr/bin/env bash
# 実験 4 (KS): 運動パターン依存性 — kinematic_state pose ソース
# Usage: exp4_ks.sh <bag_path>

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/scripts/lib_common_ks.sh"
ros_source

BAG_PATH="${1:?Usage: exp4_ks.sh <bag_path>}"
OUT_DIR="${RESULTS_DIR}/exp4_motion_ks"
mkdir -p "${OUT_DIR}"

if [[ ! -e "${BAG_PATH}" ]]; then
  echo "[ERROR] Bag not found: ${BAG_PATH}" >&2; exit 1
fi

log_header "実験 4 (KS): 運動パターン依存性"
echo "Bag        : ${BAG_PATH}"
echo "PoseSource : /localization/kinematic_state (Odometry)"

echo "[INFO] Analyzing trajectory via /localization/kinematic_state..."
python3 "${SCRIPTS_DIR}/analyze_trajectory.py" \
  "${BAG_PATH}" "${OUT_DIR}/trajectory_ks.json" \
  --pose-topic "/localization/kinematic_state" \
  --use-odom

echo "[RESULT] Trajectory quality:"
cat "${OUT_DIR}/trajectory_ks.json"

for n in 50 100 200 300 400 500; do
  echo ""
  echo "--- use_n_scans = ${n} ---"
  run_calibration "${BAG_PATH}" "${OUT_DIR}" "scans_${n}" \
    "use_n_scans:=${n}"
done

python3 - "${OUT_DIR}" <<'PYEOF'
import json, pathlib, sys

out = pathlib.Path(sys.argv[1])
print("\n=== use_n_scans vs キャリブレーション結果 (KS) ===")
print(f"{'n_scans':<12} {'x':>8} {'y':>8} {'z':>8} {'rz':>8} {'τ[ms]':>8} {'KNN':>10}")
for cf in sorted(out.glob("calibration_scans_*.json")):
    n = cf.stem.replace("calibration_scans_", "")
    c = json.loads(cf.read_text())
    tau = c.get("time_offset_s")
    tau_s = f"{tau*1000:.2f}" if tau is not None else "—"
    knn = c.get("knn_cost")
    knn_s = f"{knn:.1f}" if knn is not None else "—"
    print(
        f"  {n:<10} {c.get('x',0):>8.4f} {c.get('y',0):>8.4f} "
        f"{c.get('z',0):>8.4f} {c.get('rz',0):>8.4f} {tau_s:>8} {knn_s:>10}"
    )
PYEOF

echo ""
echo "Results: ${OUT_DIR}"
