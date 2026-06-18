#!/usr/bin/env bash
# 実験 1 (KS): 再現性テスト — kinematic_state pose ソース
# Usage: exp1_ks.sh <bag_path> [N=3]

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/scripts/lib_common_ks.sh"
ros_source

BAG_PATH="${1:?Usage: exp1_ks.sh <bag_path> [N]}"
N="${2:-3}"
OUT_DIR="${RESULTS_DIR}/exp1_repeatability_ks"
mkdir -p "${OUT_DIR}"

log_header "実験 1 (KS): 再現性テスト (N=${N})"
echo "Bag        : ${BAG_PATH}"
echo "PoseSource : /localization/kinematic_state (Odometry)"

if [[ ! -e "${BAG_PATH}" ]]; then
  echo "[ERROR] Bag not found: ${BAG_PATH}" >&2; exit 1
fi

for i in $(seq 1 "${N}"); do
  echo ""
  echo "--- Run ${i}/${N} ---"
  run_calibration "${BAG_PATH}" "${OUT_DIR}" "r${i}" \
    "use_n_scans:=500"
done

python3 - "${OUT_DIR}" <<'PYEOF'
import json, statistics, pathlib, sys

out = pathlib.Path(sys.argv[1])
calibs = sorted(out.glob("calibration_r*.json"))
data = [json.loads(f.read_text()) for f in calibs]

keys = ["x", "y", "z", "rx", "ry", "rz"]
print(f"\n--- Repeatability (N={len(data)}) ---")
print(f"{'param':<8} {'mean':>10} {'σ':>10} {'max-min':>10}")
for k in keys:
    vals = [d[k] for d in data if d.get(k) is not None]
    if vals:
        mn = statistics.mean(vals)
        sd = statistics.stdev(vals) if len(vals) > 1 else 0.0
        rng = max(vals) - min(vals)
        print(f"  {k:<6} {mn:>10.5f} {sd:>10.5f} {rng:>10.5f}")

tau_vals = [d["time_offset_s"]*1000 for d in data if d.get("time_offset_s") is not None]
if tau_vals:
    sd_tau = statistics.stdev(tau_vals) if len(tau_vals) > 1 else 0.0
    print(f"  {'τ[ms]':<6} {statistics.mean(tau_vals):>10.3f} {sd_tau:>10.3f}")
PYEOF

echo ""
echo "Results: ${OUT_DIR}"
