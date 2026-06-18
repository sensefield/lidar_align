#!/usr/bin/env bash
# 実験 2 (KS): オフセット注入テスト — kinematic_state (Odometry) pose ソース
# Usage: exp2_ks.sh <bag_path> [baseline_json]

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/scripts/lib_common_ks.sh"
ros_source

BAG_PATH="${1:?Usage: exp2_ks.sh <bag_path> [baseline_json]}"
BASELINE_JSON="${2:-${RESULTS_DIR}/exp1_repeatability_ks/calibration_r1.json}"
OUT_DIR="${RESULTS_DIR}/exp2_perturbation_ks"
PERTURBED_DIR="${OUT_DIR}/perturbed_bags"
mkdir -p "${OUT_DIR}" "${PERTURBED_DIR}"

if [[ ! -e "${BAG_PATH}" ]]; then
  echo "[ERROR] Bag not found: ${BAG_PATH}" >&2; exit 1
fi

log_header "実験 2 (KS): オフセット注入テスト"
echo "Bag     : ${BAG_PATH}"
echo "Baseline: ${BASELINE_JSON}"

# <id> <dx> <dy> <dz> <drx> <dry> <drz_rad>
declare -a CASE_IDS=(  P1    P2     P3        P4        P5       )
declare -a CASE_DX=(   0.05  0.10   0.0       0.0       0.0      )
declare -a CASE_DY=(   0.0   0.0    0.0       0.0       0.0      )
declare -a CASE_DZ=(   0.0   0.0    0.0       0.0       0.0      )
declare -a CASE_DRX=(  0.0   0.0    0.0       0.0       0.0      )
declare -a CASE_DRY=(  0.0   0.0    0.0       0.0       0.0      )
declare -a CASE_DRZ=(  0.0   0.0    0.01745   0.05236   0.08727  )

for idx in "${!CASE_IDS[@]}"; do
  case_id="${CASE_IDS[$idx]}"
  dx="${CASE_DX[$idx]}"
  dy="${CASE_DY[$idx]}"
  dz="${CASE_DZ[$idx]}"
  drx="${CASE_DRX[$idx]}"
  dry="${CASE_DRY[$idx]}"
  drz="${CASE_DRZ[$idx]}"

  echo ""
  echo "--- Case ${case_id}: dx=${dx} drz=${drz} ---"

  cat > "${OUT_DIR}/params_${case_id}.json" <<JSON
{
  "case_id":   "${case_id}",
  "delta_x":  ${dx},
  "delta_y":  ${dy},
  "delta_z":  ${dz},
  "delta_rx": ${drx},
  "delta_ry": ${dry},
  "delta_rz": ${drz}
}
JSON

  perturbed_bag="${PERTURBED_DIR}/${case_id}"
  if [[ ! -d "${perturbed_bag}" ]]; then
    python3 "${SCRIPTS_DIR}/inject_offset_odom.py" \
      "${BAG_PATH}" "${perturbed_bag}" \
      --dx "${dx}" --dy "${dy}" --dz "${dz}" \
      --drx "${drx}" --dry "${dry}" --drz "${drz}" \
      --odom-topic "/localization/kinematic_state"
  else
    echo "[INFO] Perturbed bag exists, skipping injection."
  fi

  run_calibration "${perturbed_bag}" "${OUT_DIR}" "${case_id}" \
    "use_n_scans:=500"

  if [[ -f "${BASELINE_JSON}" ]]; then
    compute_error_wrap \
      "${OUT_DIR}/calibration_${case_id}.json" \
      "${BASELINE_JSON}" \
      "${OUT_DIR}/error_${case_id}.json"
    python3 -c "
import json, pathlib
e = json.loads(pathlib.Path('${OUT_DIR}/error_${case_id}.json').read_text())
print(f\"  e_t={e['e_t_m']:.4f}m  e_r={e['e_r_deg']:.3f}°\")
"
  fi
done

python3 - "${OUT_DIR}" <<'PYEOF'
import json, pathlib, sys

out = pathlib.Path(sys.argv[1])
print("\n=== 注入テスト サマリー (KS) ===")
print(f"{'Case':<5} {'Δx[m]':>8} {'Δrz[°]':>9} {'e_t[m]':>8} {'e_r[°]':>8}")
for pf in sorted(out.glob("params_*.json")):
    cid = pf.stem.replace("params_", "")
    p = json.loads(pf.read_text())
    ef = out / f"error_{cid}.json"
    drz_deg = p["delta_rz"] * 180 / 3.14159
    if ef.exists():
        e = json.loads(ef.read_text())
        print(f"  {cid:<3} {p['delta_x']:>8.3f} {drz_deg:>9.3f} "
              f"{e.get('e_t_m', 0):>8.4f} {e.get('e_r_deg', 0):>8.3f}")
    else:
        print(f"  {cid:<3} {p['delta_x']:>8.3f} {drz_deg:>9.3f}  (誤差データなし)")
PYEOF

echo ""
echo "Results: ${OUT_DIR}"
