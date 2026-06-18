#!/usr/bin/env bash
# 実験 3: パラメータ感度分析 (one-factor-at-a-time)
# Usage: exp3_sensitivity.sh <bag_path>

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/scripts/lib_common.sh"
ros_source

BAG_PATH="${1:?Usage: exp3_sensitivity.sh <bag_path>}"
OUT_DIR="${RESULTS_DIR}/exp3_sensitivity"
mkdir -p "${OUT_DIR}"

if [[ ! -e "${BAG_PATH}" ]]; then
  echo "[ERROR] Bag not found: ${BAG_PATH}" >&2; exit 1
fi

log_header "実験 3: パラメータ感度分析"
echo "Bag: ${BAG_PATH}"

# --- スイープ定義: "ros_param_key display_name val1 val2 ..." ---
run_sweep() {
  local param_key="$1"
  local display_name="$2"
  shift 2
  local values=("$@")

  echo ""
  echo "=== Sweeping: ${display_name} (${param_key}) ==="

  for val in "${values[@]}"; do
    run_id="${param_key}_${val}"
    echo "  - ${param_key} = ${val}"
    # use_n_scans をデフォルト 500 に固定 (OOM 防止); use_n_scans スイープ時は上書きされる
    if [[ "${param_key}" == "use_n_scans" ]]; then
      run_calibration "${BAG_PATH}" "${OUT_DIR}" "${run_id}" \
        "${param_key}:=${val}"
    else
      run_calibration "${BAG_PATH}" "${OUT_DIR}" "${run_id}" \
        "use_n_scans:=500" "${param_key}:=${val}"
    fi
    cat > "${OUT_DIR}/params_${run_id}.json" <<JSON
{"param_name":"${param_key}","param_value":"${val}"}
JSON
  done
}

run_sweep "keep_points_ratio"  "サンプリング率"       0.005 0.01 0.02 0.05
run_sweep "use_n_scans"        "使用スキャン数"        50 100 200 500
run_sweep "local_knn_max_dist" "KNN最大距離"           0.05 0.1 0.2 0.5
run_sweep "knn_k"              "近傍点数 k"            1 5 10
run_sweep "max_evals"          "最大評価回数"          100.0 200.0 500.0
run_sweep "motion_compensation" "モーション補正"       true false

# サマリー
python3 - <<'PYEOF'
import json, pathlib

out = pathlib.Path("${OUT_DIR}")
groups = {}
for pf in sorted(out.glob("params_*.json")):
    p = json.loads(pf.read_text())
    gname = p["param_name"]
    run_id = pf.stem.replace("params_", "")
    cf = out / f"calibration_{run_id}.json"
    if cf.exists():
        c = json.loads(cf.read_text())
        groups.setdefault(gname, []).append({**p, **c})

print("\n=== パラメータ感度サマリー ===")
for gname, rows in groups.items():
    print(f"\n[{gname}]")
    print(f"  {'value':<12} {'x':>8} {'y':>8} {'z':>8} {'rz':>8} {'τ[ms]':>8} {'KNN':>10} {'t[s]':>6}")
    for r in rows:
        tau = r.get("time_offset_s")
        tau_s = f"{tau*1000:.2f}" if tau is not None else "—"
        knn = r.get("knn_cost")
        knn_s = f"{knn:.1f}" if knn is not None else "—"
        print(
            f"  {r['param_value']:<12} "
            f"{r.get('x', 0):>8.4f} {r.get('y', 0):>8.4f} {r.get('z', 0):>8.4f} "
            f"{r.get('rz', 0):>8.4f} {tau_s:>8} {knn_s:>10} {r.get('elapsed_s', '—')!s:>6}"
        )
PYEOF

echo ""
echo "Results: ${OUT_DIR}"
