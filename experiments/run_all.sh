#!/usr/bin/env bash
# ============================================================
# lidar_align 精度評価 マスタースクリプト
#
# Usage:
#   ./eval/run_all.sh <bag_path> [gt_json] [experiments]
#
#   bag_path    : 渡された rosbag のパス (ディレクトリ or .mcap)
#   experiments : 実行する実験番号スペース区切り or "all" (デフォルト: all)
#                 実験 1=再現性, 2=注入テスト, 3=感度分析, 4=運動依存性, 5=初期値依存性
#
# 例:
#   ./eval/run_all.sh /data/new_bag
#   ./eval/run_all.sh /data/new_bag "1 2 3"
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/scripts/lib_common.sh"

# ============================================================
# 引数処理
# ============================================================
NEW_BAG="${1:?Usage: run_all.sh <bag_path> [experiments]}"
EXPERIMENTS="${2:-all}"

if [[ ! -e "${NEW_BAG}" ]]; then
  echo "[ERROR] Bag not found: ${NEW_BAG}" >&2; exit 1
fi

# bag パスをほかのスクリプトが参照できるよう保存
echo "${NEW_BAG}" > "${SCRIPT_DIR}/new_bag.path"

ros_source

echo ""
echo "============================================================"
echo "  lidar_align 精度評価"
echo "============================================================"
echo "  Bag      : ${NEW_BAG}"
echo "  実験     : ${EXPERIMENTS}"
echo "  出力先   : ${RESULTS_DIR}"
echo "  開始時刻 : $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"

mkdir -p "${RESULTS_DIR}"

# 実行する実験番号リスト
if [[ "${EXPERIMENTS}" == "all" ]]; then
  EXP_LIST=(1 2 3 4 5)
else
  read -ra EXP_LIST <<< "${EXPERIMENTS}"
fi

run_exp() {
  local num="$1"; shift
  log_header "実験 ${num}"
  bash "${SCRIPT_DIR}/exp${num}_$(exp_name "${num}").sh" "$@" \
    || echo "[WARN] 実験 ${num} が失敗しました (続行)" >&2
}

exp_name() {
  case "$1" in
    1) echo "repeatability" ;; 2) echo "perturbation" ;;
    3) echo "sensitivity" ;;   4) echo "motion" ;;
    5) echo "initial_guess" ;; *) echo "unknown" ;;
  esac
}

# ============================================================
# 実験実行
# ============================================================
printf '%s\n' "${EXP_LIST[@]}" | grep -q '^1$' && \
  run_exp 1 "${NEW_BAG}" 5

printf '%s\n' "${EXP_LIST[@]}" | grep -q '^2$' && \
  run_exp 2 "${NEW_BAG}" \
    "${RESULTS_DIR}/exp1_repeatability/calibration_r1.json"

printf '%s\n' "${EXP_LIST[@]}" | grep -q '^3$' && \
  run_exp 3 "${NEW_BAG}"

printf '%s\n' "${EXP_LIST[@]}" | grep -q '^4$' && \
  run_exp 4 "${NEW_BAG}"

printf '%s\n' "${EXP_LIST[@]}" | grep -q '^5$' && \
  run_exp 5 "${NEW_BAG}" 20

# ============================================================
# レポート生成
# ============================================================
log_header "レポート生成"
REPORT="${RESULTS_DIR}/report_$(date +%Y%m%d_%H%M%S).md"
python3 "${SCRIPTS_DIR}/generate_report.py" "${RESULTS_DIR}" "${REPORT}"

echo ""
echo "============================================================"
echo "  評価完了"
echo "  レポート : ${REPORT}"
echo "  終了時刻 : $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"
