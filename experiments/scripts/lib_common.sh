#!/usr/bin/env bash
# experiments/scripts/lib_common.sh — 実験スクリプト共通ライブラリ
# source experiments/scripts/lib_common.sh で読み込む
#
# ディレクトリ解決:
#   EXP_DIR  = experiments/                       (このファイルの 1 つ上)
#   WS_DIR   = colcon ワークスペースのルート       (experiments の 3 つ上)
#              = <ws>/src/lidar_align/experiments → <ws>
#   環境変数 COLCON_WS を設定すればワークスペースを明示的に上書きできる。

EXP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPTS_DIR="${EXP_DIR}/scripts"
CONFIGS_DIR="${EXP_DIR}/configs"
RESULTS_DIR="${EXP_DIR}/results"
BASE_CONFIG="${CONFIGS_DIR}/base.param.yaml"

# colcon ワークスペースのルート (install/setup.bash がある場所)
WS_DIR="${COLCON_WS:-$(cd "${EXP_DIR}/../../.." && pwd)}"

ROS_SETUP="/opt/ros/humble/setup.bash"
PROJECT_SETUP="${WS_DIR}/install/setup.bash"

# ---------------------------------------------------------------------------
# ROS 2 環境をソース (未ソースの場合のみ)
# ---------------------------------------------------------------------------
ros_source() {
  set +u
  # shellcheck disable=SC1090
  [[ -z "${ROS_DISTRO:-}" ]] && source "${ROS_SETUP}"
  # shellcheck disable=SC1090
  source "${PROJECT_SETUP}"
  set -u
}

# ---------------------------------------------------------------------------
# キャリブレーション実行
#
# Usage:
#   run_calibration <bag_path> <out_dir> <run_id> [param:=value ...]
#
# 追加パラメータは ROS 2 の --ros-args -p 形式で渡す。例:
#   run_calibration /path/to/bag out/ r1 keep_points_ratio:=0.02 use_n_scans:=200
#
# 結果ファイル:
#   <out_dir>/calibration_<run_id>.txt   — lidar_align 出力
#   <out_dir>/calibration_<run_id>.json  — パース済み JSON
#   <out_dir>/stdout_<run_id>.log        — 標準出力ログ
# ---------------------------------------------------------------------------
run_calibration() {
  local bag_path="$1"
  local out_dir="$2"
  local run_id="$3"
  shift 3
  # 残りの引数 = 追加 ros params (key:=value 形式)

  mkdir -p "${out_dir}"
  local calib_txt="${out_dir}/calibration_${run_id}.txt"
  local log_file="${out_dir}/stdout_${run_id}.log"
  local calib_json="${out_dir}/calibration_${run_id}.json"

  # 追加パラメータを -p key:=val 配列に変換
  local extra_args=()
  for kv in "$@"; do
    extra_args+=(-p "${kv}")
  done

  echo "[$(date +%T)] run_id=${run_id}  bag=${bag_path}" | tee -a "${log_file}"
  local t_start t_end
  t_start=$(date +%s)

  ros2 run lidar_align lidar_align_node \
    --ros-args \
    --params-file "${BASE_CONFIG}" \
    -p "input_bag_path:=${bag_path}" \
    -p "output_calibration_path:=${calib_txt}" \
    -p "output_pointcloud_path:=${out_dir}/aligned_${run_id}.ply" \
    "${extra_args[@]}" \
    2>&1 | tee -a "${log_file}"

  t_end=$(date +%s)
  local elapsed=$(( t_end - t_start ))
  echo "elapsed_s: ${elapsed}" >> "${log_file}"

  if [[ ! -f "${calib_txt}" ]]; then
    echo "[ERROR] calibration.txt not generated for run_id=${run_id}" >&2
    return 1
  fi

  # calibration.txt → JSON
  python3 "${SCRIPTS_DIR}/parse_calibration.py" \
    "${calib_txt}" "${calib_json}" --log "${log_file}"

  # elapsed_s を JSON に追記
  python3 - <<PYEOF
import json, pathlib
p = pathlib.Path("${calib_json}")
d = json.loads(p.read_text())
d["elapsed_s"] = ${elapsed}
p.write_text(json.dumps(d, indent=2))
PYEOF

  echo "[$(date +%T)] Done: ${calib_json}  (${elapsed}s)"
}

# ---------------------------------------------------------------------------
# 誤差計算ラッパー
# Usage: compute_error_wrap <estimated.json> <gt.json> <output.json>
# ---------------------------------------------------------------------------
compute_error_wrap() {
  python3 "${SCRIPTS_DIR}/compute_error.py" "$1" "$2" "$3"
}

# ---------------------------------------------------------------------------
# ログ出力ヘルパー
# ---------------------------------------------------------------------------
log_header() {
  echo ""
  echo "============================================================"
  echo " $*"
  echo "============================================================"
}
