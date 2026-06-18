#!/usr/bin/env bash
# eval/scripts/lib_common_ks.sh
# lib_common.sh を拡張して /localization/kinematic_state (Odometry) 用の設定に上書きする

EVAL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${EVAL_DIR}/scripts/lib_common.sh"

# 上書き: kinematic_state 用 config を使用
BASE_CONFIG="${CONFIGS_DIR}/kinematic_state.param.yaml"

# 上書き: setup.bash が unbound 変数を参照するため set +u で保護しながらソース
ros_source() {
  set +u
  [[ -z "${ROS_DISTRO:-}" ]] && source "${ROS_SETUP}"
  source "${PROJECT_SETUP}"
  set -u
}
