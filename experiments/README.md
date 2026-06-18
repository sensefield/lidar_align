# lidar_align 評価実験 — 環境構築・実行手順

`lidar_align`（3D LiDAR ⇄ 6-DoF pose の外部キャリブレーション）を rosbag に対して評価する
一連の実験スクリプト。本書は **環境構築と実行手順**を扱う。

---

## 1. 環境構築

`lidar_align` パッケージ本体のインストール（ROS 2 Humble・システム依存・`colcon build`）は
**リポジトリトップの [../README.md](../README.md) の "Installation" / "Running" 節**に従う。

このディレクトリの実験を回すために追加で必要なのは以下だけ。

1. **ワークスペースのレイアウト**
   実験スクリプトは「colcon ワークスペースのルート = `experiments/` の 3 つ上」を前提とする
   （`<ws>/src/lidar_align/experiments/`）。別レイアウトの場合は `COLCON_WS` 環境変数で
   ワークスペースのルート（`install/setup.bash` のある場所）を明示する:

   ```bash
   export COLCON_WS=/path/to/your/colcon_ws
   ```

2. **パッケージのビルドと source**（トップ README の通り。ワークスペースのルートで）

   ```bash
   cd <ws>
   colcon build --packages-select lidar_align
   source install/setup.bash
   ```
   スクリプトは実行時に `install/setup.bash` を自動 source するため、手動 source は必須ではない。

3. **Python 依存**（解析スクリプト用）

   ```bash
   pip install numpy pyyaml
   ```
   bag を読む inject/analyze 系スクリプトは rosbag2 の Python API を使うため、
   ROS 2 Humble を source 済みのシェルで実行すること。

4. **評価対象の rosbag**
   bag はサイズが大きくリポジトリには含まれない。各自で用意し、実行時にパスを渡す。
   既定のトピック・初期値などは `configs/*.param.yaml` を参照（必要に応じて編集）。

---

## 2. 実行手順

すべてのスクリプトは `experiments/` 直下から、**bag のパスを引数**に与えて実行する。
結果は `experiments/results/<実験名>/` に書き出される（`.gitignore` 済み）。

### 2.1 一括実行（pose ソース = GNSS）

```bash
cd <ws>/src/lidar_align/experiments
./run_all.sh /path/to/your.bag            # 実験 1〜5 を順に実行
./run_all.sh /path/to/your.bag "1 3"      # 実験 1 と 3 のみ
```
最後に `results/report_<日時>.md` が自動生成される。

### 2.2 個別実行（pose ソース = GNSS）

| # | スクリプト | 内容 | 主な引数 |
|---|-----------|------|----------|
| 1 | `exp1_repeatability.sh <bag> [N]` | 再現性テスト（N 回実行のばらつき） | N（既定 5） |
| 2 | `exp2_perturbation.sh <bag> [baseline.json]` | オフセット注入テスト | baseline（既定 exp1 の r1） |
| 3 | `exp3_sensitivity.sh <bag>` | パラメータ感度分析（one-factor-at-a-time） | — |
| 4 | `exp4_motion.sh <bag>` | 運動パターン依存性（`use_n_scans` スイープ＋軌跡品質） | — |
| 5 | `exp5_initial_guess.sh <bag> [N]` | 初期値依存性（2 段階 vs ローカルのみ） | N（既定 20） |

```bash
./exp1_repeatability.sh /path/to/your.bag 5
./exp3_sensitivity.sh   /path/to/your.bag
```

### 2.3 pose ソース = kinematic_state（Odometry）

pose ソースを `/localization/kinematic_state`（EKF Odometry）に切り替えた版。
`configs/kinematic_state.param.yaml` を使う以外は同じ実験構成。

```bash
./exp1_ks.sh /path/to/your.bag     # 再現性
./exp2_ks.sh /path/to/your.bag     # オフセット注入
./exp3_ks.sh /path/to/your.bag     # 感度分析
./exp4_ks.sh /path/to/your.bag     # 運動依存性
./exp5_ks.sh /path/to/your.bag     # 初期値依存性
```

---

## 3. ディレクトリ構成

```
experiments/
├── README.md                       # このファイル（環境構築・実行手順）
├── run_all.sh                      # GNSS 実験 1〜5 の一括実行＋レポート生成
├── exp1_repeatability.sh … exp5_initial_guess.sh   # GNSS 各実験
├── exp1_ks.sh … exp5_ks.sh         # kinematic_state 各実験
├── configs/
│   ├── base.param.yaml             # GNSS pose 用パラメータ
│   └── kinematic_state.param.yaml  # kinematic_state pose 用
├── ground_truth/
│   └── gt_rosbag_20260408.json     # TF chain ノミナル値（GT）
├── scripts/
│   ├── lib_common.sh               # 共通ライブラリ（run_calibration 等）
│   ├── lib_common_ks.sh            # KS 用に base config を差し替える拡張
│   ├── parse_calibration.py        # calibration.txt → JSON
│   ├── compute_error.py            # 推定値 vs GT の e_t / e_r / e_τ
│   ├── analyze_trajectory.py       # 軌跡品質（非平面性スコア・累積回転）
│   ├── inject_offset_pose.py       # PoseStamped にオフセット注入（exp2）
│   ├── inject_offset_odom.py       # Odometry にオフセット注入（exp2_ks）
│   └── generate_report.py          # 全結果を集計して Markdown レポート化
└── results/                        # 実行時生成（.gitignore 済み）
```

---

## 4. パイプラインの仕組み（補足）

`scripts/lib_common.sh` の `run_calibration` が中核:

1. `configs/base.param.yaml`（または KS 版）を `--params-file` で渡し、`input_bag_path` と
   出力パスを `-p` で上書きして `ros2 run lidar_align lidar_align_node` を実行。
   個別パラメータも `key:=value` 形式で追記でき、感度分析（exp3）はこれでスイープする。
2. 出力された `calibration.txt` を `parse_calibration.py` で JSON 化（並進・回転・time offset・KNN コスト）。
3. GT 比較が必要な実験では `compute_error.py` が SE(3) 残差から `e_t [m]` / `e_r [deg]` を計算。

新しいパラメータを試したい場合は該当 exp スクリプトのスイープ定義に値を足せばよい。
