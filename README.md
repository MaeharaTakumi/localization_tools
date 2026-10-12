# localization_tools

自己位置推定（`localization_bringup`、ekf_localizer / robot_localization）の検証・調整・描画のプログラム。
**編集はホスト（`workspace/localization_tools`）、実行は自己位置推定コンテナ `localizer_rl` の `/root/localization_tools`**。

```bash
./sync.sh push   # ホスト -> コンテナ（eval/results の既存のファイルはコンテナ側を残す）
./sync.sh pull   # コンテナ -> ホスト（eval/results と /root/fig の図を workspace/results へ）
```

コンテナ内のコマンドは ROS の環境が要るので `docker exec localizer_rl bash -ic '...'` で実行する。
bag は `localizer_rl:/root/bags`、再生した結果の bag は `/root/bags/tune/<試行名>`、図は `/root/fig`。

| フォルダ | 内容 |
|---|---|
| `noise/` | **観測ノイズ R を求める**：`estimate_noise.py`（真値なし、走行中の残差）、`ndt_init_noise.py`（NDT の R を「真値 + ノイズ」の初期値で計算し直して求める）。図は `plot_noise.py` |
| `calib/` | **LiDAR の取付位置とオドメトリの倍率を推定する**（`estimate_calibration.py`、真値なし。図は `plot_calibration.py`） |
| `eval/` | 録画の再生・評価・パラメータの調整（`tune_run.py`、`tune_batch.py`、`summarize.py`、`transition_eval.py`、`make_window_bag.py`）。候補は `batches/`、結果は `results/` |
| `plot/` | 図（`plot_*.py`、`check_*.py`）。`plot_spin.py` は bag の読み込みや図の共通部品も持つ。`rerender_results.sh` で `workspace/results` の図を全部描き直す |
| `gazebo/` | Gazebo での走行と録画（`run_gazebo_test.sh` はホストで実行）。走り方は `drive_spin.py`（左右 1 周 + 前進）、`drive_stops.py`（向き・位置を変えて 13 回止まる）、`drive_profile.py`（推定器の比較用：加減速・8 の字・スラローム・後退） |
| `test/` | Gazebo なしの合成データのテスト（robot_localization の経路） |

## R を求める（`noise/estimate_noise.py`）

```bash
python3 noise/estimate_noise.py /root/bags/<録画> --out /root/fig/noise_<名前> [--truth]
```

- 指令（cmd_vel）が一定の区間（変化の 0.5 s 後から次の変化の 0.1 s 前まで、1 s 以上）だけを使う。
- 各観測から、前後 0.5 s の局所 1 次式（滑らかな真の動き）を取り除いた残差の分散を R とする（当てはめの自由度は補正）。
  - オドメトリ：v・omega（`--scale` を掛けた後）→ `R_odom`
  - NDT：LiDAR の姿勢を取付（`/tf_static`）で車体に直して当てはめ、LiDAR に戻した値との差（map 座標）→ `R_ndt`
- 推奨値は動いている区間（旋回・直進）の分散。env の `parameters` に貼れる yaml を出す。
- 真値は使わないので実機の録画にも使える（オドメトリ・NDT・指令・`/tf_static` を録っておく。トピック名はオプションで変える）。
  `--truth` は Gazebo だけで、真値との差の分散を並べて出す（比較用。R には使わない）。

## NDT の R を「真値 + ノイズ」の初期値で求める（`noise/ndt_init_noise.py`）

```bash
DRIVE=drive_stops.py localization_tools/gazebo/run_gazebo_test.sh ekf_localizer stops_ekf   # ホスト：止まる走行を録る
python3 noise/ndt_init_noise.py /root/bags/stops_ekf --out /root/fig/ndt_init_stops         # コンテナ（約 9 分）
```

- 真値は止まった区間（始めの 1 s を除く）の `/wheelchair/pose` の平均（止まっているので真値の時刻のずれは効かない）。
- NDT だけを Gazebo の設定で `ROS_DOMAIN_ID=99` に起動し、点群が届くたびに「真値 + ノイズ」を初期値として TF で出す。
  1 回目はノイズなし、2 回目以降はノイズあり（`--passes`）。ノイズは `--sigma`（既定 x, y 0.02 m、z 0.01 m、roll, pitch 0.3°、
  yaw 1.0°：運用中の EKF の誤差の rms の 2〜4 倍）。
- NDT の解と真値の差から、止まった区間ごとの平均（地図と Gazebo の一定のずれ）を引いた分散を R_ndt とする。
  初期値のずれが解に残る傾きと、図（初期値のずれと解の誤差の散布図）も出す。

## LiDAR の取付位置とオドメトリの倍率（`calib/estimate_calibration.py`）

```bash
python3 calib/estimate_calibration.py /root/bags/<録画> [...] --out /root/fig/calib [--map <地図の pcd>] [--truth]
```

真値なしで、NDT の LiDAR の姿勢とオドメトリ（倍率を掛ける前）から求める。その場の旋回と直進が入った録画を使う。
- omega の倍率：旋回の区間の NDT の yaw の変化 / オドメトリの omega の積分。v の倍率：直進の区間の移動距離 / v の積分
- 取付の x, y：その場の旋回で LiDAR が回転の中心のまわりを回る円を最小二乗で当てはめる。取付の yaw：直進の向きと LiDAR の yaw の差
- 取付の roll・pitch（床が水平と仮定）、z（`--map` の地図の床の高さとの差。参考）

## 録画を再生して評価・調整する（`eval/`）

```bash
python3 eval/tune_run.py <録画> <試行名> <estimator> '<parameters の JSON>' ['use_ekf:=false']
python3 eval/tune_batch.py <録画> eval/batches/<候補>.json eval/results/<結果>.jsonl
SCORE_KEYS=trans_max_mean,steady_mean,ang_mean,xy_mean python3 eval/summarize.py eval/results/<結果>.jsonl <基準の接頭辞>
```

`ROS_DOMAIN_ID=99` で再生するので Gazebo・MATLAB とは混ざらない。指標（`tune_run.py` の出力）は真値を使う（Gazebo だけ）。

推定器の比較（どの動きの録画にも使える指標。`dev_log.md` の 22）：

```bash
python3 eval/compare_run.py batch eval/batches/batch_compare.json eval/results/results_compare.jsonl   # 動き x 推定器 x 回数
RANGE=1 python3 eval/compare_run.py table eval/results/results_compare.jsonl                           # 表
python3 plot/plot_motion_compare.py report /root/fig/compare eval/results/results_compare.jsonl        # 軌跡・まとめ・時間推移の図
```

指標は stamp の時刻の 6 軸の誤差、実時間の誤差（100 Hz の各時刻に受信済みの最新の `/ekf_pose` とその時刻の真値の差）、
切り替え時（指令が変わる瞬間の 0.1 s 前〜1 s 後）の yaw、推定の古さ。xy は地図と Gazebo の座標のずれ（場所で変わる）を含む。

## Gazebo で走らせて録る（`gazebo/run_gazebo_test.sh`、ホストで実行）

```bash
localization_tools/gazebo/run_gazebo_test.sh <estimator> <bag 名> [launch の追加引数]
```

Gazebo を起動し直して初期位置から、左 1 周 → 右 1 周 → 3 秒前進（1 rad/s、1.5 m/s）。始める前に `sync.sh push` する。
走り方は `DRIVE` で変える（例 `DRIVE="drive_profile.py curve"`。`python3 gazebo/drive_profile.py curve --simulate` で部屋に収まるか計算できる）。

## 図

描いた図は `workspace/results/`、作業の記録は `workspace/dev_log.md`（図を埋め込む）。図のルール：後方の線ほど太く、
点線は使わない、真値は一番後ろの太い灰色、題・軸ラベル・凡例の頭文字は大文字（保存前に `plot_spin.capitalize_labels`）。
