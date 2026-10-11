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
| `noise/` | **観測ノイズ R を真値なしで求める**（`estimate_noise.py`、図は `plot_noise.py`） |
| `eval/` | 録画の再生・評価・パラメータの調整（`tune_run.py`、`tune_batch.py`、`summarize.py`、`transition_eval.py`、`make_window_bag.py`）。候補は `batches/`、結果は `results/` |
| `plot/` | 図（`plot_*.py`、`check_*.py`）。`plot_spin.py` は bag の読み込みや図の共通部品も持つ。`rerender_results.sh` で `workspace/results` の図を全部描き直す |
| `gazebo/` | Gazebo での走行と録画（`run_gazebo_test.sh` はホストで実行） |
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

## 録画を再生して評価・調整する（`eval/`）

```bash
python3 eval/tune_run.py <録画> <試行名> <estimator> '<parameters の JSON>' ['use_ekf:=false']
python3 eval/tune_batch.py <録画> eval/batches/<候補>.json eval/results/<結果>.jsonl
SCORE_KEYS=trans_max_mean,steady_mean,ang_mean,xy_mean python3 eval/summarize.py eval/results/<結果>.jsonl <基準の接頭辞>
```

`ROS_DOMAIN_ID=99` で再生するので Gazebo・MATLAB とは混ざらない。指標（`tune_run.py` の出力）は真値を使う（Gazebo だけ）。

## Gazebo で走らせて録る（`gazebo/run_gazebo_test.sh`、ホストで実行）

```bash
localization_tools/gazebo/run_gazebo_test.sh <estimator> <bag 名> [launch の追加引数]
```

Gazebo を起動し直して初期位置から、左 1 周 → 右 1 周 → 3 秒前進（1 rad/s、1.5 m/s）。始める前に `sync.sh push` する。

## 図

描いた図は `workspace/results/`、作業の記録は `workspace/dev_log.md`（図を埋め込む）。図のルール：後方の線ほど太く、
点線は使わない、真値は一番後ろの太い灰色、題・軸ラベル・凡例の頭文字は大文字（保存前に `plot_spin.capitalize_labels`）。
