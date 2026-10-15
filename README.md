# localization_tools

自己位置推定（`localization_bringup`、NDT + ekf_localizer）の検証・調整・描画・テスト走行のプログラム。
robot_localization と比べる機能（再生・合成データのテスト・比較の図）は 2026-10-15 に廃止した（git の履歴に残っている）。
**編集はホスト（`workspace/localization_tools`）、実行は自己位置推定コンテナ `test` の `/root/ros2_ws/localization_tools`**
（ROS のパッケージではないので `src` の外。colcon が中を探さないよう `COLCON_IGNORE` を置いている）。

```bash
./sync.sh push   # ホスト -> コンテナ（eval/results の既存のファイルはコンテナ側を残す）
./sync.sh pull   # コンテナ -> ホスト（eval/results と /root/ros2_ws/fig の図を workspace/results へ）
```

コンテナ内のコマンドは ROS の環境が要るので `docker exec test bash -ic '...'` で実行する。
録画（bag）は `test:/root/ros2_ws/bags` の下に、**Gazebo と実機で分けて**置く（`eval/bag_paths.py`）。図は `/root/ros2_ws/fig`。
どちらも地図（`/root/ros2_ws/map`）も、ホストの `localizer_docker/data/{bags,fig,map}` を割り当てたもの（2026-10-14 から。コンテナを作り直しても消えない。
コンテナの中で作ったファイルはホストでは root の持ち物になる）。

| 場所 | 中身 |
|---|---|
| `/root/ros2_ws/bags/gazebo/<名前>` | Gazebo の録画。隣に `<名前>_phases.txt`（動作の開始時刻）、`<名前>_drive.log`（テスト走行のログ。2026-10-14 から）、`<名前>_info.yaml`（録画の情報） |
| `/root/ros2_ws/bags/gazebo/tune/<試行名>` | その再生の出力（`tune_run.py`・`compare_run.py`。結果の行の `out_bag`） |
| `/root/ros2_ws/bags/gazebo/check/<名前>` | ツールの動作確認などの録画（比較・調整に使わない。`run_test.sh` の bag 名を `check/<名前>` にする） |
| `/root/ros2_ws/bags/real/<名前>`、`/root/ros2_ws/bags/real/tune/` | 実機の録画とその再生の出力（同じく `_info.yaml` を付ける） |

録画の情報（`eval/bag_info.py`）：Gazebo か実機か、録画日時、長さ、走らせ方、用途、録画したときの推定器、
Gazebo のオドメトリの移動平均（車輪の角速度から判定）、オドメトリの omega と真値の比（`wheel_separation_multiplier` を
試した録画は約 1、普段は約 1.06〜1.10）。`drive/run_test.sh` は録画のあとに自動で書く（用途は環境変数 `PURPOSE`）。

```bash
python3 eval/bag_info.py list                         # 録画の一覧
python3 eval/bag_info.py write <bag> --source gazebo|real --drive '...' --purpose '...' [--notes '...']
```

**録画と図はコンテナの中（`/root/ros2_ws/bags`・`/root/ros2_ws/fig`。どちらにも `COLCON_IGNORE` を置く）なので、コンテナを作り直すと消える**（`docker stop`・`start` では残る）。作り直す前にホストへ退避する。

| フォルダ | 内容 |
|---|---|
| `noise/` | **観測ノイズ R を求める**：`estimate_noise.py`（真値なし、走行中の残差）、`ndt_init_noise.py`（NDT の R を「真値 + ノイズ」の初期値で計算し直して求める）。図は `plot_noise.py` |
| `calib/` | **LiDAR の取付位置とオドメトリの倍率を推定する**（`estimate_calibration.py`、真値なし。図は `plot_calibration.py`） |
| `eval/` | 録画の再生・評価・パラメータの調整（`tune_run.py`、`tune_batch.py`、`summarize.py`、`transition_eval.py`、`make_window_bag.py`）。候補は `batches/`、結果は `results/` |
| `plot/` | 図（`plot_*.py`、`check_*.py`）。`plot_spin.py` は bag の読み込みや図の共通部品も持つ。`rerender_results.sh` で `workspace/results` の図を全部描き直す（NDT の R の図は計算に約 9 分かかるので `FULL=1` のときだけ） |
| `drive/` | **テスト走行と録画（Gazebo・実機共通）**：`run_test.sh`（ホストかコンテナの中で実行）、`drive_test.py`（走らせる）、`profiles.yaml`（動き）、`config/gazebo.yaml`・`config/real.yaml`（速さの上限・走ってよい範囲・トピック・録画するトピック）、`config/real_in_gazebo.yaml`（実機の設定を Gazebo で試す） |

## R を求める（`noise/estimate_noise.py`）

```bash
python3 noise/estimate_noise.py /root/ros2_ws/bags/gazebo/<録画> --out /root/ros2_ws/fig/noise_<名前> [--truth]
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
localization_tools/drive/run_test.sh gazebo stops stops_ekf                       # ホスト：止まる走行を録る
python3 noise/ndt_init_noise.py /root/ros2_ws/bags/gazebo/stops_ekf --out /root/ros2_ws/fig/ndt_init_stops         # コンテナ（約 9 分）
```

- 真値は止まった区間（始めの 1 s を除く）の `/wheelchair/pose` の平均（止まっているので真値の時刻のずれは効かない）。
- NDT だけを Gazebo の設定で `ROS_DOMAIN_ID=99` に起動し、点群が届くたびに「真値 + ノイズ」を初期値として TF で出す。
  1 回目はノイズなし、2 回目以降はノイズあり（`--passes`）。ノイズは `--sigma`（既定 x, y 0.02 m、z 0.01 m、roll, pitch 0.3°、
  yaw 1.0°：運用中の EKF の誤差の rms の 2〜4 倍）。
- NDT の解と真値の差から、止まった区間ごとの平均（地図と Gazebo の一定のずれ）を引いた分散を R_ndt とする。
  初期値のずれが解に残る傾きと、図（初期値のずれと解の誤差の散布図）も出す。

## LiDAR の取付位置とオドメトリの倍率（`calib/estimate_calibration.py`）

```bash
python3 calib/estimate_calibration.py /root/ros2_ws/bags/gazebo/<録画> [...] --out /root/ros2_ws/fig/calib [--map <地図の pcd>] [--truth]
```

真値なしで、NDT の LiDAR の姿勢とオドメトリ（倍率を掛ける前）から求める。その場の旋回と直進が入った録画を使う。
- omega の倍率：旋回の区間の NDT の yaw の変化 / オドメトリの omega の積分。v の倍率：直進の区間の移動距離 / v の積分
- 取付の x, y：その場の旋回で LiDAR が回転の中心のまわりを回る円を最小二乗で当てはめる。取付の yaw：直進の向きと LiDAR の yaw の差
- 取付の roll・pitch（床が水平と仮定）、z（`--map` の地図の床の高さとの差。参考）

## 録画を再生して評価・調整する（`eval/`）

```bash
python3 eval/tune_run.py <録画> <試行名> '<parameters の JSON>' ['use_ekf:=false']
python3 eval/tune_batch.py <録画> eval/batches/<候補>.json eval/results/<結果>.jsonl
SCORE_KEYS=trans_max_mean,steady_mean,ang_mean,xy_mean python3 eval/summarize.py eval/results/<結果>.jsonl <基準の接頭辞>
```

`ROS_DOMAIN_ID=99` で再生するので Gazebo・MATLAB とは混ざらない。指標（`tune_run.py` の出力）は真値を使う（Gazebo だけ）。

いろいろな動きの録画での比較（どの動きの録画にも使える指標。`dev_log.md` の 22）：


```bash
python3 eval/compare_run.py batch eval/batches/batch_compare.json eval/results/results_compare.jsonl   # 動き x 候補（label） x 回数
RANGE=1 python3 eval/compare_run.py table eval/results/results_compare.jsonl                           # 表
python3 plot/plot_motion_compare.py report /root/ros2_ws/fig/compare eval/results/results_compare.jsonl        # 軌跡・まとめ・時間推移の図
```

指標は stamp の時刻の 6 軸の誤差、実時間の誤差（100 Hz の各時刻に受信済みの最新の `/ekf_pose` とその時刻の真値の差）、
切り替え時（指令が変わる瞬間の 0.1 s 前〜1 s 後）の yaw、推定の古さ。xy は地図と Gazebo の座標のずれ（場所で変わる）を含む。

## テスト走行と録画（`drive/`、Gazebo・実機共通）

`run_test.sh` はホストからでも、自己位置推定のコンテナの中（docker が無い）からでも実行できる。ホストからは `docker exec` で
コンテナ（`LOC`、既定 `test`）に入って動かし、コンテナの中ではその場で直接動かす（そのスクリプトのある
`localization_tools` を使うので、置き場所は `/root/localization_tools` でもよい）。コンテナの中からは Gazebo を起動し直せないので、
Gazebo のときは Gazebo が動いていて車椅子が初期位置にあること。

```bash
localization_tools/drive/run_test.sh gazebo <動き> <bag 名> [launch の追加引数]   # ホストかコンテナの中
localization_tools/drive/run_test.sh real <動き> <bag 名>                                    # 端末から（確認がある）
python3 drive/drive_test.py <gazebo|real> --list               # 動きの一覧
python3 drive/drive_test.py <gazebo|real> <動き> --simulate    # 計算した軌跡と範囲（ROS 不要）
```

- **動き**（`profiles.yaml`）：`spin`（その場で左右 1 周 + 3 秒前進）、`spin5`（その場で左に 5 周 → 右に 5 周 + 3 秒前進）、`stops`（13 か所で止まる）、`accel`（加減速・停止、8.5 m）、
  `curve`（半径 0.75 m の 8 の字）、`slalom`（波長 2.5 m）、`reverse`（後退・S 字・前後の切り替え）。速さは上限に対する割合、
  直進は距離、旋回は角度、円は半径で決めるので、上限を変えても形・大きさは同じ。
- **設定**（`config/<target>.yaml`）：速さの上限（Gazebo v 1.5・後退 0.8 m/s・ω 1.0 rad/s、実機 0.5・0.3・0.5）、走ってよい範囲
  （出発時の車体の中心を基準に前 10 m・後ろ 1 m・左右 2 m。両方）、指令・姿勢・ジョイスティックのトピック、録画するトピック。
- **安全**：走る前に計算した軌跡が範囲に収まるか確かめる。走行中は、範囲の外に出た・姿勢（Gazebo は真値、実機は `/ekf_pose`
  を取付で車体に直したもの）が 0.5 s 届かない・ジョイスティックが倒された（実機）・時計が止まった・区間が 60 s を超えた・Ctrl-C
  のどれかで 0 の指令を出して止める（止めた理由は録画の情報ファイルのメモに書く）。実機は計画を表示して `yes` と入力するまで走らない。
  whill_driver も、指令が 1 s 途切れると止まり、ジョイスティックの操作を優先する。
- **Gazebo**：起動し直して車椅子を初期位置に戻し、推定器を起動して走らせ、`/root/ros2_ws/bags/gazebo/<bag 名>` に録る。
- **実機**：自己位置推定（`/ekf_pose`）・whill_driver・LiDAR が動いていることを確かめてから、`/root/ros2_ws/bags/real/<bag 名>` に録る
  （推定の起動は自分で行う。地図と初期姿勢が要るため）。test の `.bashrc` の通信設定を実機用にしておく。
  車椅子は `WHILL=red|blue|white`（既定は `config/real.yaml` の `whill: red`。自己位置推定の launch の `env:=<色>` と合わせる。
  選んだ色の `cmd_vel` を受けるノードが無ければ、見つかった車椅子を表示して止める）。録画するのは点群・`motor_speed_ts`・
  `cmd_vel`・バッテリーと推定の出力（`motor_angle_ts`・`joy_ts`・`speed_mode`・GNSS は録らない）。

  ```bash
  WHILL=red localization_tools/drive/run_test.sh real spin real_red_spin_01
  ```
- **予行演習**：`CONFIG=real_in_gazebo drive/run_test.sh gazebo <動き> <bag 名>` で、実機の上限・`/ekf_pose`・ジョイスティックの
  監視（`/test/joy_ts`）を Gazebo で試せる。

## 図

描いた図は `workspace/results/`、作業の記録は `workspace/dev_log.md`（図を埋め込む）。図のルール：後方の線ほど太く、
点線は使わない、真値は一番後ろの太い灰色、題・軸ラベル・凡例の頭文字は大文字（保存前に `plot_spin.capitalize_labels`）。
