# localization_tools

自己位置推定（`localization_bringup`、NDT + ekf_localizer）の検証・調整・描画・テスト走行のプログラム。
robot_localization と比べる機能（再生・合成データのテスト・比較の図）は 2026-10-15 に廃止した（git の履歴に残っている）。
**編集はホスト（`Docker/localization/src/localization_tools`）、実行は自己位置推定コンテナ `test` の `/root/ros2_ws/localization_tools`**
（ROS のパッケージではないので `src` の外。colcon が中を探さないよう `COLCON_IGNORE` を置いている）。

```bash
./sync.sh push   # ホスト -> コンテナ（eval/results の既存のファイルはコンテナ側を残す）
./sync.sh pull   # コンテナ -> ホスト（eval/results と /root/ros2_ws/fig の図を Docker/localization/results へ）
```

コンテナ内のコマンドは ROS の環境が要るので `docker exec test bash -ic '...'` で実行する。
録画（bag）は `test:/root/ros2_ws/bags` の下に、**Gazebo と実機で分けて**置く（`eval/bag_paths.py`）。図は `/root/ros2_ws/fig`。
どちらも地図（`/root/ros2_ws/map`）も、ホストの `Docker/localization/docker/data/{bags,fig,map}` を割り当てたもの（2026-10-14 から。コンテナを作り直しても消えない。
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

録画と図（`/root/ros2_ws/bags`・`/root/ros2_ws/fig`。どちらにも `COLCON_IGNORE` を置く）は、ホストの `Docker/localization/docker/data/{bags,fig}` を
割り当てたものなので、コンテナを作り直しても消えない。

| フォルダ | 内容 |
|---|---|
| `calibration/` | **パラメータ推定（プログラムとパラメータ）**：`run_calibration.sh`（推定用の動きで走って、続けて推定する）、`motion.py`（推定用の動き）、`calibrate.py`（録画から R・倍率・取付をまとめて推定）、`calibrate_noise.py`（観測ノイズ R、真値なし）、`calibrate_ndt_noise.py`（NDT の R を「真値 + ノイズ」の初期値で計算し直して求める。Gazebo）、`calibrate_mount_scale.py`（LiDAR の取付位置とオドメトリの倍率、真値なし）、図は `plot_noise.py`・`plot_mount_scale.py`。設定は `param/motion.yaml`（推定用の動き）と `param/calibrate.yaml`（推定。各プログラムのオプションの既定値） |
| `eval/` | 録画の再生・評価・パラメータの調整（`tune_run.py`、`tune_batch.py`、`summarize.py`、`transition_eval.py`、`make_window_bag.py`）。候補は `batches/`、結果は `results/` |
| `plot/` | 図（`plot_*.py`、`check_*.py`）。`plot_spin.py` は bag の読み込みや図の共通部品も持つ。`rerender_results.sh` で `Docker/localization/results` の図を全部描き直す（NDT の R の図は計算に約 9 分かかるので `FULL=1` のときだけ） |
| `record_rviz.sh` | **RViz の画面の録画（ホストで実行）**。下の「RViz の画面の録画」 |
| `drive/` | **テスト走行と録画（Gazebo・実機共通）**：`run_test.sh`（ホストかコンテナの中で実行）、`drive_test.py`（走らせる）、`profiles.yaml`（動き）、`config/gazebo.yaml`・`config/red.yaml`・`blue.yaml`・`white.yaml`（実機の車椅子ごと。速さの上限・走ってよい範囲・トピック・録画するトピック）、`config/real_in_gazebo.yaml`（実機の設定を Gazebo で試す）、`config/gazebo_ground.yaml` と `profiles_velocity.yaml`（Gazebo の地面でオドメトリと真値の速度を比べる） |

## R を求める（`calibration/calibrate_noise.py`）

```bash
python3 calibration/calibrate_noise.py /root/ros2_ws/bags/gazebo/<録画> --out /root/ros2_ws/fig/noise_<名前> [--truth]
```

- 指令（cmd_vel）が一定の区間（変化の 0.5 s 後から次の変化の 0.1 s 前まで、1 s 以上）だけを使う。
- 各観測から、前後 0.5 s の局所 1 次式（滑らかな真の動き）を取り除いた残差の分散を R とする（当てはめの自由度は補正）。
  - オドメトリ：v・omega（`--scale` を掛けた後）→ `R_odom`
    - **R_v は定速で直進する区間の v の残差**から求める（直進の区間が無ければ動いている区間全体）。
    - **R_omega は R_v と車輪の間隔 b（`--tread`）から計算する**：左右の車輪の速度のノイズが独立で同じ大きさなら
      v = (vR + vL)/2、omega = (vR − vL)/b より R_omega = 4 R_v (s_omega/s_v)² / b²（s は `--scale` の倍率。b/s_omega が実効的な車輪の間隔）。
      直接測った omega の残差（直進・旋回・動いている区間）も表とヒストグラムに並べる（旋回中は滑りなど車輪の速度以外のばらつきも入る）。
  - NDT：LiDAR の姿勢を取付（`/tf_static`）で車体に直して当てはめ、LiDAR に戻した値との差（map 座標）→ `R_ndt`
- `R_ndt` は動いている区間（旋回・直進）の分散。どれも残差の時間的なつながり（積分相関時間）を掛ける（`--no-correlation` で掛けない）。
  env の `parameters` に貼れる yaml を出す。
- 図：`_segments.png`（区間と残差）、`_summary.png`（標準偏差の比較）、`_hist.png`（**残差のヒストグラムと正規分布**。v は直進、
  omega は直進と旋回、NDT は動いている区間。尖度と ±4.5 σ の外の点の数も書く）。
  - 縦軸は相対度数（Relative frequency。全体を 1 とした棒ごとの割合。範囲 ±4.5 σ を 45 本に分ける）。NDT の 6 つは縦軸の範囲をそろえる。
    パラメータ推定の図は枠を 4 辺とも黒にする（box on）。
  - 青の線は、外れた点に左右されない σ（1.4826 x 中央絶対偏差）の正規分布（同じ割合に直したもの）。
    残差の σ（二乗平均の平方根。R に使う値）や、v と車輪の間隔から求めた omega の σ は図の上の文字で示す。
- 真値は使わないので実機の録画にも使える（オドメトリ・NDT・指令・`/tf_static` を録っておく。トピック名はオプションで変える）。
  `--truth` は Gazebo だけで、真値との差の分散を並べて出す（比較用。R には使わない）。

## NDT の R を「真値 + ノイズ」の初期値で求める（`calibration/calibrate_ndt_noise.py`、Gazebo）

```bash
python3 calibration/calibrate_ndt_noise.py /root/ros2_ws/bags/gazebo/<止まる区間のある録画> --out /root/ros2_ws/fig/ndt_noise_<名前>   # コンテナ
python3 calibration/calibrate_ndt_noise.py --from-json <出力の json> --out <...>   # 計算し直さず図だけ
```

- 録画は止まる区間のある Gazebo の走行（パラメータ推定用の動き、drive の `stops` など。区間は `<bag>_phases.txt` の `stop_*`）。
  `calibration gazebo` の推定（`calibrate.py`）でも行う。
- 使うのは各区間の、止まってから `settle`（1 s）後から `use_time`（10 s）。真値はその間の `/wheelchair/pose` の平均。
- NDT だけを Gazebo の設定で `ROS_DOMAIN_ID=99` に起動し、**使う区間（とその前の `lead` 1 s）だけを**、区間ごとに
  `ros2 bag play --start-offset` で途中から速さ 1 倍で流す（区間の終わりの点群が届いたら止める）。点群が届くたびに「真値 + ノイズ」を
  初期値として TF で出す。1 回目はノイズなし、2 回目以降はノイズあり（`passes` 回）。
- ノイズは 6 軸それぞれ正規分布 N(0, σ²)。σ は `param/calibrate.yaml` の `ndt_noise.sigma`（x, y 0.02 m、z 0.01 m、roll, pitch 0.3°、
  yaw 1.0°：運用中の EKF の誤差の rms の 2〜4 倍）。
- 時間は 区間の数 x（`lead` + `use_time` + 約 1 s）x（`passes` + 1）。推定用の動き（3 区間）で約 2.5 分。
- NDT の解と真値の差から、止まった区間ごとの平均（地図と Gazebo の一定のずれ）を引いた分散を R_ndt とする。
  初期値のずれが解に残る傾きと、図（初期値のずれと解の誤差の散布図 `.png`、誤差のヒストグラムと正規分布 `_hist.png`）も出す。

## パラメータ推定を走行とセットで（`calibration/run_calibration.sh`、コマンド `calibration`）

```bash
calibration/run_calibration.sh <gazebo|red|blue|white> [<bag 名>]          # 走行 → 録画 → 推定（コマンド calibration red など）
python3 calibration/motion.py <gazebo|red|blue|white>                # 走る前の計画（コマンド calibration_plan）
python3 calibration/calibrate.py <gazebo|red|blue|white> <bag>    # 録画から推定だけ（コマンド calibrate）
```

- 推定用の動きは 1 つ（drive の動きとは別）。**`param/motion.yaml` の 5 つ**（Gazebo・実機で共通の 1 組。値は m/s・rad/s・s・周の
  絶対値で、`drive/config/<target>.yaml` の上限を超えると走る前に止める）：
  その場で左に `spin_turns` 周（`spin_omega`）→ 止まる `stop_time` 秒 → 直進 `straight_v` x `straight_time` 秒（片道）→ 止まる。
  今の値（0.4 rad/s で 4 周、5 s、0.4 m/s x 20 s）で実機の上限で約 1.5 分。
- `motion.py` が動きを drive の動きのファイルの形にし、`drive/run_test.sh <target> calibration <bag 名>` に `PROFILES` で渡して走る
  （bag 名を省くと `<target>_calibration_<年月日_時分秒>`）。途中で止めたら推定しない。
- 推定は `calibrate.py`：`calibrate_noise.py`（R_odom・R_ndt とも走行中の残差から）、`calibrate_mount_scale.py`（倍率・取付）を、
  target に合わせたトピック・フレーム・倍率・車輪の間隔で動かす（指令は `drive/config/<target>.yaml`、
  オドメトリ・フレーム・`odom_scale` は localization_bringup の `param/env/<target>.yaml`、車輪の間隔は Gazebo が `param/calibrate.yaml` の
  `noise.gazebo_tread`、実機が whill_odometry の `tread_width`）。Gazebo は真値との比較（`--truth`）も出す。
- 推定の設定（区間の取り方・窓・直進の安定の判定など）は `param/calibrate.yaml`（各プログラムのオプションの既定値。オプションで上書きできる）。
- 推定にかかる時間は数秒（252 s の録画で 8 s）。
- 結果は 1 回ごとに `/root/ros2_ws/fig/calibration/<bag 名>/`：`summary.yaml`（env に入れる候補 `candidate` と今の値。**env の yaml には書き込まない**）、
  `noise.*`（`noise_hist.png` など）、`mount_scale.*`・`mount_scale_yaw.png`。
- NDT の R を「真値 + ノイズ」の初期値で求める `calibrate_ndt_noise.py`（Gazebo）は、2026-10-15 にユーザー指示で `calibrate.py` から外した（単独では使える）。

## LiDAR の取付位置とオドメトリの倍率（`calibration/calibrate_mount_scale.py`）

```bash
python3 calibration/calibrate_mount_scale.py /root/ros2_ws/bags/gazebo/<録画> [...] --out /root/ros2_ws/fig/mount_scale [--map <地図の pcd>] [--truth]
```

真値なしで、NDT の LiDAR の姿勢とオドメトリ（倍率を掛ける前）から求める。その場の旋回と直進が入った録画を使う。
- omega の倍率：旋回の区間の NDT の yaw の変化 / オドメトリの omega の積分（推定用の動きは左回りだけなので、左回りの値。
  Gazebo では左回り 0.921〜0.924、右回り 0.913〜0.914 と向きで違った）。v の倍率：直進の区間の移動距離 / v の積分
- 取付の x, y：その場の旋回で LiDAR が回転の中心のまわりを回る円を最小二乗で当てはめる。
- 取付の yaw：直進の区間で、LiDAR の yaw − 車体の進む向き（横滑りしないと仮定）を各時刻で求め、RANSAC で外れ値を除いて平均する。
  - 車体の位置（LiDAR の位置 − R(yaw_L) m'。m' は旋回で求めた取付で、取付の yaw に左右されない）と LiDAR の yaw を、前後 0.5 s（`yaw_window` 1 s）の
    局所 1 次式で平滑化し、各時刻の進む向き（位置の傾き）と LiDAR の yaw の差を求める。オドメトリの v（同じ窓で平滑化）が区間の後半の平均の
    `steady_tol`（5 %）以内でない時刻は使わない。
  - RANSAC：各時刻の値を 1 つ選んで仮の値とし、そこから `ransac_threshold_deg`（0.3°）以内の時刻（インライア）を数えることを `ransac_iterations`
    （200）回くり返して、インライアが最も多いものを選ぶ。インライアの平均（定数の最小二乗）を取付の yaw とする。
  - 図 `<出力>_yaw.png`（`mount_scale_yaw.png`）に各時刻の値（インライア・外れ値）と最終値を描く。
  - 区間全体の円弧の当てはめ、始点と終点を結ぶ方法、Censi ほか（IEEE T-RO 2013）の同時推定とも比べ、2026-10-16 にこの方法に決めた（`dev_log.md` の 49〜52）。
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

RViz の点群の表示に使う姿勢の違い（同じ点群の時刻の、EKF の予測値（`/tf`）・EKF の NDT の更新値（`/tf_ndt_update`。RViz が使う）・NDT の解（`/ndt_pose`）の差。
`publish_trigger: cloud` の録画。実機でもよい。`/tf_ndt_update` が無い録画は予測値と NDT の解だけ。`dev_log.md` の 53・54）：

```bash
python3 plot/plot_prior_posterior.py /root/ros2_ws/bags/real/blue_spin5_20261016_160557 /root/ros2_ws/fig/prior_posterior_blue_spin5.png
```

MATLAB 用の出力（ekf_localizer の `/localization/base_pose`・`vlp16_pose`・`vlp16_points`）を確かめる（`dev_log.md` の 61）。
`eval/replay_outputs.py` で録画全体を再生して出力を録り（`ROS_DOMAIN_ID=99`。実機の録画は環境の名前と地図も渡す）、
`plot/plot_matlab_outputs.py` で base_pose の周期、vlp16_pose が `/tf_ndt_update` × 取付と同じか、vlp16_points に同じ stamp の
vlp16_pose があるか・入力の点群と中身が同じか、届くまでの時間を出し、図（届くまでの時間の分布、点群の時刻の base_pose と NDT で更新した直後の値の差）を描く：

```bash
python3 eval/replay_outputs.py /root/ros2_ws/bags/gazebo/gazebo_spin5_20261016_111637 vlp16_check_spin5
python3 eval/replay_outputs.py /root/ros2_ws/bags/real/blue_spin5_20261016_160557 vlp16_check_blue_spin5 blue /root/ros2_ws/map/10build_4F_C_20260414.pcd
python3 plot/plot_matlab_outputs.py /root/ros2_ws/bags/gazebo/tune/vlp16_check_spin5 /root/ros2_ws/fig/matlab_outputs/gazebo_spin5.png /root/ros2_ws/bags/gazebo/gazebo_spin5_20261016_111637
```

実機の録画をコンテナの中で再生すると、大きい点群が受け取る側で落ちることがある（blue では NDT が解いたのが半分ほど）（ソケットの受信バッファ 208 kB に 314 kB の点群が収まらないため。ホストの net.core.rmem_max・rmem_default を 8 MB にすると全部届く。dev_log.md の 66・67）。

MATLAB（`MaeharaTakumi/whill_common_matlab` の `cr_debug` ブランチの CR_debug）がこれらを受け取って組にできるかは、`replay_outputs.py` で
再生している間に、ホストで MATLAB を `-batch` で動かして確かめる（`ROS_DOMAIN_ID=99`、Fast DDS の localhost のプロファイル。`dev_log.md` の 62）：

```bash
REPO="<whill_common_matlab のフォルダ>" TEST_DURATION=70 TEST_OUT=/tmp/cr_debug.mat \
  matlab -batch "run('<localization_tools>/matlab/check_cr_debug.m')"                                        # ホスト
python3 plot/plot_cr_debug_pairs.py /root/ros2_ws/fig/matlab_outputs/cr_debug_gazebo_spin5.mat /root/ros2_ws/fig/matlab_outputs/cr_debug_gazebo_spin5.png   # コンテナ（.mat を写して）
```

main.m と同じ流れ（Estimate2・Control2・DataLogger。指令は配信しない）で記録し、`plotter.DataPlotter`・`plotter.myplotResult` の図と
`plotter.myplotPointCloudVideo` の動画（`TEST_OUT_DIR/result`。地図は `MAP`）まで作るのは `matlab/check_cr_debug_plot.m`（`dev_log.md` の 63・64）：

```bash
REPO="<whill_common_matlab のフォルダ>" TEST_DURATION=50 TEST_OUT_DIR=/tmp/cr_debug_plot MAP=<点群地図（.pcd）> \
  matlab -batch "run('<localization_tools>/matlab/check_cr_debug_plot.m')"                                   # ホスト
```

録画の再生（自己位置推定）・MATLAB・図と動画を 1 回で行うのは `matlab/replay_matlab.sh`（ホストで実行。`dev_log.md` の 65）。
MATLAB を先に起動して購読ができてから録画を再生し（`RECORD=0` の `replay_outputs.py`）、新しい組が `STOP_IDLE`（既定 5）秒
来なくなったら MATLAB が止まって図と動画を作る。ROS の時刻は録画の `/clock`（`SIM_TIME=1`）なので、点群の古さ（`LiDAR_age`）も録画の時刻で出る：

```bash
matlab/replay_matlab.sh /root/ros2_ws/bags/gazebo/gazebo_spin5_20261016_111637 ../../results/matlab_outputs/replay_gazebo_spin5
matlab/replay_matlab.sh /root/ros2_ws/bags/real/blue_spin5_20261016_160557 ../../results/matlab_outputs/replay_blue_spin5 blue /root/ros2_ws/map/10build_4F_C_20260414.pcd
```

指標は stamp の時刻の 6 軸の誤差、実時間の誤差（100 Hz の各時刻に受信済みの最新の `/ekf_pose` とその時刻の真値の差）、
切り替え時（指令が変わる瞬間の 0.1 s 前〜1 s 後）の yaw、推定の古さ。xy は地図と Gazebo の座標のずれ（場所で変わる）を含む。

## テスト走行と録画（`drive/`、Gazebo・実機共通）

`run_test.sh` はホストからでも、自己位置推定のコンテナの中（docker が無い）からでも実行できる。ホストからは `docker exec` で
コンテナ（`LOC`、既定 `test`）に入って動かし、コンテナの中ではその場で直接動かす（そのスクリプトのある
`localization_tools` を使うので、置き場所は `/root/localization_tools` でもよい）。コンテナの中からは Gazebo を起動し直せないので、
Gazebo のときは Gazebo が動いていて車椅子が初期位置にあること。

```bash
localization_tools/drive/run_test.sh gazebo <動き> [<bag 名> [launch の追加引数]]   # ホストかコンテナの中
localization_tools/drive/run_test.sh <red|blue|white> <動き> [<bag 名>]              # 実機。端末から（確認がある）
python3 drive/drive_test.py <gazebo|red|blue|white> --list               # 動きの一覧
python3 drive/drive_test.py <gazebo|red|blue|white> <動き> --simulate    # 計算した軌跡と範囲（ROS 不要）
```

コンテナの中では、`drive`（= `drive/run_test.sh`。bag 名を省くと `<target>_<動き>_<日時>`）・`drive_list`（動きの一覧。`drive_list red spin5` で計画）・`bags_list`（録画の一覧。`bags_list real` で実機だけ）
のコマンドでも呼べる（localizer_docker の `src/aliases.txt`）。

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
  車椅子は `run_test.sh <red|blue|white>` の色で選ぶ（`config/<色>.yaml`。自己位置推定の launch の `env:=<色>` と同じ色にする。
  選んだ色の `cmd_vel` を受けるノードが無ければ、見つかった車椅子を表示して止める）。録画するのは点群・`motor_speed_ts`・
  `cmd_vel`・バッテリーと推定の出力（`motor_angle_ts`・`joy_ts`・`speed_mode`・GNSS は録らない）。

  ```bash
  localization_tools/drive/run_test.sh red spin real_red_spin_01
  ```
- **予行演習**：`CONFIG=real_in_gazebo drive/run_test.sh gazebo <動き> <bag 名>` で、実機の上限・`/ekf_pose`・ジョイスティックの
  監視（`/test/joy_ts`）を Gazebo で試せる。
- **地面でオドメトリと真値の速度を比べる**（Gazebo）：`config/gazebo_ground.yaml` は車椅子を世界の原点（建物の外の ground_plane、
  z −0.4 から落とす。yaw 0）に置き、自己位置推定は起動しない（設定の `gazebo_launch_args` を Gazebo の launch に渡し、
  `localization: false` で推定を起動しない）。上限は v 1.0 m/s・ω 1.0 rad/s、範囲の前は 15 m。動きは `profiles_velocity.yaml`
  （`spin5_w05`・`spin5_w10`：その場で左に 5 周、`straight_v05`・`straight_v10`：10 s 直進。前後に止まる）。図は `plot/plot_odom_truth.py`。

  ```bash
  CONFIG=gazebo_ground PROFILES=/root/ros2_ws/localization_tools/drive/profiles_velocity.yaml \
    drive/run_test.sh gazebo spin5_w05 gazebo_ground_spin5_w05
  python3 plot/plot_odom_truth.py /root/ros2_ws/bags/gazebo/gazebo_ground_spin5_w05 /root/ros2_ws/fig/odom_truth/odom_truth_spin_w05.png omega   # コンテナ
  ```

## RViz の画面の録画（`record_rviz.sh`、ホスト）

コンテナの RViz はホストの X に描いているので、ホストの ffmpeg で RViz の窓を探して録画する（X11 のセッションだけ。
ホストに `ffmpeg`・`xdotool`・`xwininfo`（x11-utils）が要る。コンテナには ffmpeg が無い）。

```bash
./record_rviz.sh                          # 開いている RViz の窓を Ctrl-C まで録る（./rviz_<日時>.mp4）
./record_rviz.sh -t 60 -o ~/Videos/run.mp4  # 60 s だけ、出力を指定
./record_rviz.sh --new                    # この後に開く RViz の窓を待って録る（起動の直前に実行しておく）
./record_rviz.sh --list                   # RViz の窓の一覧（ID・大きさ・題名）
./record_rviz.sh --select                 # 録る窓をクリックで選ぶ
./record_rviz.sh --id <ID>                # 窓の ID で選ぶ
```

- 窓を指定しなければ、RViz の窓が 1 つならそれ、複数なら前面の窓が RViz ならそれを録る（そうでなければ一覧を出して止まる）。
- 窓の中身だけを録る（x11grab の `-window_id`）。窓を動かしても、画面からはみ出していても録れる。録画中に窓の大きさを変えると止まる。
  他の窓が上に重なるとその部分も写る。RViz を閉じると録画も止まる。
- 出力は断片化した MP4（15 fps、H.264）なので、途中で止まっても再生できる。1 分で約 4 MB。
- 録画した bag を再生して RViz を録ることもできる（実機の走行中に録らなくてよい）。

## 図

描いた図は `Docker/localization/results/`、作業の記録は `Docker/localization/dev_log.md`（図を埋め込む）。図のルール：後方の線ほど太く、
点線は使わない、真値は一番後ろの太い灰色、題・軸ラベル・凡例の頭文字は大文字（保存前に `plot_spin.capitalize_labels`）。
