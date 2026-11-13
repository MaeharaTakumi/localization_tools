# localization_tools

自己位置推定（`localization_bringup`、NDT + ekf_localizer）のテスト走行・パラメータ推定・再生・描画のプログラム。
**編集はホスト（`Docker/localization/src/localization_tools`）、実行は自己位置推定コンテナ `test` の `/root/ros2_ws/localization_tools`**
（ROS のパッケージではないので `src` の外。colcon が中を探さないよう `COLCON_IGNORE` を置いている）。

廃止したもの（git の履歴にある。経緯は `Docker/localization/dev_log.md`）：robot_localization と比べる機能（2026-10-15）、
真値と比べる評価・調整・図のツール（`tune_run.py`・`compare_run.py`・`plot_spin.py` など。2026-11-13、開発ログの 93）、
NDT の R を「真値 + ノイズ」で求める `calibrate_ndt_noise.py`（2026-11-13、90）。

```bash
./sync.sh push   # ホスト -> コンテナ（コンテナ側は .git 以外を消してから写す）
./sync.sh pull   # コンテナの図（/root/ros2_ws/fig）-> ホストの Docker/localization/results
```

コンテナ内のコマンドは ROS の環境が要るので `docker exec test bash -ic '...'` で実行する。
録画（bag）は `test:/root/ros2_ws/bags` の下に **Gazebo と実機で分けて**置く（`eval/bag_paths.py`）。図は `/root/ros2_ws/fig`。
録画・図・地図（`/root/ros2_ws/map`）は、ホストの `Docker/localization/docker/data/{bags,fig,map}` を割り当てたもの（コンテナを作り直しても消えない。
コンテナの中で作ったファイルはホストでは root の持ち物になる。`bags`・`fig` には `COLCON_IGNORE` を置く）。

| 場所 | 中身 |
|---|---|
| `/root/ros2_ws/bags/gazebo/<名前>` | Gazebo の録画。隣に `<名前>_phases.txt`（動作の開始時刻）、`<名前>_drive.log`（走行のログ）、`<名前>_info.yaml`（録画の情報） |
| `/root/ros2_ws/bags/gazebo/tune/<試行名>` | その再生の出力（`replay_outputs.py`） |
| `/root/ros2_ws/bags/gazebo/check/<名前>` | ツールの動作確認などの録画（`run_test.sh` の bag 名を `check/<名前>` にする） |
| `/root/ros2_ws/bags/real/<名前>`、`/root/ros2_ws/bags/real/tune/` | 実機の録画とその再生の出力 |

**推定の姿勢は `/tf` の map → base**（ekf_localizer が 50 Hz で出す。フレームは localization_bringup の `param/<target>/ekf_localizer.yaml` の
`map_frame_id`・`base_frame_id`）。走行中の姿勢（実機）・軌跡の図・再生の出力は `/tf` を使い、`/ekf_pose` は録らない（2026-11-13 から）。
**Gazebo の録画に真値 `/wheelchair/pose` は入れない**（2026-11-13 から。Gazebo の走行中の姿勢には真値を使う）。

| フォルダ | 内容 |
|---|---|
| `drive/` | **テスト走行と録画（Gazebo・実機共通）**：`run_test.sh`（ホストかコンテナの中で実行）、`drive_test.py`（走らせる）、`profiles.yaml`（動き）、`config/gazebo.yaml`・`red.yaml`・`blue.yaml`・`white.yaml`（速さの上限・トピック・録画するトピック）、`config/real_in_gazebo.yaml`（実機の設定を Gazebo で試す） |
| `calibration/` | **パラメータ推定**：`run_calibration.sh`（推定用の動きで走って、続けて推定する。コマンド `calibration`）、`motion.py`（推定用の動き）、`calibrate.py`（R・倍率・取付をまとめて推定。`run_calibration.sh` が呼ぶ）、`calibrate_noise.py`（観測ノイズ R）、`calibrate_mount_scale.py`（LiDAR の取付とオドメトリの倍率）、図は `plot_noise.py`・`plot_mount_scale.py`。設定は `param/motion.yaml`（推定用の動き）と `param/calibrate.yaml`（各プログラムのオプションの既定値。`params.py` が読む） |
| `eval/` | `replay_outputs.py`（録画を再生して推定器を動かし、出力を録る）、`bag_info.py`（録画の情報ファイル・一覧）、`bag_paths.py`（録画の置き場所） |
| `plot/` | `plot_drive_paths.py`（走行の軌跡）、`plot_matlab_outputs.py`（MATLAB が `/tf` から引く値）、`plot_prior_posterior.py`（RViz の点群に使う姿勢の違い）、`plot_cr_debug_pairs.py`（MATLAB の CR_debug が受け取った組）、`rerender_results.sh`（今のツールで描ける `results/` の図を描き直す。`OUT` で出力先、`T0` でツールの場所） |
| `matlab/` | MATLAB（whill_common_matlab の CR_debug）の確認。`replay_matlab.sh` はホストで実行 |
| `common/` | **共通部品**（各スクリプトは `common` などを `sys.path` に入れて import する）：`env_files.py`（localization_bringup の `param/<env>/` を読む・写して書き換える。`make_env`・`node_params`）、`bag_io.py`（`read_bag`・`phases_of`・`stamp_sec`・`stamp_ns`・`mount_from_tf_static`）、`pose_math.py`（姿勢と 4x4 行列）、`plot_style.py`（色・線幅・`setup`・`style`・`box_on`・`capitalize_labels`・`widths_back_to_front`）、`gazebo_model.py`（Gazebo の車輪の半径と間隔、URDF の取付） |
| `record_rviz.sh` | **RViz の画面の録画（ホストで実行）** |

## テスト走行と録画（`drive/`）

`run_test.sh` はホストからでも、自己位置推定のコンテナの中（docker が無い）からでも実行できる。ホストからは `docker exec` で
コンテナ（`LOC`、既定 `test`）の `/root/ros2_ws/localization_tools` を動かす（ホストで直したら先に `deploy.sh` か `sync.sh push` で写す。
走行のスクリプトは自分では写さない）。コンテナの中からは Gazebo を起動し直せないので、Gazebo が動いていて車椅子が初期位置にあること。

```bash
drive/run_test.sh gazebo <動き> [<bag 名> [launch の追加引数]]   # ホストかコンテナの中
drive/run_test.sh <red|blue|white> <動き> [<bag 名>]              # 実機。端末から（確認がある）
python3 drive/drive_test.py <gazebo|red|blue|white> --list               # 動きの一覧
python3 drive/drive_test.py <gazebo|red|blue|white> <動き> --simulate    # 計算した軌跡（ROS 不要）
```

コンテナの中では `drive`（bag 名を省くと `<target>_<動き>_<日時>`）・`drive_list`・`bags_list [gazebo|real]` のコマンドでも呼べる（localizer_docker の `src/aliases.txt`）。

- **動き**（`profiles.yaml`）：`spin`（その場で左右 1 周 + 3 秒前進）、`spin5`（左に 5 周 → 右に 5 周 + 3 秒前進）、`stops`（13 か所で止まる）、
  `accel`（加減速・停止、8.5 m）、`curve`（半径 0.75 m の 8 の字）、`slalom`（波長 2.5 m）、`reverse`（後退・S 字・前後の切り替え）。
  速さは上限に対する割合。直進は距離、旋回は角度、円は半径で決めるので、上限を変えても形・大きさは同じ。
- **設定**（`config/<target>.yaml`）：速さの上限（Gazebo v 1.5・後退 0.8 m/s・ω 1.0 rad/s、実機 0.5・0.3・0.5）、指令・姿勢・ジョイスティックの
  トピック、録画するトピック。姿勢（`feedback`）は Gazebo が真値（`type: pose`、`/wheelchair/pose`）、実機と `real_in_gazebo` が `/tf` の map → base（`type: tf`。
  1 つのメッセージに入っている刻みのうち、今の時刻以下で最も新しいもの）。
- **走る前の確認**（録画を始めた後に `drive_test.py` が 1 つのノードで行う）：姿勢が 15 s 以内に届いて新しいこと、指令のトピック（実機は選んだ色の
  `cmd_vel`）を受けるノードが 5 s 以内に見つかること（録画の `ros2 bag record` は数えない）。どちらかが欠けたら走らずに止める（見つかった車椅子の
  `cmd_vel` も表示する。短い録画が残り、情報ファイルのメモに理由を書く）。録画するトピックで無いものは表示して続ける。Enter の指示まで約 1.3 s。
- **安全**：走る前に計算した軌跡（出発時の車体の中心を基準にした前・左の範囲）を表示する。実機は Enter を押すまで走らない（何か入力して Enter なら取りやめ）。
  走行中は、姿勢が 0.5 s 届かない・ジョイスティックが倒された（実機）・時計が止まった・区間が 60 s を超えた・Ctrl-C のどれかで 0 の指令を出して止める。
  whill_driver も、指令が 1 s 途切れると止まり、ジョイスティックの操作を優先する。
- **Gazebo**：ホストからなら Gazebo を起動し直して車椅子を初期位置に戻し、推定器を起動して走らせ、`bags/gazebo/<bag 名>` に録る。
- **実機**：`bags/real/<bag 名>` に録りながら走る（推定の起動は自分で行う。地図と初期姿勢が要るため）。test の `.bashrc` の通信設定を実機用にしておく。
  車椅子は `run_test.sh <red|blue|white>` の色で選ぶ（自己位置推定の launch の `env:=<色>` と同じ色にする）。
- **録画するトピック**：実機は点群・`motor_speed_ts`・`cmd_vel`・バッテリー・`/wheelchair/odom`・`/tf`・`/tf_static`・`/initialpose`・`/ekf_odom`・`/ndt_pose`・
  `/localization_debug`（`motor_angle_ts`・`joy_ts`・`speed_mode`・GNSS は録らない）。Gazebo は `/clock`・点群・オドメトリ・`joint_states`・指令と推定の出力。
- **予行演習**：`CONFIG=real_in_gazebo drive/run_test.sh gazebo <動き> <bag 名>` で、実機の上限・`/tf` の姿勢・ジョイスティックの監視（`/test/joy_ts`）を Gazebo で試せる。

走行の軌跡の図（`/tf` の map → base を出発時の位置・向きを基準に描く）：

```bash
python3 plot/plot_drive_paths.py <出力 png> <gazebo|red|blue|white|real_in_gazebo> <ラベル>=<bag> ...
```

## パラメータ推定（`calibration/run_calibration.sh`、コマンド `calibration`）

```bash
calibration/run_calibration.sh <gazebo|red|blue|white> [<bag 名>]   # 走行 → 録画 → 推定
python3 calibration/motion.py <gazebo|red|blue|white>               # 走る前の計画（コマンド calibration_plan）
```

毎回走って推定する（録画から推定だけをするコマンド `calibrate` は 2026-11-13 にやめた。途中で止めたら推定しないので走り直す）。

- 推定用の動きは 1 つ（drive の動きとは別）。**`param/motion.yaml` の 5 つ**（Gazebo・実機で共通。値は m/s・rad/s・s・周の絶対値で、
  `drive/config/<target>.yaml` の上限を超えると走る前に止める）：その場で左に `spin_turns` 周（`spin_omega`）→ 止まる `stop_time` 秒 →
  直進 `straight_v` x `straight_time` 秒（片道）→ 止まる。今の値（0.4 rad/s で 4 周、5 s、0.4 m/s x 20 s）で実機の上限で約 1.5 分。
- `motion.py` が動きを drive の動きのファイルの形にし、`drive/run_test.sh <target> calibration <bag 名>` に `PROFILES` で渡して走る
  （bag 名を省くと `<target>_calibration_<年月日_時分秒>`）。
- 推定は `calibrate.py`：`calibrate_noise.py`（R）、`calibrate_mount_scale.py`（倍率・取付）を、target に合わせたトピック・フレーム・倍率・
  車輪の間隔で動かす（指令は `drive/config/<target>.yaml`、フレーム・`odom_scale` は localization_bringup の `param/<target>/ekf_localizer.yaml`、
  車輪の間隔は Gazebo が `param/calibrate.yaml` の `noise.gazebo_tread`、実機が whill_odometry の `tread_width`）。数秒で終わる。
- 結果は 1 回ごとに `/root/ros2_ws/fig/calibration/<bag 名>/`：`summary.yaml`（`param/<target>/ekf_localizer.yaml` に入れる候補 `candidate` と今の値。
  **yaml には書き込まない**）、`noise.*`（`noise_hist.png` など）、`mount_scale.*`・`mount_scale_yaw.png`。
- 推定の設定（区間の取り方・窓・直進の安定の判定など）は `param/calibrate.yaml`。

### 観測ノイズ R（`calibrate_noise.py`）

真値を使わずに求める。

- 指令（cmd_vel）が一定の区間（変化の 0.5 s 後から次の変化の 0.1 s 前まで、1 s 以上）だけを使う。
- 各観測から、前後 0.5 s の局所 1 次式（滑らかな真の動き）を取り除いた残差の分散を R とする（当てはめの自由度は補正）。
  - オドメトリ：v・omega（`--scale` を掛けた後）→ `R_odom`
    - **R_v は定速で直進する区間の v の残差**から求める（直進の区間が無ければ動いている区間全体）。
    - **R_omega は R_v と車輪の間隔 b（`--tread`）から計算する**：左右の車輪の速度のノイズが独立で同じ大きさなら
      v = (vR + vL)/2、omega = (vR − vL)/b より R_omega = 4 R_v (s_omega/s_v)² / b²（s は `--scale` の倍率。b/s_omega が実効的な車輪の間隔）。
      直接測った omega の残差も表とヒストグラムに並べる（旋回中は滑りなど車輪の速度以外のばらつきも入る）。
  - NDT：LiDAR の姿勢を取付（`/tf_static`）で車体に直して当てはめ、LiDAR に戻した値との差（map 座標）→ `R_ndt`（動いている区間）
- どれも残差の時間的なつながり（積分相関時間）を掛ける（`--no-correlation` で掛けない）。
- 図：`_segments.png`（区間と残差）、`_summary.png`（標準偏差の比較）、`_hist.png`（残差のヒストグラムと正規分布。縦軸は相対度数、
  ±4.5 σ を 45 本に分ける。NDT の 6 つは縦軸をそろえる。青の線は外れた点に左右されない σ（1.4826 x 中央絶対偏差）の正規分布）。

### LiDAR の取付とオドメトリの倍率（`calibrate_mount_scale.py`）

真値なしで、NDT の LiDAR の姿勢とオドメトリ（倍率を掛ける前）から求める。その場の旋回と直進が入った録画を使う。
- omega の倍率：旋回の区間の NDT の yaw の変化 / オドメトリの omega の積分（推定用の動きは左回りだけなので左回りの値）。
  v の倍率：直進の区間の移動距離 / v の積分
- 取付の x, y：その場の旋回で LiDAR が回転の中心のまわりを回る円を最小二乗で当てはめる。
- 取付の yaw：直進の区間で、LiDAR の yaw − 車体の進む向き（横滑りしないと仮定）を各時刻で求め、RANSAC で外れ値を除いて平均する。
  - 車体の位置（LiDAR の位置 − R(yaw_L) m'。m' は旋回で求めた取付）と LiDAR の yaw を前後 0.5 s（`yaw_window`）の局所 1 次式で平滑化し、
    各時刻の進む向きと LiDAR の yaw の差を求める。オドメトリの v が区間の後半の平均の `steady_tol`（5 %）以内でない時刻は使わない。
  - RANSAC：各時刻の値を 1 つ選んで仮の値とし、`ransac_threshold_deg`（0.3°）以内の時刻を数えることを `ransac_iterations`（200）回くり返し、
    最も多いもののインライアの平均を取付の yaw とする。図 `mount_scale_yaw.png` に各時刻の値と最終値を描く。
- 取付の roll・pitch（床が水平と仮定）、z（`--map` の地図の床の高さとの差。参考）。
- 取付の roll・pitch・yaw は表示・`summary.yaml`・図とも **rad**（`param/<色>/env.yaml` の `static_tfs` と同じ）。

## 録画の再生（`eval/replay_outputs.py`）

録画を頭から最後まで再生して推定器を動かし、出力（`/tf`・`/tf_static`・`/ndt_pose`・`/slam_pose`・`/ekf_odom`・`/localization_debug`）を
入力と同じ側の `tune/<試行名>` に録る。`ROS_DOMAIN_ID=99` で動かすので、動いている推定・Gazebo・MATLAB とは混ざらない。
ノードが再生の途中で落ちた回はエラーにする。

```bash
python3 eval/replay_outputs.py /root/ros2_ws/bags/gazebo/<録画> <試行名>                                       # Gazebo
python3 eval/replay_outputs.py /root/ros2_ws/bags/real/<録画> <試行名> <red|blue|white> /root/ros2_ws/map/<地図>.pcd   # 実機
```

- 環境は localization_bringup の `param/<env>/` を `/tmp/replay_env_<試行名>/` に写して使う（`common/env_files.py`。Gazebo は `/tf_static` が
  流れないので URDF と同じ取付を `static_tfs` に書く。実機は `use_sim_time: true` で `--clock` で流す）。
- `PARAMS`（環境変数、JSON）でパラメータを上書きする：`{"ekf_localizer": {"Q": [...]}, "pcl_localization": {...}}` のように
  `{ノード名: {パラメータ: 値}}`（ノードは `ekf_localizer`・`pcl_localization`・`whill_odometry`）。`"__env__"` の中身は `env.yaml` を上書きする。
- `LAUNCH_ARGS`（環境変数）で launch の引数を足す（`matcher:=lidarslam` で NDT の代わりに SLAM、`use_ekf:=false` で EKF なし）。
  `RECORD=0` で出力を録らない、`RATE` で実機の再生の速さ。
- 実機の録画をコンテナの中で再生すると、大きい点群が受け取る側で落ちることがある（受信バッファ 208 kB に 314 kB の点群が収まらない）。
  ホストを起動し直すたびに、アプリの外の端末で `sudo sysctl -w net.core.rmem_max=8388608 net.core.rmem_default=8388608`。
- 実機 blue の `blue_calibration_20261016_133648`・`134316` はラズパイの時計がずれていて、時刻を `stamp` のままでは再生できない（`141128` 以降は正常）。

MATLAB が `/tf` から引く値（`/tf` の周期、点群の stamp より後の `/tf` が届くまでの時間、点群の stamp の時刻の `/tf`（補間）に取付を掛けた
LiDAR の姿勢と NDT の解の差）を確かめる。`compare` でラベルごとの平均と幅の表と分布の図。`--input <再生した入力の録画>` を付けると、
点群が届いた時刻に購読側が持っていた値と、TF が揃うまでの時間（`tf_wait_receipt`、待った割合 `waited`）を出す：

```bash
python3 plot/plot_matlab_outputs.py /root/ros2_ws/bags/gazebo/tune/<試行名> /root/ros2_ws/fig/matlab_outputs/<名前>.png
python3 plot/plot_matlab_outputs.py compare /root/ros2_ws/fig/matlab_outputs/compare.png \
  before=/root/ros2_ws/bags/gazebo/tune/a_1,/root/ros2_ws/bags/gazebo/tune/a_2 after=/root/ros2_ws/bags/gazebo/tune/b_1 [--input <入力の録画>]
```

RViz の点群の表示に使う姿勢の違い（同じ点群の時刻の、EKF の予測値（`/tf`）と NDT の解（`/ndt_pose`）の差。実機でもよい）：

```bash
python3 plot/plot_prior_posterior.py /root/ros2_ws/bags/real/blue_spin5_20261016_160557 /root/ros2_ws/fig/prior_posterior_blue_spin5.png
```

## MATLAB の確認（`matlab/`）

MATLAB（`MaeharaTakumi/whill_common_matlab` の `cr_debug` ブランチの CR_debug）が点群と `/tf` を受け取って姿勢を引けるかは、`replay_outputs.py` で
再生している間に、ホストで MATLAB を `-batch` で動かして確かめる（`ROS_DOMAIN_ID=99`、Fast DDS の localhost のプロファイル）：

```bash
REPO="<whill_common_matlab のフォルダ>" TEST_DURATION=70 TEST_OUT=/tmp/cr_debug.mat \
  matlab -batch "run('<localization_tools>/matlab/check_cr_debug.m')"                                        # ホスト
python3 plot/plot_cr_debug_pairs.py /root/ros2_ws/fig/matlab_outputs/cr_debug.mat /root/ros2_ws/fig/matlab_outputs/cr_debug.png   # コンテナ（.mat を写して）
```

main.m と同じ流れ（Estimate2・Control2・DataLogger。指令は配信しない）で記録し、図と動画（`TEST_OUT_DIR/result`。地図は `MAP`）まで作るのは
`matlab/check_cr_debug_plot.m`。録画の再生・MATLAB・図と動画を 1 回で行うのは `matlab/replay_matlab.sh`（ホストで実行。MATLAB を先に起動して
購読ができてから `RECORD=0` の `replay_outputs.py` で再生し、新しい組が `STOP_IDLE`（既定 5）秒できなくなったら止まって図と動画を作る）：

```bash
matlab/replay_matlab.sh /root/ros2_ws/bags/gazebo/gazebo_spin5_20261016_111637 ../../results/matlab_outputs/replay_gazebo_spin5
matlab/replay_matlab.sh /root/ros2_ws/bags/real/blue_spin5_20261016_160557 ../../results/matlab_outputs/replay_blue_spin5 blue /root/ros2_ws/map/10build_4F_C_20260414.pcd
```

## 録画の情報（`eval/bag_info.py`）

Gazebo か実機か、録画日時、長さ、走らせ方、用途、録画したときの推定器、Gazebo のオドメトリの移動平均（車輪の角速度から判定）と、
オドメトリの omega / 車輪から計算した omega。`drive/run_test.sh` が録画のあとに自動で書く（用途は環境変数 `PURPOSE`）。

```bash
python3 eval/bag_info.py list [--source gazebo|real]   # 録画の一覧（コマンド bags_list）
python3 eval/bag_info.py write <bag> --source gazebo|real --drive '...' --purpose '...' [--notes '...']
```

## RViz の画面の録画（`record_rviz.sh`、ホスト）

コンテナの RViz はホストの X に描いているので、ホストの ffmpeg で RViz の窓を探して録画する（X11 のセッションだけ。
ホストに `ffmpeg`・`xdotool`・`xwininfo`（x11-utils）が要る）。

```bash
./record_rviz.sh                          # 開いている RViz の窓を Ctrl-C まで録る（./rviz_<日時>.mp4）
./record_rviz.sh -t 60 -o ~/Videos/run.mp4  # 60 s だけ、出力を指定
./record_rviz.sh --new                    # この後に開く RViz の窓を待って録る（起動の直前に実行しておく）
./record_rviz.sh --list                   # RViz の窓の一覧（ID・大きさ・題名）
./record_rviz.sh --select                 # 録る窓をクリックで選ぶ
./record_rviz.sh --id <ID>                # 窓の ID で選ぶ
```

- 窓を指定しなければ、RViz の窓が 1 つならそれ、複数なら前面の窓が RViz ならそれを録る（そうでなければ一覧を出して止まる）。
- 窓の中身だけを録る（x11grab の `-window_id`）。録画中に窓の大きさを変えると止まる。他の窓が上に重なるとその部分も写る。
- 出力は断片化した MP4（15 fps、H.264）なので、途中で止まっても再生できる。1 分で約 4 MB。

## 図

描いた図は `Docker/localization/results/`、作業の記録は `Docker/localization/dev_log.md`（図を埋め込む）。図のルール：後方の線ほど太く、
点線は使わない、題・軸ラベル・凡例の頭文字は大文字（保存前に `plot_style.capitalize_labels`）、枠は 4 辺とも黒（色・枠・書式は `common/plot_style.py`）。
