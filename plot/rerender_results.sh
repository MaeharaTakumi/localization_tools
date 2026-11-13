#!/bin/bash
# results の図を描き直す（コンテナ test 内で実行。図は /root/ros2_ws/fig に出る。ホストで sync.sh pull すると Docker/localization/results へ）
# 描き直すのは今のツールで描ける図だけ。真値と比べる評価・図のツール（2026-11-13 に廃止）・robot_localization（2026-10-15 に廃止）・
# 「真値 + ノイズ」の NDT の R（calibrate_ndt_noise.py、2026-11-13 に廃止）の図は描き直さない（results/ の図は開発ログの記録として残す）
# パラメータ推定は録画から計算し直す（数分）。
# 環境変数 OUT で図の出力先、T0 でツールの場所を変えられる（ツールを直したとき、別の場所に描いて前の図と比べる）
set -u
T0=${T0:-/root/ros2_ws/localization_tools}
cd $T0/plot
B=/root/ros2_ws/bags/gazebo; T=$B/tune; O=${OUT:-/root/ros2_ws/fig}
mkdir -p $O
run() { echo "== $*" | cut -c1-120; "$@" 2>&1 | grep -E "saved|Error|Traceback|line [0-9]" ; }
# テスト走行の軌跡（/tf の map -> base）
run python3 plot_drive_paths.py $O/drive_paths_gazebo.png gazebo spin=$B/test_spin stops=$B/test_stops accel=$B/test_accel curve=$B/test_curve slalom=$B/test_slalom reverse=$B/test_reverse
# run_test.sh の確認（33）
run python3 plot_drive_paths.py $O/runner_paths.png gazebo "From the host (spin)=$B/check/runner_host_spin" "Inside the container (spin)=$B/check/runner_inside_spin" "Inside, real-robot flow (real limits)=$B/check/runner_inside_realflow"
# 真値なしの R（dev_log.md の 13）、取付位置とオドメトリの倍率（16）
for b in spin360_w1_ekf spin360_w1_ekf2; do run python3 ../calibration/calibrate_noise.py $B/$b --out $O/noise_$b; done
run python3 ../calibration/calibrate_mount_scale.py $B/spin360_w1_ekf $B/spin360_w1_ekf2 $B/stops_ekf --out $O/calib --map /root/ros2_ws/map/SC10_4_0.05.pcd
# RViz の点群の表示：EKF の予測値・NDT の更新値・NDT の解の差（実機 blue、53・54。tune は再生の出力）
run python3 plot_prior_posterior.py /root/ros2_ws/bags/real/tune/ndt_update_tf_blue_spin5 $O/ndt_update_tf_blue_spin5.png
for b in blue_spin5_20261016_160557 blue_calibration_20261016_141128; do run python3 plot_prior_posterior.py /root/ros2_ws/bags/real/$b $O/prior_posterior_${b%%_2026*}.png; done
# MATLAB が /tf から引く値：publish_trigger（Gazebo は update、実機は cloud）と 50 Hz の周期の比較（68。tune は再生の出力、4 回ずつ）
tf4() { echo $1_1,$1_2,$1_3,$1_4; }
run python3 plot_matlab_outputs.py compare $O/matlab_outputs/tf_compare_gazebo_spin5.png "Gazebo before (update)=$(tf4 $T/tfcmp_before_gazebo)" "Gazebo after (50 Hz)=$(tf4 $T/tfcmp_after_gazebo)"
run python3 plot_matlab_outputs.py compare $O/matlab_outputs/tf_compare_blue_spin5.png "Blue before (cloud)=$(tf4 /root/ros2_ws/bags/real/tune/tfcmp_before_blue)" "Blue after (50 Hz)=$(tf4 /root/ros2_ws/bags/real/tune/tfcmp_after_blue)"
# パラメータ推定用の動き（当時の calib）で走った録画から、R・倍率・取付をまとめて推定する（46）
run python3 ../calibration/calibrate.py gazebo $B/gazebo_calib_test --out $O/calib_gazebo
# 推定用の動き（4 方向に 12 s ずつ止まる・片道。47）
run python3 ../calibration/calibrate.py gazebo $B/gazebo_calib --out $O/calib_gazebo2
# 推定用の動き（止まる → 左に 4 周 → 片道。48）
run python3 ../calibration/calibrate.py gazebo $B/gazebo_calibration --out $O/calibration_gazebo
# LiDAR を 5° 回した Gazebo での取付の yaw の試験（49）
run python3 ../calibration/calibrate.py gazebo $B/gazebo_calibration_yaw5 --out $O/calibration_gazebo_yaw5
ls $O
