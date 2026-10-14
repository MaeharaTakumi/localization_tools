#!/bin/bash
# results の図をすべて描き直す（test_maehara 内で実行。図は /root/ros2_ws/fig に出る。ホストで sync.sh pull すると workspace/results へ）
# 真値なしの R・取付と倍率の推定は録画から計算し直す（数分）。NDT の R（ndt_init_stops、NDT を計算し直すので約 9 分）は FULL=1 のときだけ
set -u
T0=/root/ros2_ws/localization_tools
cd $T0/plot
B=/root/ros2_ws/bags/gazebo; T=$B/tune; O=/root/ros2_ws/fig; R=$T0/eval/results
mkdir -p $O
run() { echo "== $*" | cut -c1-120; "$@" 2>&1 | grep -E "saved|Error|Traceback|line [0-9]" ; }
run python3 plot_spin.py $O/spin_results.png "EKF (ekf_localizer)=$B/spin_ekf_localizer" "EKF (robot_localization)=$B/spin_robot_localization"
run python3 plot_spin.py $O/spin360_results.png "EKF (ekf_localizer)=$B/spin360_ekf" "EKF (robot_localization, before tuning)=$B/spin360_rl_base" "EKF (robot_localization, tuned)=$B/spin360_rl_tuned"
PLOT_COLORS="#2a78d6,#1baf7a,#eda100" run python3 plot_spin.py $O/spin360_ndt_compare.png "EKF (ekf_localizer)=$B/spin360_ekf" "EKF (robot_localization, tuned)=$B/spin360_rl_tuned" "NDT only=$B/spin360_ndt_only@/ndt_pose"
run python3 plot_axes.py $O/axes "EKF (ekf_localizer)=$T/axes_ekf@/ekf_pose" "EKF (robot_localization, tuned)=$T/axes_rl@/ekf_pose" "NDT only=$T/axes_ndt@/ndt_pose"
PLOT_COLORS="#2a78d6,#1baf7a,#eda100" run python3 plot_spin.py $O/transition_compare.png "EKF (ekf_localizer, tuned)=$T/final_ekf" "EKF (robot_localization, tuned)=$T/final_rl" "NDT only=$T/v_ndt_only@/ndt_pose"
run python3 plot_velocity.py $O/velocity $B/spin360_ekf 0.09 "EKF (ekf_localizer):$T/v_ekf_before_1:$T/final_ekf:/ekf_odom" "EKF (robot_localization):$T/v_rl_before_1:$T/final_rl:/odometry/global"
run python3 plot_scale.py $O/scale $R/results_valid.jsonl $R/results_scale.jsonl $R/results_mult.jsonl
ODOM_SCALE="1,0.922" LABEL_BEFORE="estimate (tuned, no odom_scale)" LABEL_AFTER="estimate (tuned + odom_scale omega 0.922)" run python3 plot_velocity.py $O/velocity_scaled $B/spin360_ekf 0.09 "EKF (ekf_localizer):$T/final_ekf:$T/scale_ekf_1:/ekf_odom" "EKF (robot_localization):$T/final_rl:$T/scale_rl_1:/odometry/global"
run python3 check_omega_slope.py $O/omega_slope.png $B/spin360_js_ekf
run python3 plot_odom_filter.py $B/spin360_w1_ekf $O
run python3 plot_ekf_compare.py $O/ekf_w1 $R/results_w1cmp.jsonl
run python3 plot_motion_compare.py report $O/compare $R/results_compare.jsonl
run python3 plot_motion_compare.py candidates $O/alpha_spin.png $R/results_alpha.jsonl "Odom_noise_alpha for ekf_localizer: spin in place (left, right, forward), 4 runs each" "ekf_a0=Alpha 0 (current)" "ekf_w0.005=Omega² to R_omega 0.005" "ekf_w0.05=Omega² to R_omega 0.05" "ekf_v0.005=V² to R_v 0.005" "ekf_vw0.005=Both 0.005"
run python3 plot_motion_compare.py candidate-series $O/alpha_spin $R/results_alpha.jsonl spin "Odom_noise_alpha for ekf_localizer, spin in place" "ekf_a0=Alpha 0 (current)" "ekf_w0.005=Omega² to R_omega 0.005" "ekf_w0.05=Omega² to R_omega 0.05" "ekf_v0.005=V² to R_v 0.005" "ekf_vw0.005=Both 0.005"
run python3 plot_odom_r.py $O/odom_r.png $B/drive_accel $B/drive_curve $B/drive_slalom $B/drive_reverse $B/spin360_w1_ekf2 $B/stops_ekf
run python3 plot_motion_compare.py candidates $O/alpha_r0_spin.png $R/results_alpha.jsonl "R_odom near 0 with odom_noise_alpha, ekf_localizer: spin in place, 4 runs each" "ekf_a0_b=Current: R_odom default, alpha 0" "ekf_r0_a0=R_odom 1e-6, alpha 0" "ekf_r0_w0.005=R_odom 1e-6, omega² 0.005" "ekf_r0_w0.05=R_odom 1e-6, omega² 0.05" "ekf_r0_v0.005=R_odom 1e-6, v² 0.005" "ekf_r0_vw0.005=R_odom 1e-6, both 0.005"
run python3 plot_motion_compare.py candidate-series $O/alpha_r0_spin $R/results_alpha.jsonl spin "R_odom near 0 with odom_noise_alpha, spin in place" "ekf_a0_b=Current" "ekf_r0_a0=R_odom 1e-6, alpha 0" "ekf_r0_w0.05=R_odom 1e-6, omega² 0.05" "ekf_r0_v0.005=R_odom 1e-6, v² 0.005" "ekf_r0_vw0.005=R_odom 1e-6, both 0.005"
run python3 plot_drive_paths.py $O/drive_paths_gazebo.png gazebo spin=$B/test_spin stops=$B/test_stops accel=$B/test_accel curve=$B/test_curve slalom=$B/test_slalom reverse=$B/test_reverse
# ekf_localizer の再生の比較（条件は configs/*.json。dev_log.md の 14・18〜21）
for c in qr_ekf qi_ekf qg legacy rk2 receipt bobs; do run python3 plot_ekf_compare.py $O/$c $R/results_qrv.jsonl configs/$c.json; done
PLOT_COLORS="#2a78d6,#eb6834" run python3 plot_spin.py $O/qr_unstable.png "EKF, R from data + previous Q, replay 1=$T/qe_ekf_oldQ_1" "EKF, R from data + previous Q, replay 2 (same settings)=$T/qe_ekf_oldQ_2"
# 真値なしの R（dev_log.md の 13）、取付位置とオドメトリの倍率（16）
for b in spin360_w1_ekf spin360_w1_ekf2; do run python3 ../noise/estimate_noise.py $B/$b --out $O/noise_$b --truth; done
run python3 ../calib/estimate_calibration.py $B/spin360_w1_ekf $B/spin360_w1_ekf2 $B/stops_ekf --out $O/calib --truth --map /root/ros2_ws/map/SC10_4_0.05.pcd
# run_test.sh の確認（33）
run python3 plot_drive_paths.py $O/runner_paths.png gazebo "From the host (spin)=$B/check/runner_host_spin" "Inside the container (spin)=$B/check/runner_inside_spin" "Inside, real-robot flow (real limits)=$B/check/runner_inside_realflow"
# NDT の R を「真値 + ノイズ」の初期値で求める（15）
[ "${FULL:-0}" = 1 ] && run python3 ../noise/ndt_init_noise.py $B/stops_ekf --out $O/ndt_init_stops
ls $O
