#!/bin/bash
# results の図をすべて描き直す（localizer_rl 内で実行。図は /root/fig に出る。ホストで sync.sh pull すると workspace/results へ）
set -u
T0=/root/localization_tools
cd $T0/plot
B=/root/bags; T=/root/bags/tune; O=/root/fig; R=$T0/eval/results
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
ls $O
