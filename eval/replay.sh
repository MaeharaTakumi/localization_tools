#!/bin/bash
# バックグラウンド起動では SIGINT が無視されるので、既定に戻してから exec する
# 録画した Gazebo の入力を ROS_DOMAIN_ID=99 で再生し、指定の推定器の出力を録る
#   replay.sh <estimator> <出力 bag>
set -u
est=$1; out=$2
here=$(cd "$(dirname "$0")" && pwd)
export ROS_DOMAIN_ID=99
rm -rf "$out"
pkill -INT -f "ros2 launch localization_bringup"; pkill -INT -f "ros2 bag record"; sleep 2
setsid python3 -c "import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])" bash -ic "export ROS_DOMAIN_ID=99; exec ros2 launch localization_bringup localization.launch.py env:=gazebo estimator:=$est rviz:=false" > /tmp/replay_launch_$est.log 2>&1 < /dev/null &
setsid python3 -c "import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])" bash -ic "export ROS_DOMAIN_ID=99; exec ros2 bag record -o $out /clock /wheelchair/pose /ekf_pose /odometry/global /ekf_odom" > /tmp/replay_record_$est.log 2>&1 < /dev/null &
sleep 12
ros2 bag play /root/ros2_ws/bags/gazebo/live_rl --topics /clock /velodyne_points /wheelchair/odom /wheelchair/pose /tf_static \
  --qos-profile-overrides-path "$here/qos_tf_static.yaml" > /tmp/replay_play_$est.log 2>&1
sleep 2
pkill -INT -f "ros2 launch localization_bringup"; pkill -INT -f "ros2 bag record"; sleep 6
echo "remaining: $(ps -eo cmd | grep -E "ekf_node|ekf_localizer_node|robot_localization_adapter|static_transform|ros2 launch|pcl_localization|ros2 bag" | grep -v grep | wc -l)"
grep -i -E "error|died|Traceback" /tmp/replay_launch_$est.log | cut -c1-160 | head -5
