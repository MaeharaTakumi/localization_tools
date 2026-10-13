#!/bin/bash
# Gazebo を起動し直して（車椅子を初期位置に戻す）、指定の推定器で drive_spin.py を走らせ、bag に録る（ホストで実行）
#   localization_tools/gazebo/run_gazebo_test.sh <estimator> <bag 名> [launch の追加引数（例: use_ekf:=false）]
#   bag は localizer_rl:/root/bags/gazebo/<bag 名>、動作の開始時刻は同じ場所の <bag 名>_phases.txt、
#   録画の情報（Gazebo・録画日時・オドメトリの移動平均・走らせ方・用途）は <bag 名>_info.yaml（eval/bag_info.py）
#   走り方は環境変数 DRIVE（既定 drive_spin.py。止まった状態を測るときは DRIVE=drive_stops.py）、用途は PURPOSE（任意）
set -u
est=$1; bag=$2; extra=${3:-}
here=$(cd "$(dirname "$0")" && pwd)
root=$(dirname "$here")   # localization_tools
LOC=localizer_rl; GZ=20261002
count_loc() {
  docker exec $LOC bash -c 'ps -eo cmd | grep -E "ekf_node|ekf_localizer_node|robot_localization_adapter|static_transform|ros2 launch|pcl_localization|ros2 bag|rviz2" | grep -v grep | wc -l'
}

echo "== restart Gazebo"
docker exec $GZ bash -c 'pkill -INT -f "ros2 launch connection_wheelchair"'
for _ in $(seq 30); do
  [ "$(docker exec $GZ bash -c 'pgrep -c -f "gzserver|gzclient|ros2 launch connection_wheelchair"')" = 0 ] && break
  sleep 1
done
docker exec $GZ bash -c 'pkill -KILL -f "gzserver|gzclient"' 2>/dev/null
docker exec -d $GZ bash -ic 'ros2 launch connection_wheelchair connection_wheelchair.launch.py > /tmp/claude_gazebo_launch.log 2>&1'
for _ in $(seq 60); do
  docker exec $LOC bash -ic 'timeout 3 ros2 topic echo --once /wheelchair/odom > /dev/null 2>&1' 2>/dev/null && break
  sleep 2
done
sleep 5

echo "== start recording and $est"
[ "$(count_loc)" = 0 ] || { echo "localization processes are still running in $LOC"; exit 1; }
# localization_tools 一式を自己位置推定のコンテナの /root/localization_tools に置く（描画・評価もそこで行う）
"$root/sync.sh" push
BAGS=/root/bags/gazebo
docker exec $LOC bash -c "mkdir -p $BAGS && rm -rf $BAGS/$bag $BAGS/${bag}_phases.txt $BAGS/${bag}_info.yaml"
docker exec -d $LOC bash -ic "ros2 bag record -o $BAGS/$bag /clock /velodyne_points /wheelchair/odom /wheelchair/joint_states /wheelchair/pose /wheelchair/diff_drive_controller/cmd_vel /tf_static /tf /localization/wheel_odom /localization/odom /ekf_pose /ekf_odom /odometry/global /odometry/local /ndt_pose /localization_debug > /tmp/record_$bag.log 2>&1"
sleep 3
docker exec -d $LOC bash -ic "ros2 launch localization_bringup localization.launch.py env:=gazebo estimator:=$est rviz:=false $extra > /tmp/launch_$bag.log 2>&1"
sleep 25   # 地図の読み込みと NDT の収束を待つ

echo "== drive"
docker exec $LOC bash -ic "python3 /root/localization_tools/gazebo/${DRIVE:-drive_spin.py}" 2>&1 | grep -v "terminal process group\|no job control"
docker exec $LOC bash -c "cp /tmp/drive_phases.txt $BAGS/${bag}_phases.txt"
sleep 5

echo "== stop"
docker exec $LOC bash -c 'pkill -INT -f "ros2 launch localization_bringup"; pkill -INT -f "ros2 bag record"'
sleep 8
echo "remaining localization processes: $(count_loc)"
docker exec $LOC bash -c "grep -h \"mount base_footprint\" /tmp/launch_$bag.log | cut -c1-170"
docker exec $LOC bash -ic "python3 /root/localization_tools/eval/bag_info.py write $BAGS/$bag --source gazebo --drive '${DRIVE:-drive_spin.py}' --purpose '${PURPOSE:-}'" 2>&1 | grep -v "terminal process group\|no job control"
