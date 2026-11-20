#!/bin/bash
# 録画を再生して ekf_localizer に /tf を出させ（ROS_DOMAIN_ID=99）、rviz2（自己位置推定と同じ設定）の tf2 に入った TF と、rviz2 が
# 引いた TF を rviz/tf_hook.cpp（LD_PRELOAD）で記録して、rviz/check_rviz_tf_interpolation.py で tf2 の補間の決まりと比べる
# （コンテナ test の中で実行。rviz2 の画面が開く。dev_log.md の 106）。
#   rviz/replay_rviz_tf.sh <bag> <出力のフォルダ>                     Gazebo の録画
#   rviz/replay_rviz_tf.sh <bag> <出力のフォルダ> <env> <地図>        実機の録画（env は red・blue・white）
# 出力のフォルダに rviz_tf.log（フックの記録）・rviz_tf_interpolation.png・summary.txt・rviz.log・replay.log を作る。
# 環境変数：RVIZ_CONFIG（既定 pcl_localization_ros2 の rviz/localization2.rviz）、LIDAR_FRAME（既定 velodyne）、RATE、PARAMS
set -eu
here=$(cd "$(dirname "$0")" && pwd)
tools=$(cd "$here/.." && pwd)
if [ $# -ne 2 ] && [ $# -ne 4 ]; then
  sed -n '2,8p' "$0"; exit 1
fi
bag=$1
out=$(mkdir -p "$2" && cd "$2" && pwd)
name=rviztf_$(basename "$bag")
config=${RVIZ_CONFIG:-$(ros2 pkg prefix pcl_localization_ros2)/share/pcl_localization_ros2/rviz/localization2.rviz}

hook=/tmp/rviz_tf_hook.so
includes=()
for d in /opt/ros/humble/include/*/; do includes+=("-I$d"); done
g++ -std=c++17 -O2 -shared -fPIC -Wall "${includes[@]}" -o "$hook" "$here/tf_hook.cpp" -ldl

echo "bag: $bag -> $out (rviz2 -d $config)"
rm -f "$out/rviz_tf.log"
ROS_DOMAIN_ID=99 LD_PRELOAD=$hook TF_HOOK_LOG=$out/rviz_tf.log \
  /opt/ros/humble/lib/rviz2/rviz2 -d "$config" --ros-args -p use_sim_time:=true > "$out/rviz.log" 2>&1 &
rviz_pid=$!
stop_rviz() {
  # rviz2 は SIGINT で終わらないことがあるので SIGKILL（記録は 1 行ごとに書き出してある）
  kill -KILL $rviz_pid 2>/dev/null || true
  wait $rviz_pid 2>/dev/null || true
}
trap stop_rviz EXIT
sleep 5
kill -0 $rviz_pid 2>/dev/null || { echo "rviz2 stopped (see $out/rviz.log)"; exit 1; }
echo "rviz2 is running; replaying the bag"
RECORD=0 RATE=${RATE:-1.0} python3 "$tools/eval/replay_outputs.py" "$bag" "$name" "${@:3}" > "$out/replay.log" 2>&1 \
  || echo "replay failed (see $out/replay.log)"
stop_rviz
trap - EXIT
python3 "$here/check_rviz_tf_interpolation.py" "$out/rviz_tf.log" "$out/rviz_tf_interpolation.png" "${LIDAR_FRAME:-velodyne}" \
  | tee "$out/summary.txt"
