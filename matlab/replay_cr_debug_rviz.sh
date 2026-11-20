#!/bin/bash
# 1 回の再生（コンテナ test、ROS_DOMAIN_ID=99）で、ホストの MATLAB の CR_debug（matlab/check_cr_debug.m）と、フックを入れた rviz2
# （rviz/replay_rviz_tf.sh）を同時に動かし、点群の stamp で引いた TF を点群ごとに比べる（rviz/compare_rviz_cr_debug.py。ホストで実行。dev_log.md の 107）。
#   replay_cr_debug_rviz.sh <bag（コンテナのパス）> <名前>                                    Gazebo の録画
#   replay_cr_debug_rviz.sh <bag> <名前> <環境の名前（red・blue・white）> <地図（コンテナのパス）>   実機の録画
# 出力はコンテナの /root/ros2_ws/fig/rviz_cr_debug/<名前>（ホストの docker/data/fig/rviz_cr_debug/<名前>）：cr_debug_poses.txt・
# cr_debug.mat・matlab.log（MATLAB）、rviz_tf.log・rviz.log・replay.log・summary.txt・rviz_tf_interpolation.png（rviz2）、
# compare.txt・rviz_cr_debug.png（比べた結果）。
# 環境変数：REPO（whill_common_matlab のフォルダ）、MATLAB（既定 R2024b）、STOP_IDLE（既定 5）、RATE（再生の速さ）、PARAMS（推定器のパラメータの上書き）
set -eu
here=$(cd "$(dirname "$0")" && pwd)
ws=$(cd "$here/../../.." && pwd)    # localization_workspace（Docker/localization）
LOC=${LOC:-test}
REPO=${REPO:-$ws/src/whill_common_matlab-main _ndtDebug}
MATLAB=${MATLAB:-/usr/local/MATLAB/R2024b/bin/matlab}
STOP_IDLE=${STOP_IDLE:-5}
if [ $# -ne 2 ] && [ $# -ne 4 ]; then
  sed -n '2,10p' "$0"; exit 1
fi
bag=$1
name=$2
case $name in */*|'') echo "the name must not contain /"; exit 1;; esac
out=$ws/docker/data/fig/rviz_cr_debug/$name      # /root/ros2_ws/fig は docker/data/fig を割り当てたもの
cout=/root/ros2_ws/fig/rviz_cr_debug/$name
mkdir -p "$out"
if [ $# -eq 4 ]; then
  mode=3; color=$3; replay_args="'$bag' '$cout' '$3' '$4'"
else
  mode=2; color=red; replay_args="'$bag' '$cout'"
fi
duration=$(docker exec $LOC bash -ic "python3 -c \"import rosbag2_py; print(rosbag2_py.Info().read_metadata('$bag', 'sqlite3').duration.nanoseconds / 1e9)\"" 2>/dev/null | tail -1)
echo "bag: $bag (${duration} s) -> $out ($MATLAB)"

ready=$out/.matlab_ready
rm -f "$ready" "$out/cr_debug_poses.txt"
# MATLAB：rviz2 の起動（5 s）と再生の長さ + 余裕まで（新しい点群が来なくなれば STOP_IDLE で先に止まる）
REPO="$REPO" MODE=$mode COLOR=$color SIM_TIME=1 STOP_IDLE=$STOP_IDLE READY_FILE="$ready" \
  TEST_DURATION=$(python3 -c "print(int($duration + 120))") TEST_OUT="$out/cr_debug.mat" TEST_POSES="$out/cr_debug_poses.txt" \
  "$MATLAB" -batch "run('$here/check_cr_debug.m')" > "$out/matlab.log" 2>&1 &
matlab_pid=$!
for _ in $(seq 180); do
  [ -e "$ready" ] && break
  kill -0 $matlab_pid 2>/dev/null || { echo "MATLAB stopped before subscribing (see $out/matlab.log)"; exit 1; }
  sleep 1
done
[ -e "$ready" ] || { echo "MATLAB did not get ready (see $out/matlab.log)"; kill $matlab_pid; exit 1; }
echo "MATLAB is subscribing; starting rviz2 and replaying the bag"
docker exec -e PARAMS="${PARAMS:-}" -e RATE="${RATE:-1.0}" $LOC bash -ic \
  "cd /root/ros2_ws/localization_tools && rviz/replay_rviz_tf.sh $replay_args" > "$out/rviz_run.log" 2>&1 \
  || echo "rviz2 or the replay failed (see $out/rviz_run.log)"
echo "replay finished; waiting for MATLAB"
wait $matlab_pid || { echo "MATLAB failed (see $out/matlab.log)"; exit 1; }
rm -f "$ready"
grep -E "^cycles|^new point|^cycles with a point|^cycles using" "$out/matlab.log" || true
docker exec $LOC bash -ic "cd /root/ros2_ws/localization_tools && python3 rviz/compare_rviz_cr_debug.py '$cout'" 2>&1 | tee "$out/compare.txt"
