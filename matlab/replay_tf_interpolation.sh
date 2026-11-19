#!/bin/bash
# 録画を再生して ekf_localizer に /tf を出させ（コンテナ test、ROS_DOMAIN_ID=99）、ホストの MATLAB の ros2tf で時刻を指定して引いた TF が
# 前後の TF の線形補間（位置は線形、回転は slerp）になっているかを matlab/check_tf_interpolation_ekf.m で確かめる（ホストで実行。dev_log.md の 105）。
#   replay_tf_interpolation.sh <bag（コンテナのパス）> <出力のフォルダ（ホスト）>                         Gazebo の録画
#   replay_tf_interpolation.sh <bag> <出力のフォルダ> <環境の名前（red・blue・white）> <地図（コンテナのパス）>   実機の録画
# 出力のフォルダに tf_interpolation_ekf.png・.mat、matlab.log、replay.log を作る。
# 環境変数：REPO（whill_common_matlab のフォルダ）、MATLAB（既定 R2024b）、STOP_IDLE（既定 5）、RATE（再生の速さ）、PARAMS（推定器のパラメータの上書き）
set -eu
here=$(cd "$(dirname "$0")" && pwd)
ws=$(cd "$here/../../.." && pwd)    # localization_workspace（Docker/localization）
LOC=${LOC:-test}
REPO=${REPO:-$ws/src/whill_common_matlab-main _ndtDebug}
MATLAB=${MATLAB:-/usr/local/MATLAB/R2024b/bin/matlab}
STOP_IDLE=${STOP_IDLE:-5}
if [ $# -ne 2 ] && [ $# -ne 4 ]; then
  sed -n '2,7p' "$0"; exit 1
fi
bag=$1
out=$(mkdir -p "$2" && cd "$2" && pwd)
name=tfinterp_$(basename "$bag")
if [ $# -eq 4 ]; then
  replay_args="'$bag' '$name' '$3' '$4'"
else
  replay_args="'$bag' '$name'"
fi
duration=$(docker exec $LOC bash -ic "python3 -c \"import rosbag2_py; print(rosbag2_py.Info().read_metadata('$bag', 'sqlite3').duration.nanoseconds / 1e9)\"" 2>/dev/null | tail -1)
echo "bag: $bag (${duration} s) -> $out ($MATLAB)"

ready=$out/.matlab_ready
rm -f "$ready"
REPO="$REPO" TEST_DOMAIN=99 TEST_DURATION=$(python3 -c "print(int($duration + 90))") STOP_IDLE=$STOP_IDLE READY_FILE="$ready" \
  TEST_OUT="$out/tf_interpolation_ekf.mat" TEST_FIG="$out/tf_interpolation_ekf.png" \
  "$MATLAB" -batch "run('$here/check_tf_interpolation_ekf.m')" > "$out/matlab.log" 2>&1 &
matlab_pid=$!
for _ in $(seq 180); do
  [ -e "$ready" ] && break
  kill -0 $matlab_pid 2>/dev/null || { echo "MATLAB stopped before subscribing (see $out/matlab.log)"; exit 1; }
  sleep 1
done
[ -e "$ready" ] || { echo "MATLAB did not get ready (see $out/matlab.log)"; kill $matlab_pid; exit 1; }
echo "MATLAB is subscribing; replaying the bag"
docker exec -e PARAMS="${PARAMS:-}" $LOC bash -ic "cd /root/ros2_ws/localization_tools && RECORD=0 RATE=${RATE:-1.0} python3 eval/replay_outputs.py $replay_args" \
  > "$out/replay.log" 2>&1 || echo "replay failed (see $out/replay.log)"
echo "replay finished; waiting for MATLAB"
wait $matlab_pid || { echo "MATLAB failed (see $out/matlab.log)"; exit 1; }
rm -f "$ready"
sed -n '/^ekf_localizer の TF/,$p' "$out/matlab.log"
