#!/bin/bash
# 録画を再生して自己位置推定を動かし（コンテナ test、ROS_DOMAIN_ID=99）、ホストの MATLAB で CR_debug を main.m と同じ流れで
# 動かして（指令は配信しない）、記録・図・動画を作る（ホストで実行。dev_log.md の 65）。
#   replay_matlab.sh <bag（コンテナのパス）> <出力のフォルダ（ホスト）>                         Gazebo の録画
#   replay_matlab.sh <bag> <出力のフォルダ> <環境の名前（red・blue・white）> <地図（コンテナのパス）>   実機の録画
# 流れ：MATLAB を起動し、購読ができたら（READY_FILE）録画を再生する。新しい組が STOP_IDLE 秒来なくなったら MATLAB が止まり、
# DataPlotter・myplotResult・myplotPointCloudVideo で図と動画を作る（出力のフォルダの result）。
# 環境変数：REPO（whill_common_matlab のフォルダ）、MATLAB（matlab の実行ファイル）、MAP（動画の地図。ホストのパス）、
#   STOP_IDLE（既定 5）、RATE（再生の速さ。実機の録画だけ）
# 実機の録画をコンテナの中で再生すると、大きい点群が受け取る側で落ちることがある（dev_log.md の 61）。
set -eu
here=$(cd "$(dirname "$0")" && pwd)
ws=$(cd "$here/../../.." && pwd)    # localization_workspace（Docker/localization）
LOC=${LOC:-test}
REPO=${REPO:-$ws/src/whill_common_matlab-main _ndtDebug}
MATLAB=${MATLAB:-/usr/local/MATLAB/R2026a/bin/matlab}
STOP_IDLE=${STOP_IDLE:-5}
if [ $# -ne 2 ] && [ $# -ne 4 ]; then
  sed -n '2,10p' "$0"; exit 1
fi
bag=$1
out=$(mkdir -p "$2" && cd "$2" && pwd)
name=matlab_$(basename "$bag")
if [ $# -eq 4 ]; then
  env_name=$3; map=$4
  mode=3; color=$env_name; sim_time=1
  default_map=$ws/docker/data/map/$(basename "$map")   # /root/ros2_ws/map は docker/data/map を割り当てたもの
  replay_args="'$bag' '$name' '$env_name' '$map'"
else
  mode=2; color=red; sim_time=1
  default_map=$ws/docker/data/map/SC10_4_0.05.pcd
  replay_args="'$bag' '$name'"
fi
MAP=${MAP:-$default_map}
duration=$(docker exec $LOC bash -ic "python3 -c \"import rosbag2_py; print(rosbag2_py.Info().read_metadata('$bag', 'sqlite3').duration.nanoseconds / 1e9)\"" 2>/dev/null | tail -1)
echo "bag: $bag (${duration} s) -> $out (MATLAB mode $mode, map $MAP)"

ready=$out/.matlab_ready
rm -f "$ready"
# MATLAB：再生の長さ + 地図の読み込みと起動の余裕まで（新しい組が来なくなれば STOP_IDLE で先に止まる）
REPO="$REPO" TEST_OUT_DIR="$out" MAP="$MAP" MODE=$mode COLOR=$color SIM_TIME=$sim_time STOP_IDLE=$STOP_IDLE \
  TEST_DURATION=$(python3 -c "print(int($duration + 90))") READY_FILE="$ready" \
  "$MATLAB" -batch "run('$here/check_cr_debug_plot.m')" > "$out/matlab.log" 2>&1 &
matlab_pid=$!
for _ in $(seq 180); do
  [ -e "$ready" ] && break
  kill -0 $matlab_pid 2>/dev/null || { echo "MATLAB stopped before subscribing (see $out/matlab.log)"; exit 1; }
  sleep 1
done
[ -e "$ready" ] || { echo "MATLAB did not get ready (see $out/matlab.log)"; kill $matlab_pid; exit 1; }
echo "MATLAB is subscribing; replaying the bag"
docker exec $LOC bash -ic "cd /root/ros2_ws/localization_tools && RECORD=0 RATE=${RATE:-1.0} python3 eval/replay_outputs.py $replay_args" \
  > "$out/replay.log" 2>&1 || echo "replay failed (see $out/replay.log)"
echo "replay finished; waiting for MATLAB (plots and video)"
wait $matlab_pid || { echo "MATLAB failed (see $out/matlab.log)"; exit 1; }
rm -f "$ready"
grep -E "cycles:|Video saved" "$out/matlab.log" || true
echo "figures: $out/result"
