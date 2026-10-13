#!/bin/bash
# テスト走行と録画（ホストで実行）。自己位置推定のコンテナ（localizer_rl）で録画し、drive/drive_test.py で走らせる
#
#   drive/run_test.sh gazebo <動き> <bag 名> [estimator] [launch の追加引数（例 use_ekf:=false）]
#     Gazebo を起動し直して車椅子を初期位置に戻し、推定器（既定 ekf_localizer）を起動して走らせる。
#     録画は localizer_rl:/root/ros2_ws/bags/gazebo/<bag 名>（同じ名前があれば上書き）
#   drive/run_test.sh real <動き> <bag 名>
#     実機。自己位置推定（/ekf_pose）・whill_driver・LiDAR が動いていること（推定の起動は自分で行う。地図・初期姿勢が要るため）。
#     走り出す前に計画を表示して確認するので、端末から実行する。録画は localizer_rl:/root/ros2_ws/bags/real/<bag 名>（同じ名前は止める）
#
#   <動き> は drive/profiles.yaml（spin・stops・accel・curve・slalom・reverse。drive/drive_test.py <target> --list）。
#   速さの上限・走ってよい範囲・トピックは drive/config/<target>.yaml。
#   環境変数：PURPOSE（録画の用途。<bag 名>_info.yaml に書く）、CONFIG（設定の yaml のコンテナ内のパス。既定は target の設定）、
#             WHILL（実機の車椅子の色 blue・red・white。既定は設定の whill。自己位置推定の launch の whill:=<色> と合わせる）
#   録画の隣に <bag 名>_phases.txt（動作の開始時刻）と <bag 名>_info.yaml（eval/bag_info.py）を書く。
set -u
target=${1:?gazebo か real}; profile=${2:?動き}; bag=${3:?bag 名}
here=$(cd "$(dirname "$0")" && pwd)
root=$(dirname "$here")   # localization_tools
LOC=localizer_rl; GZ=20261002
T=/root/localization_tools
CFG=${CONFIG:-$target}
DT="python3 $T/drive/drive_test.py $CFG ${WHILL:+--whill $WHILL}"
BAGS=/root/ros2_ws/bags/$target
case $target in gazebo|real) ;; *) echo "target は gazebo か real"; exit 1;; esac

dexec() { docker exec $LOC bash -ic "$1" 2>&1 | grep -v "terminal process group\|no job control"; }
count_loc() {
  docker exec $LOC bash -c 'ps -eo cmd | grep -E "ekf_node|ekf_localizer_node|robot_localization_adapter|static_transform|ros2 launch|pcl_localization|ros2 bag|rviz2" | grep -v grep | wc -l'
}

# localization_tools 一式をコンテナの /root/localization_tools に置く（走行・描画・評価もそこで行う）
"$root/sync.sh" push
dexec "$DT $profile --simulate" || { echo "動き $profile が範囲に収まらない・設定が読めない"; exit 1; }
topics=$(dexec "$DT --record-topics" | tail -1)

if [ "$target" = gazebo ]; then
  est=${4:-ekf_localizer}; extra=${5:-}
  echo "== restart Gazebo"
  docker exec $GZ bash -c 'pkill -INT -f "ros2 launch connection_wheelchair"'
  for _ in $(seq 30); do
    [ "$(docker exec $GZ bash -c 'pgrep -c -x gzserver; pgrep -c -x gzclient' | paste -sd+ | bc)" = 0 ] && break
    sleep 1
  done
  docker exec $GZ pkill -KILL -x gzserver 2>/dev/null; docker exec $GZ pkill -KILL -x gzclient 2>/dev/null
  docker exec -d $GZ bash -ic 'ros2 launch connection_wheelchair connection_wheelchair.launch.py > /tmp/claude_gazebo_launch.log 2>&1'
  for _ in $(seq 60); do
    docker exec $LOC bash -ic 'timeout 3 ros2 topic echo --once /wheelchair/odom > /dev/null 2>&1' 2>/dev/null && break
    sleep 2
  done
  sleep 5
  echo "== start recording and $est"
  [ "$(count_loc)" = 0 ] || { echo "localization processes are still running in $LOC"; exit 1; }
  docker exec $LOC bash -c "mkdir -p $BAGS && rm -rf $BAGS/$bag $BAGS/${bag}_phases.txt $BAGS/${bag}_info.yaml"
  docker exec -d $LOC bash -ic "ros2 bag record -o $BAGS/$bag $topics > /tmp/record_$bag.log 2>&1"
  sleep 3
  docker exec -d $LOC bash -ic "ros2 launch localization_bringup localization.launch.py env:=gazebo estimator:=$est rviz:=false $extra > /tmp/launch_$bag.log 2>&1"
  sleep 25   # 地図の読み込みと NDT の収束を待つ
  echo "== drive"
  docker exec $LOC bash -ic "$DT $profile --yes 2>&1 | tee /tmp/drive_$bag.log; exit \${PIPESTATUS[0]}" 2>&1 \
    | grep -v "terminal process group\|no job control"
  status=${PIPESTATUS[0]}
else
  echo "== check the real robot"
  if docker exec $LOC test -e $BAGS/$bag; then echo "$BAGS/$bag は既にある（別の名前にする）"; exit 1; fi
  dexec "timeout 5 ros2 topic echo --once /ekf_pose > /dev/null" || { echo "/ekf_pose が来ない（自己位置推定を起動し、初期姿勢を合わせる）"; exit 1; }
  cmd=$(dexec "$DT --command-topic" | tail -1)
  subs=$(dexec "ros2 topic info $cmd" | sed -n 's/Subscription count: //p')
  if [ "${subs:-0}" -lt 1 ]; then
    echo "$cmd を受けるノード（whill_driver）が無い。車椅子の色（WHILL=blue|red|white）を確かめる。見つかった車椅子："
    dexec "ros2 topic list" | grep "whill_node/cmd_vel" | sed 's/^/  /'
    exit 1
  fi
  for t in $topics; do
    dexec "timeout 3 ros2 topic info $t > /dev/null" > /dev/null || echo "  （まだ無いトピック：$t。録画は続ける）"
  done
  echo "== start recording"
  docker exec $LOC bash -c "mkdir -p $BAGS"
  docker exec -d $LOC bash -ic "ros2 bag record -o $BAGS/$bag $topics > /tmp/record_$bag.log 2>&1"
  sleep 3
  echo "== drive（計画を確かめて yes で走り出す。止めるときは Ctrl-C かジョイスティック）"
  docker exec -it $LOC bash -ic "$DT $profile 2>&1 | tee /tmp/drive_$bag.log; exit \${PIPESTATUS[0]}"
  status=$?
fi

docker exec $LOC bash -c "cp /tmp/drive_phases.txt $BAGS/${bag}_phases.txt"
sleep 3
echo "== stop"
docker exec $LOC bash -c "pkill -INT -f 'ros2 bag record -o $BAGS/$bag'"
if [ "$target" = gazebo ]; then
  docker exec $LOC bash -c 'pkill -INT -f "ros2 launch localization_bringup"'
  sleep 8
  echo "remaining localization processes: $(count_loc)"
  docker exec $LOC bash -c "grep -h \"mount base_footprint\" /tmp/launch_$bag.log | cut -c1-170"
else
  sleep 5
fi
notes=$(dexec "$DT $profile --describe" | tail -1)
[ "$status" = 0 ] || notes="走行を途中で止めた（drive_test.py の終了コード $status、ログ localizer_rl:/tmp/drive_$bag.log）。$notes"
dexec "python3 $T/eval/bag_info.py write $BAGS/$bag --source $target --drive 'drive/drive_test.py $profile' --purpose '${PURPOSE:-}' --notes '$notes'"
exit $status
