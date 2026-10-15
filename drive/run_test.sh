#!/bin/bash
# テスト走行と録画。drive/drive_test.py で走らせ、自己位置推定のコンテナの中で録画する
#
# どこで実行してもよい：
#   - ホスト（docker がある）：コンテナ test（環境変数 LOC で変えられる）に docker exec で入って動かす。
#     始めに sync.sh push でホストの localization_tools をコンテナの /root/ros2_ws/localization_tools に写す
#   - 自己位置推定のコンテナの中（docker が無い）：その場で直接動かす。このスクリプトのある localization_tools を使う
#
#   drive/run_test.sh gazebo <動き> <bag 名> [launch の追加引数（例 use_ekf:=false）]
#     自己位置推定（NDT + ekf_localizer）を起動して走らせる。ホストからなら Gazebo を起動し直して車椅子を初期位置に戻す
#     （コンテナの中からは Gazebo を起動し直せないので、Gazebo が動いていて車椅子が初期位置にあること）。
#     録画は /root/ros2_ws/bags/gazebo/<bag 名>（同じ名前があれば上書き）
#   drive/run_test.sh real <動き> <bag 名>
#     実機。自己位置推定（/ekf_pose）・whill_driver・LiDAR が動いていること（推定の起動は自分で行う。地図・初期姿勢が要るため）。
#     走り出す前に計画を表示して確認するので、端末から実行する。録画は /root/ros2_ws/bags/real/<bag 名>（同じ名前は止める）
#
#   <bag 名> を check/<名前> にすると bags/<target>/check/ に録る（ツールの動作確認など、比較・調整に使わない録画）。
#   <動き> は drive/profiles.yaml（spin・spin5・stops・accel・curve・slalom・reverse。drive/drive_test.py <target> --list）。
#   速さの上限・走ってよい範囲・トピックは drive/config/<target>.yaml。
#   環境変数：PURPOSE（録画の用途。<bag 名>_info.yaml に書く）、CONFIG（設定の yaml のコンテナ内のパス。既定は target の設定）、
#             WHILL（実機の車椅子の色 blue・red・white。既定は設定の whill。自己位置推定の launch の whill:=<色> と合わせる）
#   録画の隣に <bag 名>_phases.txt（動作の開始時刻）、<bag 名>_drive.log（drive_test.py の出力）、<bag 名>_info.yaml（eval/bag_info.py）を書く。
set -u
# ros2 topic の CLI は --no-daemon で使う。timeout の中で最初の ros2 が CLI のデーモンを起動すると、デーモンが timeout の
# プロセスグループに入り、時間切れのシグナルで壊れる（以後 ros2 topic が「!rclpy.ok()」で落ちる。2026-10-14 に起きた）
# --no-daemon のときトピックを探す時間は既定 0.5 s で、見つけ損ねると待たずに失敗する（10 回に 1 回）ので --spin-time 3 にする
target=${1:?gazebo か real}; profile=${2:?動き}; bag=${3:?bag 名}
tag=${bag//\//_}   # /tmp のログの名前（bag 名に check/ などの下のディレクトリを書いてもよい）
case $target in gazebo|real) ;; *) echo "target は gazebo か real"; exit 1;; esac
here=$(cd "$(dirname "$0")" && pwd)
root=$(dirname "$here")   # localization_tools
LOC=${LOC:-test}; GZ=20261002
if command -v docker > /dev/null 2>&1; then
  INSIDE=0; T=/root/ros2_ws/localization_tools; WHERE="$LOC:"
else
  INSIDE=1; T=$root; WHERE=""
  command -v ros2 > /dev/null 2>&1 || { echo "ros2 が見つからない（ROS の環境を読み込んだ端末で実行する）"; exit 1; }
fi
CFG=${CONFIG:-$target}
DT="python3 $T/drive/drive_test.py $CFG ${WHILL:+--whill $WHILL}"
BAGS=/root/ros2_ws/bags/$target

# コンテナの中でコマンドを動かす（ホストからは docker exec、コンテナの中では直接）。終了コードはコマンドのもの
cx() {
  if [ $INSIDE = 1 ]; then
    bash -c "$1"
  else
    docker exec $LOC bash -ic "$1" 2>&1 | grep -v "terminal process group\|no job control"
    return ${PIPESTATUS[0]}
  fi
}
# 裏で動かす。止めるときは pkill -INT（裏で起動すると SIGINT が無視されるので、既定に戻してから exec する）
cbg() {
  if [ $INSIDE = 1 ]; then
    setsid python3 -c "import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp('bash', ['bash', '-c', sys.argv[1]])" \
      "$1" < /dev/null > /dev/null 2>&1 &
  else
    docker exec -d $LOC bash -ic "$1"
  fi
}
# 端末で対話する（実機の走り出しの確認）
cit() {
  if [ $INSIDE = 1 ]; then bash -c "$1"; else docker exec -it $LOC bash -ic "$1"; fi
}
count_loc() {
  cx 'ps -eo cmd | grep -E "ekf_localizer_node|static_transform|ros2 launch|pcl_localization|ros2 bag|rviz2" | grep -v grep | wc -l'
}
# pkill -f のパターンの先頭を [x] にして、pkill を動かす bash 自身（コマンド行にパターンを含む）に当たらないようにする
stop_bg() { cx "pkill -INT -f '[${1:0:1}]${1:1}'"; }

# ホストから：localization_tools 一式をコンテナの /root/ros2_ws/localization_tools に写す（走行・描画・評価もそこで行う）
[ $INSIDE = 1 ] || "$root/sync.sh" push
cx "$DT $profile --simulate" || { echo "動き $profile が範囲に収まらない・設定が読めない"; exit 1; }
topics=$(cx "$DT --record-topics" | tail -1)
cx "mkdir -p $(dirname $BAGS/$bag) && touch $(dirname $BAGS)/COLCON_IGNORE"

if [ "$target" = gazebo ]; then
  extra=${4:-}
  if [ $INSIDE = 0 ]; then
    echo "== restart Gazebo"
    docker exec $GZ bash -c 'pkill -INT -f "[r]os2 launch connection_wheelchair"'
    for _ in $(seq 30); do
      [ "$(docker exec $GZ bash -c 'pgrep -c -x gzserver; pgrep -c -x gzclient' | paste -sd+ | bc)" = 0 ] && break
      sleep 1
    done
    docker exec $GZ pkill -KILL -x gzserver 2>/dev/null; docker exec $GZ pkill -KILL -x gzclient 2>/dev/null
    docker exec -d $GZ bash -ic 'ros2 launch connection_wheelchair connection_wheelchair.launch.py > /tmp/claude_gazebo_launch.log 2>&1'
    for _ in $(seq 60); do
      cx 'timeout 5 ros2 topic echo --once --no-daemon --spin-time 3 /wheelchair/odom > /dev/null 2>&1' && break
      sleep 2
    done
    sleep 5
  else
    echo "== コンテナの中なので Gazebo は起動し直さない（Gazebo が動いていて、車椅子が初期位置にあること）"
    cx 'timeout 8 ros2 topic echo --once --no-daemon --spin-time 3 /wheelchair/odom > /dev/null 2>&1' || { echo "Gazebo（/wheelchair/odom）が見えない"; exit 1; }
  fi
  echo "== start recording and localization ${extra:+($extra)}"
  [ "$(count_loc)" = 0 ] || { echo "localization processes are still running in ${WHERE:-this container}"; exit 1; }
  cx "rm -rf $BAGS/$bag $BAGS/${bag}_phases.txt $BAGS/${bag}_drive.log $BAGS/${bag}_info.yaml"
  cbg "ros2 bag record -o $BAGS/$bag $topics > /tmp/record_$tag.log 2>&1"
  sleep 3
  cbg "ros2 launch localization_bringup localization.launch.py env:=gazebo rviz:=false $extra > /tmp/launch_$tag.log 2>&1"
  sleep 25   # 地図の読み込みと NDT の収束を待つ
  echo "== drive"
  cx "$DT $profile --yes 2>&1 | tee /tmp/drive_$tag.log; exit \${PIPESTATUS[0]}"
  status=$?
else
  echo "== check the real robot"
  if cx "test -e $BAGS/$bag"; then echo "$BAGS/$bag は既にある（別の名前にする）"; exit 1; fi
  cx "timeout 8 ros2 topic echo --once --no-daemon --spin-time 3 /ekf_pose > /dev/null 2>&1" || { echo "/ekf_pose が来ない（自己位置推定を起動し、初期姿勢を合わせる）"; exit 1; }
  cmd=$(cx "$DT --command-topic" | tail -1)
  subs=$(cx "ros2 topic info --no-daemon --spin-time 3 $cmd" | sed -n 's/Subscription count: //p')
  if [ "${subs:-0}" -lt 1 ]; then
    echo "$cmd を受けるノード（whill_driver）が無い。車椅子の色（WHILL=blue|red|white）を確かめる。見つかった車椅子："
    cx "ros2 topic list --no-daemon --spin-time 3" | grep "whill_node/cmd_vel" | sed 's/^/  /'
    exit 1
  fi
  present=$(cx "ros2 topic list --no-daemon --spin-time 3")
  for t in $topics; do
    echo "$present" | grep -qx "$t" || echo "  （まだ無いトピック：$t。録画は続ける）"
  done
  echo "== start recording"
  cbg "ros2 bag record -o $BAGS/$bag $topics > /tmp/record_$tag.log 2>&1"
  sleep 3
  echo "== drive（計画を確かめて yes で走り出す。止めるときは Ctrl-C かジョイスティック）"
  cit "$DT $profile 2>&1 | tee /tmp/drive_$tag.log; exit \${PIPESTATUS[0]}"
  status=$?
fi

cx "cp /tmp/drive_phases.txt $BAGS/${bag}_phases.txt; cp /tmp/drive_$tag.log $BAGS/${bag}_drive.log"
sleep 3
echo "== stop"
stop_bg "ros2 bag record -o $BAGS/$bag"
if [ "$target" = gazebo ]; then
  stop_bg "ros2 launch localization_bringup"
  sleep 8
  echo "remaining localization processes: $(count_loc)"
  cx "grep -h 'mount base_footprint' /tmp/launch_$tag.log | cut -c1-170"
else
  sleep 5
fi
notes=$(cx "$DT $profile --describe" | tail -1)
[ "$status" = 0 ] || notes="走行を途中で止めた（drive_test.py の終了コード $status、ログ $(basename $bag)_drive.log）。$notes"
cx "python3 $T/eval/bag_info.py write $BAGS/$bag --source $target --drive 'drive/drive_test.py $profile' --purpose '${PURPOSE:-}' --notes '$notes'"
exit $status
