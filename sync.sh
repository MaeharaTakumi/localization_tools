#!/bin/bash
# localization_tools をホストと自己位置推定コンテナ（test）の /root/ros2_ws/localization_tools の間で写す（ホストで実行）
#   sync.sh push   ホスト -> コンテナ。コンテナ側は .git 以外を消してから写す（ホストで消したファイルを残さない。2026-11-13 から）
#   sync.sh pull   コンテナ -> ホスト（/root/ros2_ws/fig の図を RESULTS（既定は localization/src/ の隣の results）へ）
# 実行・描画はコンテナで行い、プログラムの編集はホストで行う。
set -eu
here=$(cd "$(dirname "$0")" && pwd)
LOC=${LOC:-test}
RESULTS=${RESULTS:-$here/../../results}   # 図の置き場所（ホスト。Docker/localization/results）
DST=/root/ros2_ws/localization_tools
case "${1:-push}" in
  push)
    docker exec $LOC bash -c "mkdir -p $DST && find $DST -mindepth 1 -maxdepth 1 ! -name .git -exec rm -rf {} +"
    tar -C "$here" --exclude=.git --exclude=__pycache__ -cf - . \
      | docker exec -i $LOC tar -C $DST --no-same-owner -xf -
    echo "pushed to $LOC:$DST"
    ;;
  pull)
    if docker exec $LOC test -d /root/ros2_ws/fig; then
      mkdir -p "$RESULTS"
      docker cp $LOC:/root/ros2_ws/fig/. "$RESULTS/"
      rm -f "$RESULTS/COLCON_IGNORE"   # colcon に探させないための印（図ではない）
    fi
    echo "pulled figures"
    ;;
  *)
    echo "usage: $0 [push|pull]"; exit 1
    ;;
esac
