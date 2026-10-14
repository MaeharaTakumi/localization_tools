#!/bin/bash
# localization_tools をホストと自己位置推定コンテナ（test_maehara）の /root/ros2_ws/localization_tools の間で写す（ホストで実行）
#   sync.sh push   ホスト -> コンテナ（.git を除く。eval/results の既存のファイルはコンテナ側を残す）
#   sync.sh pull   コンテナ -> ホスト（eval/results の結果と、/root/ros2_ws/fig の図を workspace/results へ）
# 実行・描画・結果の追記はコンテナで行い、プログラムの編集はホストで行う。
set -eu
here=$(cd "$(dirname "$0")" && pwd)
LOC=${LOC:-test_maehara}
DST=/root/ros2_ws/localization_tools
case "${1:-push}" in
  push)
    docker exec $LOC mkdir -p $DST
    tar -C "$here" --exclude=.git --exclude=__pycache__ --exclude=./eval/results -cf - . \
      | docker exec -i $LOC tar -C $DST --no-same-owner -xf -
    # 結果（jsonl）はコンテナで追記するので、コンテナに無いものだけ写す
    tar -C "$here" -cf - eval/results | docker exec -i $LOC tar -C $DST --no-same-owner --skip-old-files -xf -
    echo "pushed to $LOC:$DST"
    ;;
  pull)
    docker cp $LOC:$DST/eval/results/. "$here/eval/results/"
    if docker exec $LOC test -d /root/ros2_ws/fig; then
      docker cp $LOC:/root/ros2_ws/fig/. "$here/../results/"
      rm -f "$here/../results/COLCON_IGNORE"   # colcon に探させないための印（図ではない）
    fi
    echo "pulled eval/results and figures"
    ;;
  *)
    echo "usage: $0 [push|pull]"; exit 1
    ;;
esac
