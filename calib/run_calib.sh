#!/bin/bash
# パラメータ推定：推定用の動き（drive の calib。速さ・回数・秒数は param/calib.yaml）で走って録画し、続けて推定する
#
#   calib/run_calib.sh <gazebo|red|blue|white> [<bag 名>]
#     走行と録画は drive/run_test.sh <target> calib <bag 名>（Gazebo は推定器も起動、実機は推定・whill_driver・LiDAR が
#     動いていること。実機は計画を表示して yes で走り出す）。bag 名を省くと <target>_calib_<年月日_時分秒>。
#     推定は calib/estimate_all.py（観測ノイズ R・オドメトリの倍率・LiDAR の取付）。結果は /root/ros2_ws/fig/calib/<bag 名>/
#     （summary.yaml に env に入れる候補。env の yaml には書き込まない）。
#   走行を途中で止めたときは推定しない（録画が残るので calib/estimate_all.py <target> <bag> で推定はできる）。
#   ホストからでも、自己位置推定のコンテナの中からでも実行できる（run_test.sh と同じ）。
set -u
target=${1:?gazebo・red・blue・white のどれか}
bag=${2:-${target}_calib_$(date +%Y%m%d_%H%M%S)}
case $target in gazebo) source=gazebo;; red|blue|white) source=real;; *) echo "target は gazebo・red・blue・white のどれか"; exit 1;; esac
here=$(cd "$(dirname "$0")" && pwd)
root=$(dirname "$here")   # localization_tools
LOC=${LOC:-test}

PURPOSE=${PURPOSE:-パラメータ推定（calib/run_calib.sh）} "$root/drive/run_test.sh" "$target" calib "$bag"
status=$?
if [ $status != 0 ]; then
  echo "走行が最後まで終わらなかったので推定しない（終了コード $status）。推定するなら：python3 calib/estimate_all.py $target /root/ros2_ws/bags/$source/$bag"
  exit $status
fi
echo "== estimate"
if command -v docker > /dev/null 2>&1; then
  docker exec $LOC bash -ic "python3 /root/ros2_ws/localization_tools/calib/estimate_all.py $target /root/ros2_ws/bags/$source/$bag" 2>&1 \
    | grep -v "terminal process group\|no job control"
  exit ${PIPESTATUS[0]}
else
  python3 "$root/calib/estimate_all.py" "$target" "/root/ros2_ws/bags/$source/$bag"
fi
