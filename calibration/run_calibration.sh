#!/bin/bash
# パラメータ推定：推定用の動き（calibration/param/motion.yaml）で走って録画し、続けて推定する（calibration/calibrate.py）
#
#   calibration/run_calibration.sh <gazebo|red|blue|white> [<bag 名>]
#     走行と録画は drive/run_test.sh <target> calibration <bag 名>（動きは calibration/motion.py が作り、PROFILES で渡す。Gazebo は推定器も
#     起動、実機は推定・whill_driver・LiDAR が動いていること。実機は計画を表示して yes で走り出す）。
#     bag 名を省くと <target>_calibration_<年月日_時分秒>。
#     推定は calibrate.py（観測ノイズ R・オドメトリの倍率・LiDAR の取付）。推定の設定は
#     calibration/param/calibrate.yaml。結果は /root/ros2_ws/fig/calibration/<bag 名>/（summary.yaml に env に入れる候補。env の yaml には書き込まない）。
#   走行を途中で止めたときは推定しない（録画が残るので calibration/calibrate.py <target> <bag> で推定はできる）。
#   ホストからでも、自己位置推定のコンテナの中からでも実行できる（run_test.sh と同じ）。
set -u
target=${1:?gazebo・red・blue・white のどれか}
bag=${2:-${target}_calibration_$(date +%Y%m%d_%H%M%S)}
case $target in gazebo) source=gazebo;; red|blue|white) source=real;; *) echo "target は gazebo・red・blue・white のどれか"; exit 1;; esac
here=$(cd "$(dirname "$0")" && pwd)
root=$(dirname "$here")   # localization_tools
LOC=${LOC:-test}
PROF=/tmp/calibration_profiles.yaml   # 推定用の動き（コンテナの中）

if command -v docker > /dev/null 2>&1; then
  "$root/sync.sh" push > /dev/null
  cx() { docker exec $LOC bash -ic "$1" 2>&1 | grep -v "terminal process group\|no job control"; return ${PIPESTATUS[0]}; }
  T=/root/ros2_ws/localization_tools
else
  cx() { bash -c "$1"; }
  T=$root
fi
cx "python3 $T/calibration/motion.py ${CONFIG:-$target} --out $PROF" || exit 1

PROFILES=$PROF PURPOSE=${PURPOSE:-パラメータ推定（calibration/run_calibration.sh）} "$root/drive/run_test.sh" "$target" calibration "$bag"
status=$?
if [ $status != 0 ]; then
  echo "走行が最後まで終わらなかったので推定しない（終了コード $status）。推定するなら：calibrate $target $bag"
  exit $status
fi
echo "== calibrate"
cx "python3 $T/calibration/calibrate.py $target /root/ros2_ws/bags/$source/$bag"
