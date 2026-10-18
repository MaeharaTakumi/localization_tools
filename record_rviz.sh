#!/bin/bash
# RViz の窓を探して録画する（ホストで実行。コンテナの RViz はホストの X に描いているので、ホストの ffmpeg で窓を録る）
#
#   record_rviz.sh [-o <出力.mp4>] [-t <秒>] [-r <fps>] [--new | --select | --id <窓の ID>] [--wait <秒>] [--list]
#
#   -o       出力（既定 ./rviz_<日時>.mp4）
#   -t       録画する時間 [s]（省くと Ctrl-C まで）
#   -r       フレームレート（既定 15）
#   --new    この後に開いた RViz の窓を待って録る（launch と同時に起動するとき。例：起動の直前にこれを実行しておく）
#   --select 録る窓をクリックで選ぶ
#   --id     窓の ID を指定する（--list で分かる）
#   --wait   窓が見つかるまで待つ時間 [s]（既定 60）
#   --list   RViz の窓の一覧を出して終わる
#   指定しなければ、開いている RViz の窓が 1 つならそれ、複数なら前面の窓が RViz ならそれを録る。
#
# 窓の中身だけを録る（ffmpeg の x11grab -window_id。窓を動かしても付いていく。題名の棒は入らない）。
# 録画中に窓の大きさを変えると止まる。他の窓が上に重なるとその部分も写る。RViz を閉じると録画も止まる。
# 出力は断片化した MP4 なので、途中で止まっても再生できる。X11 だけ（Wayland のセッションでは録れない）。
set -u
PATTERN=' - RViz$'   # RViz の主窓の題名（「<設定ファイル>[*] - RViz」）
OUT=""; DURATION=""; FPS=15; MODE=auto; WIN=""; WAIT=60

usage() { awk 'NR > 1 && /^#/ {sub(/^# ?/, ""); print; next} NR > 1 {exit}' "$0"; exit "${1:-0}"; }
while [ $# -gt 0 ]; do
  case "$1" in
    -o) OUT=$2; shift 2 ;;
    -t) DURATION=$2; shift 2 ;;
    -r) FPS=$2; shift 2 ;;
    --new) MODE=new; shift ;;
    --select) MODE=select; shift ;;
    --id) MODE=id; WIN=$2; shift 2 ;;
    --wait) WAIT=$2; shift 2 ;;
    --list) MODE=list; shift ;;
    -h|--help) usage 0 ;;
    *) echo "unknown option: $1" >&2; usage 1 ;;
  esac
done

for c in ffmpeg xdotool xwininfo; do
  command -v $c >/dev/null || { echo "$c が無い（ホストで実行する。sudo apt install ffmpeg xdotool x11-utils）" >&2; exit 1; }
done
[ -n "${DISPLAY:-}" ] || { echo "DISPLAY が無い（ホストのデスクトップの端末で実行する）" >&2; exit 1; }
[ "${XDG_SESSION_TYPE:-x11}" = wayland ] && echo "警告：Wayland のセッションでは x11grab で窓を録れない" >&2

# 見えている RViz の主窓（幅 200 px 以上。起動途中の小さな窓は除く）の ID
rviz_windows() {
  local w ww
  for w in $(xdotool search --onlyvisible --name "$PATTERN" 2>/dev/null); do
    ww=$(xwininfo -id "$w" 2>/dev/null | awk '/Width:/{print $NF}')
    [ "${ww:-0}" -gt 200 ] && echo "$w"
  done | sort -u
}
describe() { echo "$1  $(xwininfo -id "$1" | awk '/Width:/{w=$NF} /Height:/{h=$NF} END{print w "x" h}')  $(xdotool getwindowname "$1")"; }

case $MODE in
  list)
    for w in $(rviz_windows); do describe "$w"; done
    exit 0 ;;
  id) ;;
  select)
    echo "録る RViz の窓をクリックする"
    WIN=$(xwininfo | awk '/Window id:/{print $4}') ;;
  new)
    before=$(rviz_windows)
    echo "新しい RViz の窓を待つ（${WAIT} s まで）"
    for _ in $(seq 1 $((WAIT * 2))); do
      WIN=$(comm -13 <(echo "$before") <(rviz_windows) | tail -1)
      [ -n "$WIN" ] && break
      sleep 0.5
    done
    sleep 2 ;;   # 窓が開いた直後は大きさが変わるので少し待つ
  auto)
    for i in $(seq 1 $((WAIT * 2))); do
      wins=$(rviz_windows)
      [ -n "$wins" ] && break
      [ "$i" = 1 ] && echo "RViz の窓を待つ（${WAIT} s まで）"
      sleep 0.5
    done
    n=$(echo "$wins" | grep -c .)
    if [ "$n" = 1 ]; then
      WIN=$wins
    elif [ "$n" -gt 1 ]; then
      active=$(xdotool getactivewindow 2>/dev/null)
      if echo "$wins" | grep -qx "${active:-none}"; then
        WIN=$active
      else
        echo "RViz の窓が $n 個ある。録る窓を前面にしてから実行するか、--select・--id <ID> で選ぶ：" >&2
        for w in $wins; do describe "$w" >&2; done
        exit 1
      fi
    fi ;;
esac
[ -n "$WIN" ] || { echo "RViz の窓が見つからない" >&2; exit 1; }
xwininfo -id "$WIN" >/dev/null 2>&1 || { echo "窓 $WIN が無い" >&2; exit 1; }

[ -n "$OUT" ] || OUT=./rviz_$(date +%Y%m%d_%H%M%S).mp4
mkdir -p "$(dirname "$OUT")"
echo "録画：$(describe "$WIN")"
echo "出力：$OUT（${DURATION:-Ctrl-C} ${DURATION:+s }まで、${FPS} fps）"

# Ctrl-C は ffmpeg が受けて書き終える。スクリプトはその後も続ける
trap ':' INT
ffmpeg -hide_banner -loglevel error -nostdin -y \
  -f x11grab -window_id "$WIN" -framerate "$FPS" -i "$DISPLAY" ${DURATION:+-t "$DURATION"} \
  -vf 'scale=trunc(iw/2)*2:trunc(ih/2)*2' -c:v libx264 -preset veryfast -crf 26 -pix_fmt yuv420p \
  -movflags +frag_keyframe+empty_moov "$OUT"
trap - INT
xwininfo -id "$WIN" >/dev/null 2>&1 || echo "窓が閉じたので録画を止めた（上の ffmpeg のエラーはそのため）"
if [ -s "$OUT" ]; then
  echo "保存：$OUT（$(du -h "$OUT" | cut -f1)、$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$OUT" 2>/dev/null | cut -d. -f1) s）"
else
  echo "録画できなかった" >&2; exit 1
fi
