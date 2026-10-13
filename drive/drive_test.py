#!/usr/bin/env python3
"""テスト走行（Gazebo・実機共通）：profiles.yaml の動きを、config/<target>.yaml の上限・範囲・トピックで走らせる.

  python3 drive_test.py <target> <動き> [--yes] [--phases <出力>]   走らせる（自己位置推定のコンテナで）
  python3 drive_test.py <target> <動き> --simulate                  計算した軌跡と範囲の確認だけ（ROS 不要）
  python3 drive_test.py <target> --list                             動きの一覧
  python3 drive_test.py <target> --record-topics                    録画するトピック（run_test.sh が使う）
    target：gazebo か real（drive/config/<target>.yaml）、または設定の yaml のパス
    --whill <色>：実機の車椅子（blue・red・white）。設定の whill を変え、トピックの {whill} を Drp5_whill_<色> にする

走る前に、出発時の姿勢を基準に計算した軌跡が範囲（area。plan_margin の余裕込み）に収まるかを確かめ、収まらなければ走らない。
走行中は次のどれかで止める（0 の指令を出して終わる。終了コード 1）：
  - 車体の中心が範囲の外に出た（姿勢は feedback：Gazebo は真値、実機は /ekf_pose を取付で車体に直したもの）
  - feedback_timeout より新しい姿勢が無い（実機で自己位置推定が止まった）
  - ジョイスティックが倒された（実機。whill_driver も手での操作を優先する）
  - 1 区間が max_segment_time を超えた、Ctrl-C
confirm: true（実機）なら、計画を表示して yes と入力するまで走らない（--yes で省く）。
動作の開始時刻を <phases>（既定 /tmp/drive_phases.txt）に「<区間の名前> <時刻 [s]>」で書く（最後は stop。止まる区間は
stop_<label>_start / _end）。時刻は Gazebo は sim time、実機は ROS の時刻（壁時計）。
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def config_path(target):
    return target if target.endswith('.yaml') else os.path.join(HERE, 'config', f'{target}.yaml')


WHILL_COLORS = ('blue', 'red', 'white')


def load_config(target, whill=None):
    """設定を読み、トピックの {whill} を Drp5_whill_<色>（--whill か設定の whill）にする."""
    cfg = load_yaml(config_path(target))
    color = whill or cfg.get('whill')
    if color:
        if color not in WHILL_COLORS:
            raise SystemExit(f'車椅子の色は {", ".join(WHILL_COLORS)} のどれか: {color}')
        cfg['whill'] = color

    def fill(v):
        if isinstance(v, str):
            if '{whill}' in v and not color:
                raise SystemExit(f'設定に whill（車椅子の色）が無い: {v}')
            return v.replace('{whill}', f'Drp5_whill_{color}') if color else v
        if isinstance(v, list):
            return [fill(x) for x in v]
        if isinstance(v, dict):
            return {k: fill(x) for k, x in v.items()}
        return v

    return fill(cfg)


# ---------------------------------------------------------------------------
# 動き（profiles.yaml）を、上限を掛けた区間の列にする
# ---------------------------------------------------------------------------

def expand(segments, i=None):
    """repeat を展開し、name・label の {i} を埋める."""
    out = []
    for s in segments:
        if 'repeat' in s:
            for k in range(int(s['repeat'])):
                out += expand(s['segments'], k)
        else:
            s = dict(s)
            for key in ('name', 'label'):
                if key in s and i is not None:
                    s[key] = str(s[key]).format(i=i)
            out.append(s)
    return out


def resolve(s, lim):
    """区間を {name, label, v, w, w_func, end: (種類, 値), expected} にする（速さは絶対値 m/s・rad/s）."""
    def frac(x, key):
        if abs(x) > 1.0 + 1e-9:
            raise ValueError(f'{key} は上限に対する割合（-1〜1）: {x}（区間 {s}）')
        return x

    seg = {'name': s.get('name', ''), 'label': s.get('label'), 'v': 0.0, 'w': 0.0, 'w_func': None}
    if 'stop' in s:
        seg['name'] = seg['name'] or (f"stop {seg['label']}" if seg['label'] else 'stop')
        seg['end'] = ('time', float(s['stop']))
        seg['expected'] = float(s['stop'])
        return seg
    if 'slalom' in s:
        p = s['slalom']
        v = frac(float(p['v']), 'v') * lim['v_max']
        k = (2 * math.pi / p['wavelength']) ** 2 * p['amplitude']   # omega の振幅 = k v
        if k * v > 0.95 * lim['w_max']:
            v = 0.95 * lim['w_max'] / k   # omega が上限を超えないよう遅くする
        period = p['wavelength'] / v
        amp = k * v
        total = (int(p['cycles']) + 1) * period

        def w_func(t, amp=amp, period=period, total=total):
            a = amp / 2 if t < period / 2 or t >= total - period / 2 else amp
            return a * math.sin(2 * math.pi * t / period)

        seg.update(v=v, w_func=w_func, end=('time', total), expected=total, slalom=(amp, period))
        return seg
    w = frac(float(s.get('w', 0.0)), 'w') * lim['w_max']
    if 'radius' in s:
        v = float(s['radius']) * abs(w)
        vlim = lim['v_reverse_max'] if s.get('reverse') else lim['v_max']
        if v > vlim:
            w *= vlim / v
            v = vlim
        v = -v if s.get('reverse') else v
    else:
        fv = frac(float(s.get('v', 0.0)), 'v')
        v = fv * (lim['v_max'] if fv >= 0 else lim['v_reverse_max'])
    seg.update(v=v, w=w)
    if 'time' in s:
        seg['end'] = ('time', float(s['time']))
        seg['expected'] = float(s['time'])
    elif 'yaw' in s:
        target = math.radians(float(s['yaw']))
        if w == 0 or target * w <= 0:
            raise ValueError(f'yaw と w の符号が合わない（区間 {s}）')
        seg['end'] = ('yaw', target)
        seg['expected'] = abs(target / w)
    elif 'distance' in s:
        if v == 0:
            raise ValueError(f'distance の区間は v が 0 でないこと（区間 {s}）')
        seg['end'] = ('distance', float(s['distance']))
        seg['expected'] = float(s['distance']) / abs(v)
    else:
        raise ValueError(f'区間の終わり（time・yaw・distance・stop・slalom）が無い（区間 {s}）')
    return seg


def build(cfg, profiles, name):
    if name not in profiles:
        raise SystemExit(f'動き {name} は無い。あるもの: {", ".join(profiles)}')
    return [resolve(s, cfg['limits']) for s in expand(profiles[name]['segments'])]


def omega_at(seg, t):
    return seg['w_func'](t) if seg['w_func'] else seg['w']


# ---------------------------------------------------------------------------
# 計算した軌跡と範囲
# ---------------------------------------------------------------------------

def simulate(segs, w_scale, dt=0.01):
    """出発時の姿勢を原点・x 軸前向きとした一輪車モデル（omega は w_scale 倍）。(区間ごとの要約, 軌跡)."""
    x = y = yaw = 0.0
    pts = [(0.0, 0.0)]
    rows = []
    for s in segs:
        kind, target = s['end']
        t = dyaw = dist = 0.0
        while True:
            if kind == 'time' and t >= target - 1e-9:
                break
            if kind == 'yaw' and abs(dyaw) >= abs(target):
                break
            if kind == 'distance' and dist >= target:
                break
            if t > 600:
                raise RuntimeError(f'区間 {s["name"]} が終わらない')
            w = omega_at(s, t) * w_scale
            x += s['v'] * dt * math.cos(yaw + 0.5 * w * dt)
            y += s['v'] * dt * math.sin(yaw + 0.5 * w * dt)
            yaw += w * dt
            dyaw += w * dt
            dist += abs(s['v']) * dt
            t += dt
            pts.append((x, y))
        rows.append((s, t, x, y, math.degrees(yaw)))
    return rows, np.array(pts)


def inside(area, x, y, margin=0.0):
    return (-area['behind'] + margin <= x <= area['forward'] - margin
            and -area['right'] + margin <= y <= area['left'] - margin)


def plan_report(cfg, segs, name):
    rows, pts = simulate(segs, cfg.get('sim_omega_scale', 1.0))
    area, margin = cfg['area'], cfg.get('plan_margin', 0.0)
    who = f"、車椅子 Drp5_whill_{cfg['whill']}" if cfg.get('whill') else ''
    lines = [f"動き {name}（{cfg['target']}{who}、上限 v {cfg['limits']['v_max']} m/s・後退 {cfg['limits']['v_reverse_max']} m/s・"
             f"omega {cfg['limits']['w_max']} rad/s）"]
    for s, t, x, y, yaw in rows:
        kind, target = s['end']
        end = {'time': f'{target:.1f} s', 'yaw': f'{math.degrees(target):+.0f} deg', 'distance': f'{target:.2f} m'}[kind]
        w = f"slalom ±{s['slalom'][0]:.2f}" if s.get('slalom') else f"{s['w']:+.2f}"
        lines.append(f"  {s['name']:<20} v {s['v']:+.2f} m/s  w {w:>12} rad/s  until {end:<9} "
                     f"({t:5.1f} s) -> 前 {x:+5.2f} m, 左 {y:+5.2f} m, 向き {yaw:+6.0f} deg")
    total = sum(r[1] for r in rows)
    ok = all(inside(area, px, py, margin) for px, py in pts)
    lines.append(f"  計 {total:.1f} s、前 {pts[:, 0].min():+.2f}〜{pts[:, 0].max():+.2f} m、"
                 f"左 {pts[:, 1].min():+.2f}〜{pts[:, 1].max():+.2f} m（範囲：前 {area['forward']}、後ろ {area['behind']}、"
                 f"左 {area['left']}、右 {area['right']} m、余裕 {margin} m）→ {'範囲内' if ok else '範囲の外に出る'}")
    return '\n'.join(lines), ok


# ---------------------------------------------------------------------------
# 走らせる（ROS）
# ---------------------------------------------------------------------------

def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def run(cfg, segs, name, args):
    import time as _time

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy

    fb_cfg, cmd_cfg, joy_cfg = cfg['feedback'], cfg['command'], cfg.get('joystick')
    mount = None
    if fb_cfg.get('mount'):
        m = np.eye(4)
        m[:3, :3] = rpy_matrix(*fb_cfg['mount']['rpy'])
        m[:3, 3] = fb_cfg['mount']['xyz']
        mount = np.linalg.inv(m)   # T_B = T_L · T_BL⁻¹

    class Driver(Node):

        def __init__(self):
            super().__init__('drive_test', parameter_overrides=[
                Parameter('use_sim_time', value=bool(cfg.get('use_sim_time', False)))])
            if cmd_cfg['type'] == 'twist_stamped':
                from geometry_msgs.msg import TwistStamped as CmdType
            else:
                from geometry_msgs.msg import Twist as CmdType
            self.cmd_type = CmdType
            self.pub = self.create_publisher(CmdType, cmd_cfg['topic'], 10)
            self.pose = None        # (x, y, yaw) 車体
            self.pose_time = None   # 受け取った時刻（このノードの時計）
            self.pose_mono = None   # 受け取った時刻（壁時計。sim time の時計が止まっても古さを測れるように）
            if fb_cfg['type'] == 'pose':
                from geometry_msgs.msg import Pose
                self.create_subscription(Pose, fb_cfg['topic'], lambda m: self.on_pose(m), qos_profile_sensor_data)
            else:
                from geometry_msgs.msg import PoseWithCovarianceStamped
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.create_subscription(PoseWithCovarianceStamped, fb_cfg['topic'],
                                         lambda m: self.on_pose(m.pose.pose), qos)
            self.joy = 0.0
            if joy_cfg:
                from geometry_msgs.msg import Vector3Stamped
                self.create_subscription(Vector3Stamped, joy_cfg['topic'], self.on_joy, qos_profile_sensor_data)

        def on_pose(self, p):
            if self.now() == 0.0:
                return   # sim time の時計が動き出す前（transient_local の古い値が先に届く）。受け取った時刻が分からない
            t = np.eye(4)
            t[:3, :3] = quat_matrix(p.orientation)
            t[:3, 3] = (p.position.x, p.position.y, p.position.z)
            if mount is not None:
                t = t @ mount
            self.pose = (t[0, 3], t[1, 3], math.atan2(t[1, 0], t[0, 0]))
            self.pose_time = self.now()
            self.pose_mono = _time.monotonic()

        def on_joy(self, m):
            self.joy = max(abs(m.vector.x), abs(m.vector.y))

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def send(self, v, w):
            msg = self.cmd_type()
            tw = msg
            if cmd_cfg['type'] == 'twist_stamped':
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = cmd_cfg.get('frame_id', 'base_link')
                tw = msg.twist
            tw.linear.x, tw.angular.z = float(v), float(w)
            self.pub.publish(msg)

    # Ctrl-C で rclpy が通信を閉じると、止めるための 0 の指令を送れない。rclpy には Ctrl-C を処理させず、
    # KeyboardInterrupt を受けてから 0 を送って終わる
    from rclpy.signals import SignalHandlerOptions
    import signal

    def on_term(signum, frame):   # docker の停止（SIGTERM）でも 0 を送って終わる
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_term)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Driver()
    dt = 1.0 / float(cmd_cfg.get('rate', 50.0))
    phases = []
    reason = None

    def stop_robot(n=25):
        for _ in range(n):
            node.send(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.02)

    def stale():
        """姿勢の古さ [s]（壁時計）."""
        return _time.monotonic() - node.pose_mono

    try:
        wall_limit = 15.0
        t_wait = _time.monotonic()
        while (node.now() == 0.0 or node.pose is None) and _time.monotonic() - t_wait < wall_limit:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.pose is None:
            raise RuntimeError(f"姿勢（{fb_cfg['topic']}）が {wall_limit:.0f} s 届かない。自己位置推定（実機）・Gazebo が動いているか確かめる")
        t_wait = _time.monotonic()   # 新しい姿勢が届き続けていること（止まった推定の最後の値で走り出さない）
        while stale() > cfg['feedback_timeout'] and _time.monotonic() - t_wait < 3.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        if stale() > cfg['feedback_timeout']:
            raise RuntimeError(f"姿勢（{fb_cfg['topic']}）が {stale():.1f} s 更新されていない")
        if joy_cfg and node.joy > joy_cfg['threshold']:
            raise RuntimeError('ジョイスティックが倒れている')
        text, ok = plan_report(cfg, segs, name)
        print(text, flush=True)
        if not ok:
            raise RuntimeError('計算した軌跡が範囲に収まらない（area・plan_margin か動きを直す）')
        if cfg.get('confirm') and not args.yes:
            print(f"\n指令のトピック {cmd_cfg['topic']}、姿勢 {fb_cfg['topic']}。周りの安全を確かめ、"
                  "ジョイスティックですぐ止められるようにしてから、yes と入力すると走り出す：", flush=True)
            if input().strip() != 'yes':
                raise RuntimeError('取りやめた')
            for _ in range(10):   # 入力を待つ間に溜まった姿勢を読む
                rclpy.spin_once(node, timeout_sec=0.02)
            if stale() > cfg['feedback_timeout']:
                raise RuntimeError(f"姿勢（{fb_cfg['topic']}）が {stale():.1f} s 更新されていない")
        x0, y0, yaw0 = node.pose
        c0, s0 = math.cos(yaw0), math.sin(yaw0)
        area = cfg['area']
        for s in segs:
            t0 = node.now()
            m0 = _time.monotonic()          # 区間の打ち切りは壁時計でも測る
            last_now, last_now_mono = t0, m0   # 時計（sim time）が止まっていないか
            if s['label']:
                phases.append((f"stop_{s['label']}_start", t0))
            else:
                phases.append((s['name'], t0))
            kind, target = s['end']
            prev = node.pose
            dyaw = dist = 0.0
            while True:
                rclpy.spin_once(node, timeout_sec=dt)
                now = node.now()
                el = now - t0
                mono = _time.monotonic()
                if now != last_now:
                    last_now, last_now_mono = now, mono
                elif mono - last_now_mono > 1.0:
                    raise RuntimeError('時計（/clock）が 1 s 進んでいない（Gazebo が止まった）')
                if node.pose is not prev and node.pose is not None:
                    dyaw += math.atan2(math.sin(node.pose[2] - prev[2]), math.cos(node.pose[2] - prev[2]))
                    dist += math.hypot(node.pose[0] - prev[0], node.pose[1] - prev[1])
                    prev = node.pose
                if kind == 'time' and el >= target:
                    break
                if kind == 'yaw' and abs(dyaw) >= abs(target):
                    break
                if kind == 'distance' and dist >= target:
                    break
                if stale() > cfg['feedback_timeout']:
                    raise RuntimeError(f"姿勢が {stale():.2f} s 届いていない（{fb_cfg['topic']}）")
                if joy_cfg and node.joy > joy_cfg['threshold']:
                    raise RuntimeError('ジョイスティックが倒された（手での操作）')
                ex, ey = node.pose[0] - x0, node.pose[1] - y0
                fx, fy = c0 * ex + s0 * ey, -s0 * ex + c0 * ey   # 出発時の向きを基準にした前・左
                if not inside(area, fx, fy):
                    raise RuntimeError(f'範囲の外に出た（前 {fx:+.2f} m、左 {fy:+.2f} m）')
                if el > cfg['max_segment_time'] or mono - m0 > cfg['max_segment_time']:
                    raise RuntimeError(f"区間 {s['name']} が {cfg['max_segment_time']} s で終わらない")
                node.send(s['v'], omega_at(s, el))
            if s['label']:
                phases.append((f"stop_{s['label']}_end", node.now()))
            print(f"{s['name']}: {node.now() - t0:.2f} s, yaw {math.degrees(dyaw):+.1f} deg, {dist:.2f} m", flush=True)
        phases.append(('stop', node.now()))
    except (RuntimeError, KeyboardInterrupt) as e:
        reason = str(e) or 'Ctrl-C'
    finally:
        stop_robot()
        with open(args.phases, 'w') as f:
            f.writelines(f'{n} {t:.3f}\n' for n, t in phases)
        node.destroy_node()
        rclpy.shutdown()
    if reason:
        print(f'止めた：{reason}', file=sys.stderr)
        return 1
    print('\n'.join(f'{n} {t:.3f}' for n, t in phases))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('target')
    ap.add_argument('profile', nargs='?')
    ap.add_argument('--profiles', default=os.path.join(HERE, 'profiles.yaml'))
    ap.add_argument('--simulate', action='store_true')
    ap.add_argument('--yes', action='store_true', help='確認を省く（confirm: true でも）')
    ap.add_argument('--phases', default='/tmp/drive_phases.txt')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--record-topics', action='store_true')
    ap.add_argument('--describe', action='store_true', help='動きと設定を 1 行で（録画の情報ファイル用）')
    ap.add_argument('--whill', default=None, help='実機の車椅子の色（blue・red・white）。既定は設定の whill')
    ap.add_argument('--command-topic', action='store_true', help='指令のトピック（run_test.sh が使う）')
    args = ap.parse_args()
    cfg = load_config(args.target, args.whill)
    if args.command_topic:
        print(cfg['command']['topic'])
        return 0
    profiles = load_yaml(args.profiles)
    if args.record_topics:
        print(' '.join(cfg['record_topics']))
        return 0
    if args.list or not args.profile:
        for k, p in profiles.items():
            print(f"{k:<10} {p.get('description', '')}")
        return 0
    segs = build(cfg, profiles, args.profile)
    if args.describe:
        lim, area = cfg['limits'], cfg['area']
        who = f"車椅子 Drp5_whill_{cfg['whill']}。" if cfg.get('whill') else ''
        print(f"{who}{profiles[args.profile].get('description', '')}。上限 v {lim['v_max']}・後退 {lim['v_reverse_max']} m/s・"
              f"omega {lim['w_max']} rad/s、範囲 前 {area['forward']}・後ろ {area['behind']}・左 {area['left']}・"
              f"右 {area['right']} m（出発時の車体を基準）")
        return 0
    if args.simulate:
        text, ok = plan_report(cfg, segs, args.profile)
        print(text)
        return 0 if ok else 2
    return run(cfg, segs, args.profile, args)


if __name__ == '__main__':
    sys.exit(main())
