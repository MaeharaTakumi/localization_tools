#!/usr/bin/env python3
"""Gazebo の車椅子を、決まった動き（加減速・曲線・スラローム・後退）で走らせる（推定器の比較用）.

  python3 drive_profile.py <動き> [--simulate]
    accel    直進の加減速・停止：0.5 → 1.0 → 1.5 → 0.5 m/s → 停止 → 1.5 m/s で急発進 → 急停止 → 1.0 m/s → 停止（約 16 m）
    curve    曲線：6 m 前進 → 8 の字（半径 1 m の左回り 1 周 → 右回り 1 周）を v, omega 0.4 と 0.8 で 1 回ずつ
    slalom   スラローム：v 1.0 m/s、omega を正弦波（振幅 0.5 rad/s、周期 6 s、最初と最後の半周期は振幅半分で向きを元に戻す）
    reverse  後退：前進 → 停止 → 後退 → 後退しながら右・左に 90° ずつ（S 字）→ 止まらずに前進 → 止まらずに速い後退 → 停止

各区間は sim time の長さか、真値 /wheelchair/pose の yaw の変化（円・90° の旋回）で終える。区間の開始時刻（sim time）を
/tmp/drive_phases.txt に「<区間の名前> <時刻>」で書く（最後は stop）。部屋は初期位置 (80, 7) の西側に幅約 8 m（y 2.5〜10.5）、
長さ約 38 m（x 46〜84）。--simulate は Gazebo なしで、omega が指令の 0.92 倍になる（滑り）として軌跡の範囲を表示する。
"""
import math
import sys

A_SLALOM, T_SLALOM, N_SLALOM = 0.5, 6.0, 3   # スラロームの振幅 [rad/s]・周期 [s]・振幅が全部の周期の数
TIMEOUT = 40.0   # yaw で終える区間の打ち切り [s]


def slalom_omega(t):
    """最初と最後の半周期は振幅半分（向きの平均が初めの向きになり、終わりで元の向きに戻る）."""
    total = (N_SLALOM + 1) * T_SLALOM
    half = A_SLALOM / 2 if t < T_SLALOM / 2 or t >= total - T_SLALOM / 2 else A_SLALOM
    return half * math.sin(2 * math.pi * t / T_SLALOM)


def seg(name, v, w=0.0, time=None, yaw=None):
    """区間：v [m/s]、w [rad/s]（定数か区間の開始からの時間の関数）、time [s] か yaw [rad]（真値の向きの変化）で終える."""
    return {'name': name, 'v': v, 'w': w, 'time': time, 'yaw': yaw}


FULL = 2 * math.pi
PROFILES = {
    'accel': [
        seg('v 0.5', 0.5, time=3.0), seg('v 1.0', 1.0, time=3.0), seg('v 1.5', 1.5, time=3.0),
        seg('v 0.5 (decel)', 0.5, time=2.0), seg('stop 1', 0.0, time=2.0),
        seg('v 1.5 (start)', 1.5, time=3.0), seg('stop 2', 0.0, time=2.0), seg('v 1.0 (start)', 1.0, time=2.0),
    ],
    'curve': [
        seg('forward', 0.5, time=12.0),
        seg('left circle', 0.4, 0.4, yaw=FULL), seg('right circle', 0.4, -0.4, yaw=-FULL),
        seg('left circle fast', 0.8, 0.8, yaw=FULL), seg('right circle fast', 0.8, -0.8, yaw=-FULL),
    ],
    'slalom': [
        seg('forward', 1.0, time=2.0),
        seg('slalom', 1.0, slalom_omega, time=(N_SLALOM + 1) * T_SLALOM),
    ],
    'reverse': [
        seg('forward', 0.5, time=8.0), seg('stop 1', 0.0, time=1.0), seg('reverse', -0.5, time=4.0),
        seg('reverse right', -0.5, -0.4, yaw=-math.pi / 2), seg('reverse left', -0.5, 0.4, yaw=math.pi / 2),
        seg('forward 2', 0.5, time=4.0), seg('reverse fast', -0.8, time=3.0),
    ],
}


def omega_of(s, t):
    return s['w'](t) if callable(s['w']) else s['w']


def simulate(segments, x=80.037, y=7.072, yaw=3.110, w_scale=0.922, dt=0.01):
    """指令どおりの一輪車モデル（omega は w_scale 倍）で軌跡の範囲を表示する."""
    xs, ys = [x], [y]
    for s in segments:
        t, yaw0 = 0.0, yaw
        while True:
            if s['time'] is not None and t >= s['time']:
                break
            if s['yaw'] is not None and abs(yaw - yaw0) >= abs(s['yaw']):
                break
            w = omega_of(s, t) * w_scale
            x += s['v'] * dt * math.cos(yaw + 0.5 * w * dt)
            y += s['v'] * dt * math.sin(yaw + 0.5 * w * dt)
            yaw += w * dt
            t += dt
            xs.append(x)
            ys.append(y)
        print(f"{s['name']:<20} {t:6.2f} s  end ({x:6.2f}, {y:5.2f})  yaw {math.degrees(yaw):7.1f} deg")
    print(f'x {min(xs):.2f} .. {max(xs):.2f}, y {min(ys):.2f} .. {max(ys):.2f}')


def main():
    profile = sys.argv[1]
    segments = PROFILES[profile]
    if '--simulate' in sys.argv:
        simulate(segments)
        return

    from geometry_msgs.msg import Pose
    from geometry_msgs.msg import TwistStamped
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data

    class Driver(Node):

        def __init__(self):
            super().__init__('drive_profile', parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.pub = self.create_publisher(TwistStamped, '/wheelchair/diff_drive_controller/cmd_vel', 10)
            self.yaw = None   # 真値の yaw（連続になるように足し合わせたもの）
            self.create_subscription(Pose, '/wheelchair/pose', self.on_pose, qos_profile_sensor_data)

        def on_pose(self, p):
            q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.yaw = yaw if self.yaw is None else self.yaw + math.atan2(math.sin(yaw - self.yaw),
                                                                          math.cos(yaw - self.yaw))

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def send(self, v, w):
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x, msg.twist.angular.z = v, w
            self.pub.publish(msg)

    rclpy.init()
    node = Driver()
    while node.now() == 0.0 or node.yaw is None:   # /clock と真値を待つ
        rclpy.spin_once(node, timeout_sec=0.1)
    lines = []
    for s in segments:
        t0, yaw0 = node.now(), node.yaw
        lines.append(f"{s['name']} {t0:.3f}")
        while True:
            t = node.now() - t0
            if s['time'] is not None and t >= s['time']:
                break
            if s['yaw'] is not None and (abs(node.yaw - yaw0) >= abs(s['yaw']) or t > TIMEOUT):
                break
            node.send(s['v'], omega_of(s, t))
            rclpy.spin_once(node, timeout_sec=0.02)
        print(f"{s['name']}: {node.now() - t0:.2f} s, yaw {math.degrees(node.yaw - yaw0):.1f} deg")
    lines.append(f'stop {node.now():.3f}')
    for _ in range(20):
        node.send(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.05)
    with open('/tmp/drive_phases.txt', 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
