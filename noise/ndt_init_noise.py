#!/usr/bin/env python3
"""NDT の観測ノイズ R_ndt を、「真値 + ノイズ」を初期値にして NDT を計算し直して求める（localizer_rl 内で実行）.

  python3 ndt_init_noise.py <止まる走行の bag> [--out /root/ros2_ws/fig/ndt_init] [--passes 3] [--sigma ...]

- bag は drive/profiles.yaml の stops の走行（何か所かで止まる）。止まった区間は <bag>_phases.txt の stop_<名前>_start/_end。
- 真値：止まった区間（始めの --settle 秒を除く）の /wheelchair/pose の平均（車体）に、取付（/tf_static）を掛けた LiDAR の姿勢。
  止まっているので真値の時刻のずれ（header なし）は効かない。
- NDT（pcl_localization、Gazebo の設定）だけを ROS_DOMAIN_ID=99 で起動し、bag の点群を流す。点群が届くたびに、
  止まった区間なら「真値 + 正規分布のノイズ」（--sigma、LiDAR の [x, y, z, roll, pitch, yaw]）を、それ以外は真値を
  初期値として TF（map -> base、点群の stamp）で出す（NDT は use_tf_init_guess で引く）。
- 1 回目はノイズなし（点群ごとのばらつきだけ）、2 回目以降はノイズあり（毎回違う乱数）。
- NDT の解と真値の差から、区間ごとの平均（地図と Gazebo の一定のずれ）を引いたものの分散を R_ndt とする。
  初期値のずれが解に残る割合（傾き）も出す（1 なら初期値のまま、0 なら初期値に関係なく収束）。

ノイズの既定値（運用中に EKF の予測が真値からずれる大きさ、アバウト）：今の設定の再生で、動いている間の EKF の誤差は
xy の rms 4〜6 mm・最大 29〜50 mm、z の rms 2〜6 mm・最大 7〜27 mm、yaw の rms 約 0.5°・最大 3.8〜4.9°（速度が変わる瞬間）。
速度が変わる瞬間も入るように rms の 2〜4 倍：x, y 0.02 m、z 0.01 m、roll, pitch 0.3°、yaw 1.0°。
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise')]   # plot_spin などを import する
from estimate_noise import mount_from_tf_static, NDT_KEYS, wrap  # noqa: E402
from plot_spin import read_bag  # noqa: E402
from robot_localization_adapter import matrix_to_vec, pose_msg_to_matrix, vec_to_matrix  # noqa: E402

SHARE = '/root/ros2_ws/install'
NDT_PARAM = f'{SHARE}/pcl_localization_ros2/share/pcl_localization_ros2/param/localization.yaml'
ENV = f'{SHARE}/localization_bringup/share/localization_bringup/param/env/gazebo.yaml'
DEFAULT_SIGMA = '0.02,0.02,0.01,0.3,0.3,1.0'   # [m, m, m, deg, deg, deg]


def matrix_to_quat(r):
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1])
    return x, y, z, w


def to_transform(t, stamp, parent, child):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id, msg.child_frame_id = parent, child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = t[:3, 3]
    q = matrix_to_quat(t[:3, :3])
    msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
    return msg


def perturb(t_true, noise):
    """LiDAR の姿勢に [dx, dy, dz, droll, dpitch, dyaw] を足す（位置は map 座標、角度は rpy に足す）."""
    v = matrix_to_vec(t_true)
    return vec_to_matrix(v + noise)


class InitGuess(Node):
    """点群が届くたびに初期値（真値 + ノイズ）を TF で出し、NDT の解を集める."""

    def __init__(self, stops, truth_fn, mount, base, lidar, sigma):
        super().__init__('ndt_init_noise')
        self.stops, self.truth_fn, self.mount, self.sigma = stops, truth_fn, mount, sigma
        self.inv_mount = np.linalg.inv(mount)
        self.base, self.lidar = base, lidar
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.static.sendTransform(to_transform(mount, self.get_clock().now().to_msg(), base, lidar))
        self.rng = np.random.default_rng(0)
        self.noisy = False
        self.pass_index = 0
        self.inits, self.results, self.debug = [], [], []
        self.create_subscription(PointCloud2, '/velodyne_points', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/ndt_pose', self.on_ndt, 10)
        self.create_subscription(Float32MultiArray, '/localization_debug', self.on_debug, 10)

    def stop_of(self, t):
        for i, stop in enumerate(self.stops):   # (名前, 開始, 終了, 真値の LiDAR の姿勢, 真値のばらつき)
            if stop[1] <= t <= stop[2]:
                return i
        return -1

    def on_cloud(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        k = self.stop_of(t)
        if k >= 0:
            noise = self.rng.normal(0.0, self.sigma) if self.noisy else np.zeros(6)
            t_lidar = perturb(self.stops[k][3], noise)
            self.inits.append({'pass': self.pass_index, 't': t, 'stop': k, 'noise': noise.tolist()})
        else:
            t_lidar = self.truth_fn(t) @ self.mount
        self.tf.sendTransform(to_transform(t_lidar @ self.inv_mount, msg.header.stamp, 'map', self.base))

    def on_ndt(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.results.append({'pass': self.pass_index, 't': t, 'wall': time.time(),
                             'vec': matrix_to_vec(pose_msg_to_matrix(msg.pose.pose)).tolist()})

    def on_debug(self, msg):
        self.debug.append({'pass': self.pass_index, 'wall': time.time(), 'data': list(msg.data)})


def change_state(node, transition, timeout=90.0):
    cli = node.create_client(ChangeState, '/pcl_localization/change_state')
    if not cli.wait_for_service(timeout_sec=30.0):
        raise RuntimeError('pcl_localization の change_state が見つからない')
    req = ChangeState.Request()
    req.transition.id = transition
    fut = cli.call_async(req)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        time.sleep(0.1)
    node.destroy_client(cli)
    if not fut.done() or not fut.result().success:
        raise RuntimeError(f'transition {transition} に失敗')


def stop_proc(p):
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
        try:
            p.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait()


def load_truth(d, stops_raw, settle, mount):
    clk = np.array([(t, m.clock.sec + m.clock.nanosec * 1e-9) for t, m in d['/clock']])
    pt = np.interp([t for t, _ in d['/wheelchair/pose']], clk[:, 0], clk[:, 1])
    pv = np.array([matrix_to_vec(pose_msg_to_matrix(m)) for _, m in d['/wheelchair/pose']])
    o = np.argsort(pt)
    pt, pv = pt[o], pv[o]
    pv[:, 5] = np.unwrap(pv[:, 5])
    stops = []
    for name, a, b in stops_raw:
        m = (pt >= a + settle) & (pt <= b)
        mean = pv[m].mean(axis=0)   # 止まっているので平均（yaw は unwrap 済みなので平均してよい）
        spread = pv[m].std(axis=0)
        stops.append((name, a + settle, b, vec_to_matrix(mean) @ mount, spread))

    def truth_fn(t):
        return vec_to_matrix([np.interp(t, pt, pv[:, i]) for i in range(6)])
    return stops, truth_fn


def analyse(node, stops, sigma):
    """パスごと・区間ごとに NDT の解と真値の差を取り、統計を返す."""
    init = {(r['pass'], round(r['t'], 4)): r for r in node.inits}
    rows = []
    for r in node.results:
        key = (r['pass'], round(r['t'], 4))
        if key not in init:
            continue
        k = init[key]['stop']
        true_v = matrix_to_vec(stops[k][3])
        e = np.array(r['vec']) - true_v
        e[3:] = wrap(e[3:])
        rows.append({'pass': r['pass'], 'stop': k, 'err': e, 'noise': np.array(init[key]['noise'])})
    out = {}
    for label, sel in (('no_noise', lambda p: p == 0), ('noise', lambda p: p > 0)):
        rs = [r for r in rows if sel(r['pass'])]
        if not rs:
            continue
        err = np.array([r['err'] for r in rs])
        noise = np.array([r['noise'] for r in rs])
        stop = np.array([r['stop'] for r in rs])
        bias = {k: err[stop == k].mean(axis=0) for k in set(stop.tolist())}
        res = err - np.array([bias[k] for k in stop])
        cov = res.T @ res / max(1, len(res) - len(bias))
        slope = [float((noise[:, i] * res[:, i]).sum() / (noise[:, i] ** 2).sum()) if (noise[:, i] ** 2).sum() > 0
                 else float('nan') for i in range(6)]
        out[label] = {'n': len(rs), 'cov': cov.tolist(), 'var': np.diag(cov).tolist(), 'slope': slope,
                      'bias': {stops[k][0]: bias[k].tolist() for k in bias},
                      'max_abs_res': np.abs(res).max(axis=0).tolist(),
                      'res': res.tolist(), 'noise': noise.tolist(), 'stop': stop.tolist()}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--out', default='/root/ros2_ws/fig/ndt_init')
    ap.add_argument('--passes', type=int, default=3, help='ノイズありで流す回数（このほかにノイズなしを 1 回）')
    ap.add_argument('--sigma', default=DEFAULT_SIGMA, help='ノイズの標準偏差 x,y,z [m],roll,pitch,yaw [deg]')
    ap.add_argument('--settle', type=float, default=1.0, help='止まり始めから使わない時間 [s]')
    ap.add_argument('--base', default='base_footprint')
    ap.add_argument('--lidar', default='velodyne')
    args = ap.parse_args()
    s = [float(x) for x in args.sigma.split(',')]
    sigma = np.array(s[:3] + [math.radians(x) for x in s[3:]])

    d = read_bag(args.bag, ['/clock', '/tf_static', '/wheelchair/pose'])
    mount = mount_from_tf_static(d, args.base, args.lidar)
    ph = {}
    with open(args.bag.rstrip('/') + '_phases.txt') as f:
        for line in f:
            name, t = line.rsplit(' ', 1)
            ph[name] = float(t)
    stops_raw = [(n[5:-6], ph[n], ph[n[:-6] + '_end']) for n in ph if n.startswith('stop_') and n.endswith('_start')]
    stops, truth_fn = load_truth(d, stops_raw, args.settle, mount)
    print(f'{len(stops)} stops, mount {np.round(matrix_to_vec(mount), 4).tolist()}, '
          f'sigma {np.round(sigma, 4).tolist()} [m, rad]')

    # NDT のパラメータ：パッケージの値 -> gazebo.yaml の pcl_localization -> この試験用（launch の ndt_params と同じ）
    env = yaml.safe_load(open(ENV))
    over = dict(env['parameters'].get('pcl_localization', {}))
    over.update({'base_frame_id': args.base, 'scan_time_source': 'message', 'publish_rviz_tf': False,
                 'publish_tf': False, 'use_tf_init_guess': True, 'tf_init_guess_timeout': 0.2,
                 'use_sim_time': True})
    param_path = '/tmp/ndt_init_noise_params.yaml'
    with open(param_path, 'w') as f:
        yaml.safe_dump({'/**': {'ros__parameters': over}}, f)

    os.environ['ROS_DOMAIN_ID'] = '99'
    rclpy.init()
    node = InitGuess(stops, truth_fn, mount, args.base, args.lidar, sigma)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    log = open('/tmp/ndt_init_noise.log', 'w')
    for p in range(args.passes + 1):
        node.pass_index, node.noisy = p, p > 0
        node.rng = np.random.default_rng(p)
        ndt = subprocess.Popen(
            ['ros2', 'run', 'pcl_localization_ros2', 'pcl_localization_node', '--ros-args',
             '-r', '__node:=pcl_localization', '--params-file', NDT_PARAM, '--params-file', param_path,
             '-r', 'velodyne_points:=/velodyne_points', '-r', 'pcl_pose:=/current_pose'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            change_state(node, Transition.TRANSITION_CONFIGURE)   # 地図の読み込み
            change_state(node, Transition.TRANSITION_ACTIVATE)
            time.sleep(1.0)
            subprocess.run(['ros2', 'bag', 'play', args.bag, '--topics', '/clock', '/velodyne_points'],
                           stdout=log, stderr=subprocess.STDOUT)
            time.sleep(2.0)
        finally:
            stop_proc(ndt)
        n_init = sum(1 for r in node.inits if r['pass'] == p)
        n_res = sum(1 for r in node.results if r['pass'] == p)
        print(f'pass {p} ({"noise" if p else "no noise"}): {n_init} scans in stops, {n_res} NDT results')
    log.close()

    stats = analyse(node, stops, sigma)
    node.destroy_node()
    rclpy.shutdown()

    old = [1.0e-2, 1.0e-2, 1.0e-2, 0.03, 0.03, 1.0e-3]   # 今の R_ndt（gazebo.yaml・ekf.yaml）
    unit = [('mm', 1e3)] * 3 + [('deg', math.degrees(1.0))] * 3
    print(f"\n{'':<8}{'sigma no noise':>16}{'sigma noise':>14}{'slope':>8}{'max |res|':>11}{'now R sigma':>13}")
    for i, k in enumerate(NDT_KEYS):
        u, f = unit[i]
        a = math.sqrt(stats['no_noise']['var'][i]) * f if 'no_noise' in stats else float('nan')
        b = math.sqrt(stats['noise']['var'][i]) * f
        print(f'{k + " [" + u + "]":<14}{a:10.3f}{b:14.3f}{stats["noise"]["slope"][i]:8.2f}'
              f'{stats["noise"]["max_abs_res"][i] * f:11.2f}{math.sqrt(old[i]) * f:13.2f}')
    print('bias (NDT - truth, per stop, no noise) [mm, deg]:')
    for name, b in (stats.get('no_noise') or stats['noise'])['bias'].items():
        print(f'  {name:<4} ' + ' '.join(f'{x * f:7.2f}' for x, (_, f) in zip(b, unit)))
    r_ndt = [float(f'{x:.3g}') for x in stats['noise']['var']]
    snippet = {'ekf_localizer': {'R_ndt': r_ndt}, 'robot_localization_adapter': {'R_ndt': r_ndt}}
    print('\n# R_ndt（真値 + ノイズを初期値にした NDT の誤差の分散）\n' + yaml.safe_dump(snippet, sort_keys=False))
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    with open(args.out + '.yaml', 'w') as f:
        f.write(f'# ndt_init_noise.py {args.bag} sigma {args.sigma}\n')
        yaml.safe_dump(snippet, f, sort_keys=False)
    with open(args.out + '.json', 'w') as f:
        json.dump({'bag': args.bag, 'sigma': sigma.tolist(), 'stops': [s[0] for s in stops],
                   'stats': stats, 'R_ndt': r_ndt, 'R_ndt_now': old}, f)
    print(f'saved {args.out}.yaml, {args.out}.json')
    from plot_noise import plot_init_noise
    plot_init_noise(args.out + '.png', stats, sigma, old)


if __name__ == '__main__':
    main()
