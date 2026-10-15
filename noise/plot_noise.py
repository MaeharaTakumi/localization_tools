#!/usr/bin/env python3
"""estimate_noise.py の図（estimate_noise.py から呼ぶ）.

  plot_segments：使った区間（指令が一定）と、オドメトリ・NDT の残差の時間推移
  plot_summary ：標準偏差の比較（真値なしの推定：動いている区間・停止、--truth なら真値との差）
  plot_hist    ：残差のヒストグラムと正規分布（estimate_noise.py・ndt_init_noise.py）
"""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise')]   # plot_spin などを import する
from plot_spin import (BAND, capitalize_labels, GRID, INK, INK2, SURFACE,  # noqa: E402
                       widths_back_to_front)

CLASS_LABEL = {'stopped': 'Stopped', 'turning': 'Turning', 'straight': 'Straight', 'curve': 'Curve'}
C_OMEGA, C_V, C_YAW, C_X, C_Y = '#2a78d6', '#eb6834', '#4a3aa7', '#1baf7a', '#eda100'
C_EST, C_EFF, C_STOP, C_TRUTH = '#2a78d6', '#4a3aa7', '#1baf7a', '#eb6834'
C_MAD = '#eda100'   # ヒストグラムの頑健な sigma の正規分布


def setup():
    plt.rcParams.update({
        'font.size': 10, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2,
        'ytick.color': INK2, 'text.color': INK, 'axes.titlecolor': INK, 'axes.titlesize': 10,
        'axes.titleweight': 'bold', 'axes.titlelocation': 'left'})


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(axis='y', color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for spine in ('top', 'right'):
        ax.spines[spine].set_visible(False)


def plot_segments(path, ot, ov, ow, nt, used, odom_rows, ndt_rows, segs, half):
    setup()
    moving = [s for s in segs if abs(s[2]) > 1e-3 or abs(s[3]) > 1e-3]
    t0 = moving[0][0] if moving else ot[0]
    fig, axes = plt.subplots(5, 1, figsize=(12, 12), sharex=True, facecolor=SURFACE)
    panels = [
        ('Odometry angular velocity (scaled) [rad/s]', [('Omega', ot, ow, C_OMEGA)], False),
        ('Odometry linear velocity [m/s]', [('V', ot, ov, C_V)], False),
        ('Odometry residual (after removing the local linear trend)  angular velocity [rad/s]',
         [('Omega residual', None, 'omega', C_OMEGA)], 'odom'),
        ('NDT residual  yaw [deg]', [('Yaw residual', None, 'yaw', C_YAW)], 'ndt'),
        ('NDT residual  x, y (map frame) [mm]', [('X residual', None, 'x', C_X), ('Y residual', None, 'y', C_Y)], 'ndt'),
    ]
    for ax, (title, series, kind) in zip(axes, panels):
        style(ax)
        for a, b, cls in used:   # 使った区間（指令が一定）
            ax.axvspan(a - t0, b - t0, color=BAND, lw=0, zorder=0)
        widths = widths_back_to_front(len(series))
        for i, (label, t, y, color) in enumerate(series):
            if kind is False:
                ax.plot(t - t0, y, color=color, lw=1.4, zorder=3, label=label)
                continue
            rows = odom_rows if kind == 'odom' else ndt_rows
            scale = 1.0 if kind == 'odom' else (math.degrees(1.0) if y in ('yaw',) else 1e3)
            for k, (_, r) in enumerate(rows):
                ax.plot(r['_t'] - t0, r[y] * scale, color=color, lw=widths[i], zorder=3 + i,
                        label=label if k == 0 else None)
        if kind:
            ax.axhline(0, color=INK2, lw=0.8, zorder=2)
        ax.set_title(title)
        if len(series) > 1:
            ax.legend(loc='upper right', frameon=False, ncol=len(series))
    for a, b, cls in used:   # 区間の種類を一番上に書く
        axes[0].text((a + b) / 2 - t0, 1.02, CLASS_LABEL[cls], transform=axes[0].get_xaxis_transform(),
                     ha='center', va='bottom', fontsize=8.5, color=INK2)
    axes[-1].set_xlabel('Time since the first motion command [s]')
    fig.suptitle(f'Segments with a constant command (shaded) and residuals used for R (local linear window '
                 f'{2 * half:.1f} s)', x=0.02, ha='left', fontsize=12, fontweight='bold', color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(path, dpi=110, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {path}')


def plot_summary(path, est_odom, est_ndt, tru_odom, tru_ndt, tau=None):
    setup()
    groups = [('Odometry v [m/s]', est_odom, tru_odom, [('v', 'V')], 1.0),
              ('Odometry omega [rad/s]', est_odom, tru_odom, [('omega', 'Omega')], 1.0),
              ('NDT position (map frame) [mm]', est_ndt, tru_ndt, [('x', 'X'), ('y', 'Y'), ('z', 'Z')], 1e3),
              ('NDT angle [deg]', est_ndt, tru_ndt, [('roll', 'Roll'), ('pitch', 'Pitch'), ('yaw', 'Yaw')],
               math.degrees(1.0))]
    bars = [('No truth: moving', C_EST, lambda e, t, k: e['moving'][k][0]),
            ('No truth: stopped', C_STOP, lambda e, t, k: e['stopped'][k][0])]
    if tau is not None:   # 時間的なつながりの補正をした R_eff（分散 x tau_int）
        bars.insert(1, ('No truth: moving x correlation time (R_eff)', C_EFF,
                        lambda e, t, k: e['moving'][k][0] * tau[k]))
    if tru_odom is not None:
        bars.append(('Truth check: moving (Gazebo only)', C_TRUTH, lambda e, t, k: t['moving'][k][0]))
    fig, axes = plt.subplots(1, 4, figsize=(18, 4.8), facecolor=SURFACE,
                             gridspec_kw={'width_ratios': [1.3, 1.3, 2.4, 2.4]})
    width = 0.8 / len(bars)
    for ax, (title, est, tru, keys, f) in zip(axes, groups):
        style(ax)
        top = 0.0
        for j, (label, color, get) in enumerate(bars):
            for i, (k, _) in enumerate(keys):
                var = get(est, tru, k)
                s = math.sqrt(var) * f if np.isfinite(var) else 0.0
                x = i + (j - (len(bars) - 1) / 2) * width
                ax.bar(x, s, width * 0.92, color=color, zorder=3, label=label if i == 0 else None)
                ax.text(x, s, f'{s:.2g}', ha='center', va='bottom', fontsize=7.5, color=INK)
                top = max(top, s)
        ax.set_ylim(0, top * 1.18 if top > 0 else 1)
        ax.set_xticks(range(len(keys)))
        ax.set_xticklabels([lab for _, lab in keys])
        ax.set_title(title, fontsize=9.5)
    axes[0].set_ylabel('Standard deviation')
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper left', bbox_to_anchor=(0.02, 0.93), ncol=4, frameon=False)
    fig.suptitle('Observation noise (standard deviation) estimated without the truth', x=0.02, ha='left',
                 fontsize=12, fontweight='bold', color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.84))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(path, dpi=120, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {path}')


def plot_init_noise(path, stats, sigma, old):
    """ndt_init_noise.py の図：初期値のずれと NDT の誤差の散布図（6 成分）と、標準偏差の比較."""
    setup()
    names = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
    scale = [1e3] * 3 + [math.degrees(1.0)] * 3
    units = ['mm'] * 3 + ['deg'] * 3
    st = stats['noise']
    res, noise, stop = np.array(st['res']), np.array(st['noise']), np.array(st['stop'])
    fig = plt.figure(figsize=(18, 8.6), facecolor=SURFACE)
    gs = fig.add_gridspec(2, 6, height_ratios=[1, 1.05], hspace=0.45, wspace=0.35)
    for i in range(6):
        ax = fig.add_subplot(gs[0, i])
        style(ax)
        x, y = noise[:, i] * scale[i], res[:, i] * scale[i]
        ax.scatter(x, y, s=7, color=C_EST, alpha=0.55, lw=0, zorder=3)
        lim = np.abs(x).max() * 1.05
        k = st['slope'][i]
        ax.plot([-lim, lim], [-k * lim, k * lim], color=INK2, lw=1.0, zorder=4)
        ax.axhline(0, color=GRID, lw=0.8, zorder=2)
        ax.set_title(f'{names[i]}  (slope {k:.2f})', fontsize=9.5)
        ax.set_xlabel(f'Initial guess error [{units[i]}]')
        if i == 0:
            ax.set_ylabel('NDT error, stop mean removed')
    groups = [('NDT position error, standard deviation [mm]', range(3)),
              ('NDT angle error, standard deviation [deg]', range(3, 6))]
    bars = [('Initial guess = truth (no noise)', C_STOP, lambda i: stats['no_noise']['var'][i] if 'no_noise' in stats else np.nan),
            ('Initial guess = truth + noise (used as R)', C_EST, lambda i: st['var'][i]),
            ('Current R (gazebo.yaml)', C_TRUTH, lambda i: old[i])]
    for g, (title, idx) in enumerate(groups):
        ax = fig.add_subplot(gs[1, 3 * g:3 * g + 3])
        style(ax)
        idx = list(idx)
        width = 0.8 / len(bars)
        top = 0.0
        for j, (label, color, get) in enumerate(bars):
            for n, i in enumerate(idx):
                v = math.sqrt(get(i)) * scale[i] if np.isfinite(get(i)) else 0.0
                x = n + (j - 1) * width
                ax.bar(x, v, width * 0.92, color=color, zorder=3, label=label if n == 0 else None)
                ax.text(x, v, f'{v:.2g}', ha='center', va='bottom', fontsize=8, color=INK)
                top = max(top, v)
        ax.set_yscale('log')
        ax.set_ylim(top * 1e-4 if top > 0 else 1e-3, top * 3)
        ax.set_xticks(range(len(idx)))
        ax.set_xticklabels([names[i] for i in idx])
        ax.set_title(title + '  (log scale)', fontsize=9.5)
        if g == 0:
            ax.legend(loc='upper left', frameon=False, fontsize=9)
    sig = ', '.join(f'{n} {s * f:.3g} {u}' for n, s, f, u in zip(names, sigma, scale, units))
    fig.suptitle(f'NDT error when the initial guess is truth + noise (noise sigma: {sig})', x=0.02, ha='left',
                 fontsize=12, fontweight='bold', color=INK, y=0.995)
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(path, dpi=110, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {path}')


def plot_hist(path, panels, title):
    """残差のヒストグラム（確率密度）と正規分布 N(0, sigma^2) の比較.

    panels：[{'title', 'unit', 'data'（表示の単位）, 'curves': [(凡例, sigma（表示の単位）, 色), ...]}]。
    curves の 1 つ目は残差そのものの sigma（二乗平均の平方根。R に使う値）。範囲は +-4.5 sigma（外れた点の数を書く）。
    外れた点に左右されない sigma（1.4826 x 中央絶対偏差。正規分布なら sigma と同じ）の正規分布も重ねる（裾の重さを見る）。
    """
    setup()
    ncol = 3
    nrow = (len(panels) + ncol - 1) // ncol
    fig, axes = plt.subplots(nrow, ncol, figsize=(15, 3.9 * nrow), facecolor=SURFACE, squeeze=False)
    for ax in axes.flat[len(panels):]:
        ax.set_visible(False)
    for ax, p in zip(axes.flat, panels):
        style(ax)
        x = np.asarray(p['data'], float)
        x = x[np.isfinite(x)]
        ax.set_title(p['title'], fontsize=9.5, pad=15)
        ax.set_xlabel(f"Residual [{p['unit']}]")
        if len(x) < 5:
            ax.text(0.5, 0.5, 'No data', transform=ax.transAxes, ha='center', va='center', color=INK2)
            continue
        s0 = float(np.sqrt(np.mean(x ** 2)))
        lim = 4.5 * s0 if s0 > 0 else 1.0
        out = int((np.abs(x) > lim).sum())
        dens, _, _ = ax.hist(x[np.abs(x) <= lim], bins=np.linspace(-lim, lim, 46), density=True, color=C_EST,
                             alpha=0.35, edgecolor=SURFACE, lw=0.6, zorder=2, label=f'Residuals (n {len(x)})')
        # 縦軸はヒストグラムと（残差の sigma・頑健な sigma の）正規分布の山の高いほうまで。それより細い分布は上が切れる
        robust = 1.4826 * float(np.median(np.abs(x - np.median(x))))
        curves = list(p['curves'][:1]) + ([('Normal, robust sigma (1.4826 MAD)', robust, C_MAD)] if robust > 0 else []) \
            + list(p['curves'][1:])
        peak = max([float(dens.max())] + [1.0 / (sg * math.sqrt(2 * math.pi)) for _, sg, _ in curves[:2] if sg > 0])
        grid = np.linspace(-lim, lim, 400)
        widths = widths_back_to_front(len(curves))
        for i, (label, sigma, color) in enumerate(curves):
            if not (np.isfinite(sigma) and sigma > 0):
                continue
            pdf = np.exp(-0.5 * (grid / sigma) ** 2) / (sigma * math.sqrt(2 * math.pi))
            ax.plot(grid, np.minimum(pdf, 1.08 * peak), color=color, lw=widths[i], zorder=3 + i,
                    label=f'{label} (sigma {sigma:.3g})')
        ax.set_ylim(0, 1.15 * peak)
        m = x - x.mean()
        kurt = float(np.mean(m ** 4) / np.mean(m ** 2) ** 2 - 3.0) if np.mean(m ** 2) > 0 else float('nan')
        note = f'mean {x.mean():.2g}, excess kurtosis {kurt:.1f}' + (f', {out} outside +-4.5 sigma' if out else '')
        ax.text(0.0, 1.015, note, transform=ax.transAxes, ha='left', va='bottom', fontsize=8, color=INK2)
        ax.legend(loc='upper left', frameon=False, fontsize=7.8)
    axes[0][0].set_ylabel('Probability density')
    fig.suptitle(title, x=0.02, ha='left', fontsize=12, fontweight='bold', color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(path, dpi=110, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {path}')
