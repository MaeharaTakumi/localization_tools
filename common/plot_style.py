"""図の共通の書式（色・線幅・ラベル・枠）.

描き方の決まり（ユーザー指示）：
  - 折れ線は、後方（先に描く）の線ほど太く描く。真値は一番後ろの太い薄灰色の実線。点線・破線はできるだけ使わない
  - 題・軸ラベル・凡例・注記の頭文字は大文字（保存の直前に capitalize_labels(fig)）
  - 図の枠は 4 辺とも黒（box_on。MATLAB の box on）
"""
import os

import matplotlib.pyplot as plt
import numpy as np

# dataviz 参照パレット（categorical slot 1〜4）と面・文字色
SERIES = os.environ.get('PLOT_COLORS', '#2a78d6,#eb6834,#1baf7a,#eda100').split(',')  # 系列の色は環境変数で固定できる
SURFACE, INK, INK2, GRID, BAND = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df', '#f0efec'
BAND_ALT = '#f6f5f2'   # 動作の区間の帯（BAND と交互に塗る）
# 真値は一番後ろの太い薄灰色の実線
TRUTH_COLOR, TRUTH_LW = '#a3a29c', 3.6


def capitalize_labels(fig):
    """図の題・軸ラベル・凡例・注記の頭文字を大文字にする（savefig の直前に呼ぶ）.

    先頭の語が識別子（ekf_localizer、/wheelchair/odom、angle_diff など _ か / を含む語）ならそのままにする。
    """
    def fix(text):
        s = text.get_text()
        if not s or not s[0].islower():
            return
        first = s.split()[0]
        if '_' in first or '/' in first:
            return
        text.set_text(s[0].upper() + s[1:])

    texts = list(fig.texts)
    if fig._suptitle is not None:
        texts.append(fig._suptitle)
    for leg in fig.legends:
        texts += leg.get_texts()
    for ax in fig.axes:
        texts += [ax.title, ax._left_title, ax._right_title, ax.xaxis.label, ax.yaxis.label, *ax.texts]
        if ax.get_legend() is not None:
            texts += ax.get_legend().get_texts()
    for text in texts:
        fix(text)


def widths_back_to_front(n, widest=2.6, thinnest=1.1):
    """n 本の線を描く順（後方→手前）の線幅。後方ほど太い."""
    if n <= 1:
        return [1.6] * n
    return [float(w) for w in np.linspace(widest, thinnest, n)]


def setup():
    """図の共通の設定（文字の大きさ・色、題は左寄せの太字）。図を作る前に呼ぶ."""
    plt.rcParams.update({
        'font.size': 10, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2,
        'ytick.color': INK2, 'text.color': INK, 'axes.titlecolor': INK, 'axes.titlesize': 10,
        'axes.titleweight': 'bold', 'axes.titlelocation': 'left'})


def box_on(ax):
    """図の枠を 4 辺とも黒にする（MATLAB の box on）."""
    for spine in ax.spines.values():
        spine.set_visible(True)
        spine.set_color('black')
        spine.set_linewidth(0.8)


def style(ax, grid_axis='y', box=True, bands=()):
    """軸の共通の書式：面の色、目盛り線（grid_axis：'y' か 'both'）、枠（box=True なら 4 辺とも黒、False なら上と右を消す）.

    bands（[(名前, 開始, 終了)]）を渡すと、動作の区間を交互の色の帯で塗る。
    """
    ax.set_facecolor(SURFACE)
    for i, (_, a, b) in enumerate(bands):
        ax.axvspan(a, b, color=BAND if i % 2 == 0 else BAND_ALT, lw=0, zorder=0)
    ax.grid(axis=grid_axis, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    if box:
        box_on(ax)
    else:
        for spine in ('top', 'right'):
            ax.spines[spine].set_visible(False)
