"""Animate selected trials of a series: true robot vs where SLAM believes it is, top-down.

usage: python3 tools/make_video.py data/series_04_inject_guard_on 1,3 docs/videos/guard_on_jumps [--formats mp4,gif]
"""
import argparse
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

sys.path.insert(0, os.path.dirname(__file__))
from viz_common import COLORS, draw_arena, load_series

ap = argparse.ArgumentParser()
ap.add_argument('series')
ap.add_argument('trials', help='comma separated trial indices, played in this order')
ap.add_argument('out', help='output path without extension')
ap.add_argument('--formats', default='mp4,gif')
ap.add_argument('--fps', type=int, default=25)
ap.add_argument('--step', type=int, default=1, help='use every n-th sample (samples are 5 Hz)')
ap.add_argument('--lim', type=float, default=7.0)
a = ap.parse_args()

s, trials = load_series(a.series)
info = {int(t['trial']): t for t in trials}
sel = [int(x) for x in a.trials.split(',')]
frames = []                                    # (trial, index into samples)
for tr in sel:
    idx = np.where(s['trial'] == tr)[0][::a.step]
    frames += [(tr, i) for i in idx]
title = os.path.basename(a.series.rstrip('/'))

fig, ax = plt.subplots(figsize=(7.2, 7.2), dpi=100)
draw_arena(ax, lim=a.lim)
true_line, = ax.plot([], [], color=COLORS['true'], lw=1.6, label='true path')
bel_line, = ax.plot([], [], color=COLORS['slam'], lw=1.6, ls='--', label='SLAM belief')
goal_pt, = ax.plot([], [], 'k*', ms=14, label='goal')
ax.plot(0, 0, 'go', ms=8, label='map origin')
tri_true = Polygon(np.zeros((3, 2)), closed=True, color=COLORS['true'], zorder=5)
tri_bel = Polygon(np.zeros((3, 2)), closed=True, fill=False, ec=COLORS['slam'], lw=2, zorder=6)
ax.add_patch(tri_true)
ax.add_patch(tri_bel)
txt = ax.text(0.02, 0.98, '', transform=ax.transAxes, va='top', fontsize=10,
              bbox=dict(boxstyle='round', fc='white', ec='#adb5bd', alpha=0.9))
ax.legend(loc='lower right', fontsize=8)


def tri(x, y, yaw, size=0.45):
    pts = np.array([[size, 0], [-size * 0.6, size * 0.5], [-size * 0.6, -size * 0.5]])
    c, sn = np.cos(yaw), np.sin(yaw)
    return pts @ np.array([[c, sn], [-sn, c]]) + [x, y]


state = {'trial': None, 'start': 0}


def update(k):
    tr, i = frames[k]
    if tr != state['trial']:
        state['trial'], state['start'] = tr, np.where(s['trial'] == tr)[0][0]
        goal_pt.set_data([float(info[tr]['goal_x'])], [float(info[tr]['goal_y'])])
    j = slice(state['start'], i + 1)
    true_line.set_data(s['gt_x'][j], s['gt_y'][j])
    bel_line.set_data(s['bel_x'][j], s['bel_y'][j])
    tri_true.set_xy(tri(s['gt_x'][i], s['gt_y'][i], s['gt_yaw'][i]))
    tri_bel.set_xy(tri(s['bel_x'][i], s['bel_y'][i], s['bel_yaw'][i]))
    t = info[tr]
    inj = f"  injected jump: {t['injected']}" if t.get('injected') else ''
    txt.set_text(f"{title}\ntrial {tr}  t = {s['t'][i] - s['t'][state['start']]:5.1f} s{inj}\n"
                 f"SLAM error {s['err'][i]:.2f} m\nresult: {t.get('result') or '-'}")
    ax.set_title('blue = true robot, orange outline = where SLAM thinks it is')
    return true_line, bel_line, goal_pt, tri_true, tri_bel, txt


anim = animation.FuncAnimation(fig, update, frames=len(frames), blit=False)
os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
for fmt in a.formats.split(','):
    path = f'{a.out}.{fmt}'
    if fmt == 'mp4':
        try:
            import imageio_ffmpeg
            matplotlib.rcParams['animation.ffmpeg_path'] = imageio_ffmpeg.get_ffmpeg_exe()
            anim.save(path, writer=animation.FFMpegWriter(fps=a.fps, bitrate=1800))
        except Exception as e:                      # no ffmpeg: leave only the gif
            print('mp4 skipped:', e)
            continue
    else:
        anim.save(path, writer=animation.PillowWriter(fps=min(a.fps, 15)))
    print('wrote', path, f'{os.path.getsize(path) / 1e6:.1f} MB', len(frames), 'frames')
