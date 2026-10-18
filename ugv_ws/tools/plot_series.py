"""Static figures for one trial series: top-down trajectories, SLAM error over time, per-trial errors.

usage: python3 tools/plot_series.py data/series_04_inject_guard_on docs/figures/series_04
"""
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from viz_common import COLORS, draw_arena, load_series

src, out = sys.argv[1:3]
os.makedirs(out, exist_ok=True)
name = os.path.basename(src.rstrip('/'))
s, trials = load_series(src)

# 1) top-down trajectories: ground truth vs where the robot believed it was
fig, ax = plt.subplots(1, 2, figsize=(14, 7))
for a, (kx, ky, title, col) in zip(ax, (('gt_x', 'gt_y', 'True path (simulator)', COLORS['true']),
                                        ('bel_x', 'bel_y', 'Where the robot believed it was (SLAM)', COLORS['slam']))):
    draw_arena(a, lim=9)
    for tr in np.unique(s['trial']):
        m = s['trial'] == tr
        a.plot(s[kx][m], s[ky][m], color=col, lw=0.9, alpha=0.7)
    for t in trials:
        a.plot(float(t['goal_x']), float(t['goal_y']), 'k*', ms=8)
    a.plot(0, 0, 'go', ms=9, label='start')
    a.set_title(title)
ax[0].legend(loc='lower left')
fig.suptitle(f'{name}: all trials, stars = goals')
fig.tight_layout()
fig.savefig(os.path.join(out, 'trajectories.png'), dpi=130)
plt.close(fig)

# 2) SLAM-vs-truth error over time, trial bands, injected trials marked
fig, ax = plt.subplots(figsize=(14, 4.5))
t0 = s['t'][0]
ax.plot(s['t'] - t0, s['err'], color=COLORS['slam'], lw=1)
for t in trials:
    m = s['trial'] == int(t['trial'])
    if not m.any():
        continue
    x0, x1 = s['t'][m][0] - t0, s['t'][m][-1] - t0
    if int(t['trial']) % 2 == 0:
        ax.axvspan(x0, x1, color='#f1f3f5', zorder=0)
    inj = t.get('injected', '')
    ax.text((x0 + x1) / 2, ax.get_ylim()[1] * 0.97, t['trial'] + (f"\n{inj[0]}" if inj else ''),
            ha='center', va='top', fontsize=8, color=COLORS['bad'] if inj else '#495057')
ax.axhline(1.0, color=COLORS['bad'], ls='--', lw=0.8)
ax.set_xlabel('time, s (simulated)')
ax.set_ylabel('SLAM position error, m')
ax.set_title(f'{name}: SLAM error vs true pose (trial numbers on top; G/R = injected jump during GOTO/RETURN)')
fig.tight_layout()
fig.savefig(os.path.join(out, 'slam_error_vs_time.png'), dpi=130)
plt.close(fig)

# 3) per-trial goal / return error
ok = [t for t in trials if t.get('return_err_true')]
idx = [int(t['trial']) for t in ok]
fig, ax = plt.subplots(figsize=(14, 4.5))
w = 0.4
ax.bar(np.array(idx) - w / 2, [float(t['goal_err_true']) for t in ok], w, label='goal error', color=COLORS['true'])
ax.bar(np.array(idx) + w / 2, [float(t['return_err_true']) for t in ok], w, label='return-to-start error', color=COLORS['guard'])
ax.axhline(0.3, color=COLORS['true'], ls=':', lw=1)
ax.axhline(0.5, color=COLORS['guard'], ls=':', lw=1)
for t in ok:
    if t.get('result') and t['result'] != 'SUCCESS':
        ax.text(int(t['trial']), 0, t['result'].replace('_', '\n'), ha='center', va='bottom', fontsize=6, color=COLORS['bad'])
ax.set_xlabel('trial')
ax.set_ylabel('error vs true pose, m')
ax.set_title(f'{name}: per-trial errors (dotted: acceptance 0.3 m / 0.5 m; red text: non-SUCCESS result)')
ax.legend()
fig.tight_layout()
fig.savefig(os.path.join(out, 'per_trial_errors.png'), dpi=130)
plt.close(fig)
print('figures ->', out)
