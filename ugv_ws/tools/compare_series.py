"""Comparison figures across series.

usage: python3 tools/compare_series.py docs/figures/comparison
"""
import csv
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from viz_common import COLORS

out = sys.argv[1]
os.makedirs(out, exist_ok=True)
D = 'data'


def rows(name):
    return [r for r in csv.DictReader(open(f'{D}/{name}/trials.csv')) if r.get('return_err_true')]


# A: natural operation, series 01-03
names = [('series_01', 'S01 old textures\nno guard'), ('series_02_aliased_guard', 'S02 aliased\nguard'),
         ('series_03_unique_guard', 'S03 unique\nguard')]
fig, ax = plt.subplots(1, 3, figsize=(14, 4.5))
for a, (key, title, thr) in zip(ax, (('goal_err_true', 'goal error, m', 0.3), ('return_err_true', 'return-to-start error, m', 0.5),
                                      ('ate_rmse_m', 'ATE RMSE per trial, m', None))):
    data = [[float(r[key]) for r in rows(n)] for n, _ in names]
    a.boxplot(data, labels=[l for _, l in names], showmeans=True)
    for i, d in enumerate(data, 1):
        a.scatter(np.random.default_rng(i).normal(i, 0.05, len(d)), d, s=10, color=COLORS['true'], alpha=0.6, zorder=3)
    if thr:
        a.axhline(thr, color=COLORS['bad'], ls=':', lw=1)
    a.set_title(title)
fig.suptitle('Natural operation, 16 trials per series (true-pose errors)')
fig.tight_layout()
fig.savefig(f'{out}/natural_operation.png', dpi=130)
plt.close(fig)

# B: injected jumps, guard on vs off
fig, ax = plt.subplots(figsize=(14, 4.8))
w = 0.4
for off, (name, label, col) in enumerate((('series_04_inject_guard_on', 'guard ON', COLORS['guard']),
                                           ('series_05_inject_guard_off', 'guard OFF', COLORS['bad']))):
    r = rows(name)
    x = np.array([int(t['trial']) for t in r]) + (off - 0.5) * w
    ax.bar(x, [min(float(t['return_err_true']), 11) for t in r], w, label=label, color=col)
    for t in r:
        if t['injected']:
            ax.text(int(t['trial']) + (off - 0.5) * w, min(float(t['return_err_true']), 11) + 0.1,
                    t['injected'][0], ha='center', fontsize=7)
ax.axhline(0.5, color='k', ls=':', lw=1)
ax.set_xlabel('trial  (G/R above a bar = jump injected during GOTO/RETURN; guard-OFF trials 6-8 timed out and are absent)')
ax.set_ylabel('true distance from start at the end, m')
ax.set_title('Injected map->odom jumps: how far from the start the robot really ended up')
ax.legend()
fig.tight_layout()
fig.savefig(f'{out}/injected_jumps_guard_on_vs_off.png', dpi=130)
plt.close(fig)
print('wrote', out)
