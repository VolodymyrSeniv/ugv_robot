"""Shared helpers for plotting/animating trial series (matplotlib, runs in the container)."""
import csv
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

# Arena (see tools/make_worlds.py): name, cx, cy, sx, sy, yaw
OBSTACLES = [('box1', 4, 1.5, 1, 1, 0.3), ('box2', 7, -2, 1.5, 0.5, 0.0),
             ('box3', 2, -3, 0.8, 0.8, 0.8), ('box4', -3, 3, 1.2, 1.2, 0.5)]
WALLS = [(-10.3, -10.15, 20.6, 0.3), (-10.3, 9.85, 20.6, 0.3), (-10.15, -10, 0.3, 20), (9.85, -10, 0.3, 20)]
COLORS = {'true': '#2a6fdb', 'slam': '#e8590c', 'guard': '#2b8a3e', 'bad': '#c92a2a', 'grid': '#dddddd'}


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def load_series(path):
    """Returns (samples dict of arrays, trials list of dict)."""
    rows = list(csv.DictReader(open(os.path.join(path, 'samples.csv'))))
    s = {k: np.array([float(r[k]) for r in rows]) for k in
         ('trial', 't', 'gt_x', 'gt_y', 'gt_yaw', 'slam_x', 'slam_y', 'slam_yaw')}
    trials = list(csv.DictReader(open(os.path.join(path, 'trials.csv'))))
    # belief expressed in the world frame: one alignment at the very first sample
    yaw = wrap(s['gt_yaw'][0] - s['slam_yaw'][0])
    c, sn = math.cos(yaw), math.sin(yaw)
    tx = s['gt_x'][0] - (c * s['slam_x'][0] - sn * s['slam_y'][0])
    ty = s['gt_y'][0] - (sn * s['slam_x'][0] + c * s['slam_y'][0])
    s['bel_x'] = c * s['slam_x'] - sn * s['slam_y'] + tx
    s['bel_y'] = sn * s['slam_x'] + c * s['slam_y'] + ty
    s['bel_yaw'] = wrap(s['slam_yaw'] + yaw)
    s['err'] = np.hypot(s['gt_x'] - s['bel_x'], s['gt_y'] - s['bel_y'])
    return s, trials


def draw_arena(ax, lim=11):
    for x, y, w, h in WALLS:
        ax.add_patch(Rectangle((x, y), w, h, color='#868e96'))
    for _, cx, cy, sx, sy, yaw in OBSTACLES:
        ax.add_patch(Rectangle((-sx / 2, -sy / 2), sx, sy, color='#adb5bd',
                               transform=matplotlib.transforms.Affine2D().rotate(yaw).translate(cx, cy) + ax.transData))
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect('equal')
    ax.grid(color=COLORS['grid'], lw=0.5)
    ax.set_xlabel('x, m')
    ax.set_ylabel('y, m')
