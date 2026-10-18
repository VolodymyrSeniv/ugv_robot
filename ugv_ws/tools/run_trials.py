"""Run a series of go-to-goal-and-return missions in simulation and compute metrics.

Runs inside the container against a live `autonomy.launch.py` stack:
    python3 /ws/tools/run_trials.py --out /ws/data/trials_001 --repeats 2

Per trial the start pose is re-captured (/ugv/set_start) at the robot's current pose, a goal is sent
on /ugv/goal and the mission (GOTO -> RETURN -> IDLE) is observed. Errors are measured against the
simulator's TRUE pose (/ground_truth/odom), never against anything the robot estimates.

Frames: world = Gazebo, map = SLAM. For every trial T_world_map is estimated from the true pose and
the SLAM pose at start (the map frame is anchored there), so a goal given in `map` can be located
in the world and the SLAM estimate can be compared with the truth.
"""
import argparse
import csv
import json
import math
import os
import statistics as st
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Float32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

# Goals in the map frame (metres), at least ~1.8 m from the arena obstacles.
GOALS = [(3.0, 0.0), (4.0, -1.0), (2.0, 2.0), (5.0, -0.5),
         (3.0, -1.0), (1.5, 2.5), (4.5, -2.0), (2.0, 0.0)]
# Farther goals, for runs on a previously saved map (localization mode)
FAR_GOALS = [(6.0, -5.0), (-5.0, -5.0), (0.0, 6.0), (-6.0, 0.0),
             (-5.0, 5.0), (5.0, 5.0), (0.0, -6.0), (7.0, 0.0)]
GOTO_TIMEOUT = 180.0     # sim seconds
RETURN_TIMEOUT = 180.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(yaw, t, p):
    """Apply 2D transform (yaw, t) to point p."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * p[0] - s * p[1] + t[0], s * p[0] + c * p[1] + t[1])


def world_from_map(true_pose, slam_pose):
    """T_world_map such that T_world_map * slam_pose == true_pose."""
    yaw = wrap(true_pose[2] - slam_pose[2])
    rx, ry = compose(yaw, (0.0, 0.0), slam_pose[:2])
    return yaw, (true_pose[0] - rx, true_pose[1] - ry)


class Runner(Node):
    def __init__(self):
        super().__init__('trial_runner', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.gt = None
        self.status = ''
        self.result = ''
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/ugv/mission_result', self._on_result, latched)
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_gt, 10)
        self.create_subscription(String, '/ugv/mission_status', self._on_status, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/ugv/goal', 10)
        self.jump_pub = self.create_publisher(Float32MultiArray, '/fault/inject_jump', 10)
        self.reset_pub = self.create_publisher(Empty, '/fault/reset', 10)
        self.inject = False
        self.jump = (0.0, 0.0, 1.7)         # rotate the map frame by ~100 deg about its origin
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.samples = []            # (trial, t, gt_x, gt_y, gt_yaw, slam_x, slam_y, slam_yaw)
        self.trial = -1
        self.create_timer(0.2, self._sample)

    def _on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_status(self, m):
        self.status = m.data.split()[0] if m.data else ''

    def _on_result(self, m):
        self.result = m.data

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def slam(self):
        try:
            t = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
        except Exception:
            return None
        return (t.translation.x, t.translation.y, yaw_of(t.rotation))

    def _sample(self):
        s = self.slam()
        if self.gt is not None and s is not None and self.trial >= 0:
            self.samples.append((self.trial, self.now(), *self.gt, *s))

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until(self, cond, timeout_sim):
        t0 = self.now()
        while rclpy.ok() and self.now() - t0 < timeout_sim:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
        return False

    def trigger(self, name):
        cli = self.create_client(Trigger, name)
        if not cli.wait_for_service(timeout_sec=5.0):
            return False, 'service unavailable'
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        return (r.success, r.message) if r else (False, 'no response')

    def send_goal(self, g):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y = g
        msg.pose.orientation.w = 1.0
        for _ in range(6):                    # a single publish can be lost before discovery
            self.goal_pub.publish(msg)
            self.spin_for(0.5)
            if self.status in ('GOTO', 'RETURN'):
                return True
        return False

    def recover(self):
        """After a failed trial: try to get home, then reset the mission state."""
        self.trigger('/ugv/abort')
        self.spin_for(1.0)
        ok, _ = self.trigger('/ugv/return_home')
        if ok:
            self.wait_until(lambda: self.status == 'IDLE', RETURN_TIMEOUT)
        self.trigger('/ugv/abort')
        self.spin_for(1.0)

    def do_inject(self):
        self.jump_pub.publish(Float32MultiArray(data=list(self.jump)))
        self.spin_for(0.3)

    def reset_injection(self):
        self.reset_pub.publish(Empty())
        self.spin_for(1.5)

    def run_trial(self, idx, goal):
        self.trial = idx
        self.result = ''
        # with --inject: odd trials get a jump, during GOTO (idx%4==1) or RETURN (idx%4==3)
        phase = ('GOTO' if idx % 4 == 1 else 'RETURN' if idx % 4 == 3 else '') if self.inject else ''
        row = {'trial': idx, 'goal_x': goal[0], 'goal_y': goal[1], 'success': False,
               'result': '', 'injected': phase, 'reason': ''}
        ok, msg = self.trigger('/ugv/set_start')
        if not ok:
            row['reason'] = f'set_start failed: {msg}'
            return row
        self.spin_for(0.6)
        gt0, sl0 = self.gt, self.slam()
        if gt0 is None or sl0 is None:
            row['reason'] = 'no pose at start'
            return row
        yaw_wm, t_wm = world_from_map(gt0, sl0)
        goal_w = compose(yaw_wm, t_wm, goal)
        t_start = self.now()

        if not self.send_goal(goal):
            row['reason'] = 'goal not accepted'
            return row
        if phase == 'GOTO':
            self.spin_for(3.0)
            self.do_inject()
        reached = self.wait_until(lambda: self.status in ('RETURN', 'FAILED', 'IDLE'), GOTO_TIMEOUT)
        t_goal = self.now()
        gt_g, sl_g = self.gt, self.slam()
        if not reached or self.status == 'FAILED':
            row['reason'] = 'goto timeout' if not reached else 'goto failed'
            row['result'] = self.result
            self.recover()
            return row
        # Mission may have gone GOTO -> RETURN: pose at that moment is "at goal".
        row['goal_err_true'] = math.dist(gt_g[:2], goal_w)
        row['goal_err_slam'] = math.dist(sl_g[:2], goal)
        row['loc_err_at_goal'] = math.dist(gt_g[:2], compose(yaw_wm, t_wm, sl_g[:2]))

        if phase == 'RETURN':
            self.spin_for(6.0)
            self.do_inject()
        done = self.wait_until(lambda: self.status == 'IDLE', RETURN_TIMEOUT)
        t_end = self.now()
        self.spin_for(1.0)                    # let the robot settle
        gt1, sl1 = self.gt, self.slam()
        if not done:
            row['reason'] = 'return timeout'
            self.recover()
            return row
        self.spin_for(0.5)
        row['success'] = True                 # mission finished (any result); see 'result'
        row['result'] = self.result
        row['return_err_true'] = math.dist(gt1[:2], gt0[:2])
        row['return_err_slam'] = math.dist(sl1[:2], sl0[:2])
        row['loc_err_at_end'] = math.dist(gt1[:2], compose(yaw_wm, t_wm, sl1[:2]))
        row['end_heading_err_deg'] = abs(math.degrees(wrap(gt1[2] - gt0[2])))
        row['t_goto_s'] = t_goal - t_start
        row['t_return_s'] = t_end - t_goal
        mine = [s for s in self.samples if s[0] == idx]
        pts = np.array([[s[2], s[3]] for s in mine])
        row['path_len_true_m'] = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))) if len(pts) > 1 else 0.0
        errs = [math.dist((s[2], s[3]), compose(yaw_wm, t_wm, (s[5], s[6]))) for s in mine]
        row['ate_rmse_m'] = float(math.sqrt(np.mean(np.square(errs)))) if errs else float('nan')
        row['ate_max_m'] = float(max(errs)) if errs else float('nan')
        return row


def quat_from_yaw(y):
    return 0.0, 0.0, math.sin(y / 2), math.cos(y / 2)


def write_tum(path, samples, which):
    with open(path, 'w') as f:
        for s in samples:
            x, y, yw = (s[2], s[3], s[4]) if which == 'gt' else (s[5], s[6], s[7])
            qx, qy, qz, qw = quat_from_yaw(yw)
            f.write(f'{s[1]:.3f} {x:.4f} {y:.4f} 0 {qx} {qy} {qz:.6f} {qw:.6f}\n')


def stats(vals):
    v = sorted(x for x in vals if x == x)
    if not v:
        return {}
    p95 = v[min(len(v) - 1, int(math.ceil(0.95 * len(v))) - 1)]
    return {'n': len(v), 'mean': st.mean(v), 'median': st.median(v),
            'std': st.pstdev(v), 'p95': p95, 'max': v[-1]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--repeats', type=int, default=2)
    ap.add_argument('--max-trials', type=int, default=0)
    ap.add_argument('--goals', choices=['near', 'far'], default='near')
    ap.add_argument('--inject', action='store_true', help='inject map->odom jumps (needs fault_injector.py)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    rclpy.init()
    r = Runner()
    r.inject = a.inject
    print('waiting for the stack (status, ground truth, map->base_link)...', flush=True)
    r.wait_until(lambda: r.status == 'IDLE' and r.gt is not None and r.slam() is not None, 300)

    base = FAR_GOALS if a.goals == 'far' else GOALS
    goals = []
    for k in range(a.repeats):
        goals += base if k % 2 == 0 else list(reversed(base))
    if a.max_trials:
        goals = goals[:a.max_trials]

    rows = []
    for i, g in enumerate(goals):
        row = r.run_trial(i, g)
        if r.inject:
            r.reset_injection()
        rows.append(row)
        print(f"trial {i:2d} goal={g} {row['result'] or '-':26s} "
              f"goal_err={row.get('goal_err_true', float('nan')):.2f} "
              f"return_err={row.get('return_err_true', float('nan')):.2f} "
              f"ate={row.get('ate_rmse_m', float('nan')):.2f} {row['reason']}", flush=True)

    # global (whole-run) drift: one alignment at the first trial start
    first = [s for s in r.samples if s[0] == 0]
    glob = {}
    if first:
        s0 = first[0]
        yaw_wm, t_wm = world_from_map((s0[2], s0[3], s0[4]), (s0[5], s0[6], s0[7]))
        e = [math.dist((s[2], s[3]), compose(yaw_wm, t_wm, (s[5], s[6]))) for s in r.samples]
        glob = {'ate_rmse_m': math.sqrt(float(np.mean(np.square(e)))), 'ate_max_m': max(e),
                'final_drift_m': e[-1], 'duration_samples': len(e)}

    keys = ['trial', 'goal_x', 'goal_y', 'success', 'result', 'injected', 'reason', 'goal_err_true', 'goal_err_slam',
            'loc_err_at_goal', 'return_err_true', 'return_err_slam', 'loc_err_at_end',
            'end_heading_err_deg', 't_goto_s', 't_return_s', 'path_len_true_m', 'ate_rmse_m', 'ate_max_m']
    with open(os.path.join(a.out, 'samples.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['trial', 't', 'gt_x', 'gt_y', 'gt_yaw', 'slam_x', 'slam_y', 'slam_yaw'])
        w.writerows(r.samples)
    write_tum(os.path.join(a.out, 'gt.tum'), r.samples, 'gt')
    write_tum(os.path.join(a.out, 'slam.tum'), r.samples, 'slam')

    ok = [x for x in rows if x['success']]
    for x in ok:                              # ground-truth classification of each finished trial
        x['true_ok'] = x['goal_err_true'] < 0.3 and x['return_err_true'] < 0.5
        x['loc_bad'] = x['ate_max_m'] > 1.0   # SLAM estimate was >1 m off the truth at some point
        x['flagged'] = x['result'] != 'SUCCESS'
    reported_ok = [x for x in ok if x['result'] == 'SUCCESS']
    summary = {
        'trials': len(rows), 'finished': len(ok),
        'reported_success': len(reported_ok),
        'true_ok_of_reported_success': sum(1 for x in reported_ok if x['true_ok']),
        'false_success': sum(1 for x in reported_ok if not x['true_ok']),
        'flagged_by_guard': sum(1 for x in ok if x['flagged']),
        'results': {k: sum(1 for x in ok if x['result'] == k) for k in sorted({x['result'] for x in ok})},
        'localization_bad_trials': sum(1 for x in ok if x['loc_bad']),
        'bad_and_flagged': sum(1 for x in ok if x['loc_bad'] and x['flagged']),
        'bad_not_flagged': sum(1 for x in ok if x['loc_bad'] and not x['flagged']),
        'flagged_but_fine': sum(1 for x in ok if not x['loc_bad'] and x['flagged']),
        'failures': [{'trial': x['trial'], 'goal': (x['goal_x'], x['goal_y']), 'reason': x['reason']}
                     for x in rows if not x['success']],
        'goal_err_true_m': stats([x['goal_err_true'] for x in reported_ok]),
        'return_err_true_m': stats([x['return_err_true'] for x in ok]),
        'loc_err_at_end_m': stats([x['loc_err_at_end'] for x in ok]),
        'ate_rmse_per_trial_m': stats([x['ate_rmse_m'] for x in ok]),
        'time_total_s': stats([x['t_goto_s'] + x['t_return_s'] for x in ok]),
        'path_len_true_m': stats([x['path_len_true_m'] for x in ok]),
        'whole_run_drift': glob,
        'returned_home_true_lt_0.5m': sum(1 for x in ok if x['return_err_true'] < 0.5),
        'by_injection': {
            ph or 'none': {
                'n': len(g), 'returned_home_lt_0.5m': sum(1 for x in g if x['return_err_true'] < 0.5),
                'flagged': sum(1 for x in g if x['flagged']),
                'reported_success': sum(1 for x in g if x['result'] == 'SUCCESS'),
                'reported_success_but_not_home': sum(1 for x in g if x['result'] == 'SUCCESS' and x['return_err_true'] >= 0.5),
                'median_return_err_m': st.median([x['return_err_true'] for x in g]),
            } for ph, g in ((p, [x for x in ok if x['injected'] == p]) for p in sorted({x['injected'] for x in ok})) if g},
    }
    for x in ok:
        x['true_ok'], x['loc_bad'], x['flagged'] = int(x['true_ok']), int(x['loc_bad']), int(x['flagged'])
    keys += ['true_ok', 'loc_bad', 'flagged']
    with open(os.path.join(a.out, 'trials.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, keys, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(a.out, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2), flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
