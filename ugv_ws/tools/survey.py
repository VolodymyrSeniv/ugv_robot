"""Drive a tour of waypoints with Nav2 to build a map (mapping mode).

The map only covers what the camera has seen, and a goal outside the known map is refused, so each
waypoint is approached in short steps (<= STEP m) along a straight line, like a crude frontier
exploration. A failed step is retried shorter; a waypoint that cannot be approached is skipped.

usage (inside the container): python3 /ws/tools/survey.py
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener

# map frame == world at start; stays clear of the arena obstacles (see tools/make_worlds.py)
WAYPOINTS = [(4, -1), (6, -5), (0, -6), (-5, -5), (-6, 0), (-5, 5), (0, 6), (5, 5), (6, 1), (0, 0)]
TIMEOUT = 90.0    # sim seconds per step
STEP = 2.0        # m


class Survey(Node):
    def __init__(self):
        super().__init__('survey', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)

    def pose(self):
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform.translation
                return t.x, t.y
            except Exception:
                pass
        return None

    def approach(self, wx, wy):
        """Walk towards (wx, wy) in steps; returns 'ok' when within 0.6 m, else a failure reason."""
        fails = 0
        for _ in range(40):
            p = self.pose()
            if p is None:
                return 'no pose'
            dx, dy = wx - p[0], wy - p[1]
            d = math.hypot(dx, dy)
            if d < 0.6:
                return 'ok'
            step = min(d, STEP if fails == 0 else 1.0)
            yaw = math.atan2(dy, dx)
            r = self.go(p[0] + dx / d * step, p[1] + dy / d * step, yaw)
            print(f'   step from ({p[0]:.1f},{p[1]:.1f}) to ({p[0] + dx / d * step:.1f},{p[1] + dy / d * step:.1f}): {r}', flush=True)
            fails = 0 if r == 'ok' else fails + 1
            if fails >= 3:
                return r
        return 'too many steps'

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def go(self, x, y, yaw=0.0):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        fut = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return 'rejected'
        res = handle.get_result_async()
        t0 = self.now()
        while rclpy.ok() and not res.done() and self.now() - t0 < TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not res.done():
            handle.cancel_goal_async()
            rclpy.spin_once(self, timeout_sec=1.0)
            return 'timeout'
        return {4: 'ok', 5: 'canceled', 6: 'aborted'}.get(res.result().status, str(res.result().status))


def main():
    rclpy.init()
    n = Survey()
    n.nav.wait_for_server()
    n.pose()
    ok = 0
    for i, (x, y) in enumerate(WAYPOINTS):
        r = n.approach(x, y)
        ok += r == 'ok'
        print(f'waypoint {i} ({x}, {y}): {r}', flush=True)
    print(f'survey done: {ok}/{len(WAYPOINTS)} waypoints reached', flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
