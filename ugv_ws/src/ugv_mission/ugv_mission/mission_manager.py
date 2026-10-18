"""Mission manager: go to a goal clicked by the operator, then return to the start pose.

State machine: IDLE -> GOTO -> RETURN -> IDLE (or FAILED).

The start pose is captured from map->base_link, and also from odom->base_link. The odom frame is
continuous (EKF of visual odometry + IMU), the map frame can jump when SLAM accepts a loop closure,
including a wrong one (perceptual aliasing).

Localization-jump guard: map->odom is watched. A jump larger than `jump_trans_m` / `jump_yaw_rad`
between two consecutive checks means the map frame moved under the robot, so a goal given in map
coordinates no longer points where the operator meant. Then:
  * during GOTO: the goal is abandoned and the robot goes home (start pose rebuilt from the
    continuous odom-frame copy); the mission result is ABORTED_LOCALIZATION_JUMP;
  * during RETURN: the home goal is recomputed from the odom-frame copy; the result is
    COMPLETED_JUMP_DETECTED (it arrived, but the map is not to be trusted).
A mission is reported as SUCCESS on /ugv/mission_result only if no jump was seen.

Goals are accepted on /ugv/goal (PoseStamped, frame map).
"""
import math
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


class State(Enum):
    IDLE = 'IDLE'
    GOTO = 'GOTO'
    RETURN = 'RETURN'
    FAILED = 'FAILED'


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('goal_topic', '/ugv/goal')
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('jump_trans_m', 0.4)
        self.declare_parameter('jump_yaw_rad', 0.2)
        self.declare_parameter('jump_guard', True)

        g = lambda n: self.get_parameter(n).value
        self.map_frame, self.odom_frame, self.base_frame = g('map_frame'), g('odom_frame'), g('base_frame')
        self.max_retries = g('max_retries')
        self.jump_trans, self.jump_yaw, self.jump_guard = g('jump_trans_m'), g('jump_yaw_rad'), g('jump_guard')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.state = State.IDLE
        self.start_pose = None        # PoseStamped in map
        self.start_odom = None        # (x, y, yaw) in odom, continuous
        self.goal_pose = None
        self.retries = 0
        self.goal_handle = None
        self.token = 0                # identifies the active Nav2 goal; stale callbacks are ignored
        self.last_map_odom = None
        self.jump_seen = False
        self.goal_aborted = False
        self.degraded = False         # home target is built from the odom-frame copy
        self.last_result = ''

        self.status_pub = self.create_publisher(String, '/ugv/mission_status', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.result_pub = self.create_publisher(String, '/ugv/mission_result', latched)
        self.start_pub = self.create_publisher(PoseStamped, '/ugv/start_pose', latched)
        self.create_subscription(PoseStamped, g('goal_topic'), self.on_goal, 10)
        self.create_service(Trigger, '/ugv/abort', self.on_abort)
        self.create_service(Trigger, '/ugv/return_home', self.on_return_home)
        self.create_service(Trigger, '/ugv/set_start', self.on_set_start)
        self.create_timer(0.5, self.tick)
        self.create_timer(0.1, self.watch_localization)

    # ---- start pose and frames --------------------------------------------
    def lookup(self, target, source):
        try:
            t = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time()).transform
        except TransformException:
            return None
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    def capture_start(self) -> bool:
        m = self.lookup(self.map_frame, self.base_frame)
        o = self.lookup(self.odom_frame, self.base_frame)
        if m is None or o is None:
            return False
        self.start_pose = self.make_pose(*m)
        self.start_odom = o
        self.degraded = False
        self.start_pub.publish(self.start_pose)
        self.get_logger().info(f'start pose captured: map ({m[0]:.2f}, {m[1]:.2f})')
        return True

    @staticmethod
    def make_pose_msg(x, y, yaw, frame):
        p = PoseStamped()
        p.header.frame_id = frame
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return p

    def make_pose(self, x, y, yaw):
        return self.make_pose_msg(x, y, yaw, self.map_frame)

    def home_in_map(self):
        """Home target in the map frame. After a jump it is rebuilt through the current map->odom."""
        if not self.degraded:
            return self.start_pose
        mo = self.lookup(self.map_frame, self.odom_frame)
        if mo is None or self.start_odom is None:
            return self.start_pose
        c, s = math.cos(mo[2]), math.sin(mo[2])
        x = c * self.start_odom[0] - s * self.start_odom[1] + mo[0]
        y = s * self.start_odom[0] + c * self.start_odom[1] + mo[1]
        return self.make_pose(x, y, wrap(self.start_odom[2] + mo[2]))

    def tick(self):
        if self.start_pose is None and not self.capture_start():
            self.publish_status('waiting for map->base_link to capture start pose')
            return
        self.publish_status(f'jumps={int(self.jump_seen)}' if self.jump_seen else '')

    # ---- localization guard ------------------------------------------------
    def watch_localization(self):
        cur = self.lookup(self.map_frame, self.odom_frame)
        if cur is None:
            return
        prev, self.last_map_odom = self.last_map_odom, cur
        if prev is None or not self.jump_guard:
            return
        d = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
        a = abs(wrap(cur[2] - prev[2]))
        if d > self.jump_trans or a > self.jump_yaw:
            self.on_jump(d, a)

    def on_jump(self, d, a):
        if self.state not in (State.GOTO, State.RETURN):
            self.get_logger().info(f'map->odom jump while {self.state.value}: {d:.2f} m, {math.degrees(a):.0f} deg')
            return
        self.get_logger().error(
            f'LOCALIZATION JUMP during {self.state.value}: {d:.2f} m, {math.degrees(a):.0f} deg')
        self.jump_seen = True
        self.degraded = True
        if self.state == State.GOTO:
            self.goal_aborted = True
            self.state = State.RETURN
        self.cancel_current()
        self.retries = 0
        self.send_goal(self.home_in_map())

    # ---- inputs ---------------------------------------------------------
    def reset_flags(self):
        self.jump_seen = self.goal_aborted = False
        self.degraded = False
        self.retries = 0

    def on_goal(self, msg: PoseStamped):
        if self.start_pose is None:
            self.get_logger().warn('goal rejected: start pose not captured yet')
            return
        if self.state in (State.GOTO, State.RETURN):
            self.get_logger().warn(f'goal rejected: mission busy ({self.state.value})')
            return
        self.goal_pose = msg
        self.goal_pose.header.frame_id = msg.header.frame_id or self.map_frame
        self.reset_flags()
        self.state = State.GOTO
        self.send_goal(self.goal_pose)

    def on_abort(self, _req, resp):
        self.cancel_current()
        self.state = State.IDLE
        resp.success, resp.message = True, 'aborted'
        return resp

    def on_set_start(self, _req, resp):
        if self.state in (State.GOTO, State.RETURN):
            resp.success, resp.message = False, f'mission busy ({self.state.value})'
        elif self.capture_start():
            if self.state == State.FAILED:
                self.state = State.IDLE
            resp.success, resp.message = True, 'start pose set to current pose'
        else:
            resp.success, resp.message = False, 'map->base_link not available'
        return resp

    def on_return_home(self, _req, resp):
        if self.start_pose is None:
            resp.success, resp.message = False, 'start pose unknown'
            return resp
        self.cancel_current()
        self.reset_flags()
        self.state = State.RETURN
        self.send_goal(self.home_in_map())
        resp.success, resp.message = True, 'returning to start'
        return resp

    # ---- navigation -----------------------------------------------------
    def send_goal(self, pose: PoseStamped):
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.fail('navigate_to_pose action server unavailable')
            return
        self.token += 1
        token = self.token
        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        fut = self.nav.send_goal_async(goal)
        fut.add_done_callback(lambda f, t=token: self.on_goal_response(f, t))
        self.get_logger().info(f'{self.state.value}: sent goal')

    def on_goal_response(self, fut, token):
        if token != self.token:
            return
        self.goal_handle = fut.result()
        if not self.goal_handle.accepted:
            self.fail('goal rejected by Nav2')
            return
        self.goal_handle.get_result_async().add_done_callback(lambda f, t=token: self.on_result(f, t))

    def goal_for_state(self):
        return self.goal_pose if self.state == State.GOTO else self.home_in_map()

    def on_result(self, fut, token):
        if token != self.token:           # result of a goal we already cancelled or replaced
            return
        status = fut.result().status
        self.goal_handle = None
        if self.state == State.IDLE:      # aborted by operator
            return
        if status == 4:                   # SUCCEEDED
            if self.state == State.GOTO:
                self.get_logger().info('goal reached, returning to start')
                self.retries = 0
                self.state = State.RETURN
                self.send_goal(self.home_in_map())
            else:
                self.finish()
            return
        if self.retries < self.max_retries:
            self.retries += 1
            self.get_logger().warn(
                f'{self.state.value} failed (status {status}), retry {self.retries}')
            self.send_goal(self.goal_for_state())
        elif self.state == State.GOTO:
            # Could not reach the goal: still try to get the robot home.
            self.get_logger().error('goal unreachable, returning to start')
            self.goal_aborted = True
            self.retries = 0
            self.state = State.RETURN
            self.send_goal(self.home_in_map())
        else:
            self.fail('could not return to start')

    def finish(self):
        if self.goal_aborted:
            result = 'ABORTED_LOCALIZATION_JUMP' if self.jump_seen else 'ABORTED_GOAL_UNREACHABLE'
        elif self.jump_seen:
            result = 'COMPLETED_JUMP_DETECTED'
        else:
            result = 'SUCCESS'
        self.get_logger().info(f'mission finished: {result}')
        self.last_result = result
        self.state = State.IDLE
        self.result_pub.publish(String(data=result))

    def cancel_current(self):
        self.token += 1                   # invalidates callbacks of the goal being cancelled
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def fail(self, reason: str):
        self.get_logger().error(reason)
        self.state = State.FAILED
        self.last_result = 'FAILED'
        self.result_pub.publish(String(data=f'FAILED: {reason}'))

    def publish_status(self, extra: str = ''):
        msg = String()
        msg.data = f'{self.state.value} {extra}'.strip()
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
