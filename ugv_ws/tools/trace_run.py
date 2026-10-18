"""Log ground truth vs VO vs EKF vs SLAM (map->base_link) once per second to CSV."""
import csv, math, sys
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Trace(Node):
    def __init__(self, path):
        super().__init__('trace_run', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.last = {}
        self.status = ''
        for name, topic in (('gt', '/ground_truth/odom'), ('vo', '/vo/odom'), ('ekf', '/odometry/filtered')):
            self.create_subscription(Odometry, topic, lambda m, n=name: self.last.__setitem__(n, (m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))), 10)
        self.create_subscription(String, '/ugv/mission_status', lambda m: setattr(self, 'status', m.data.split()[0]), 10)
        self.buf = Buffer(); TransformListener(self.buf, self)
        self.f = open(path, 'w', newline=''); self.w = csv.writer(self.f)
        self.w.writerow(['t', 'status', 'gt_x', 'gt_y', 'gt_yaw', 'vo_x', 'vo_y', 'vo_yaw', 'ekf_x', 'ekf_y', 'ekf_yaw', 'slam_x', 'slam_y', 'slam_yaw'])
        self.create_timer(1.0, self.tick)

    def tick(self):
        row = [round(self.get_clock().now().nanoseconds / 1e9, 1), self.status]
        for n in ('gt', 'vo', 'ekf'):
            row += [round(v, 3) for v in self.last.get(n, (float('nan'),) * 3)]
        try:
            t = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
            row += [round(t.translation.x, 3), round(t.translation.y, 3), round(yaw(t.rotation), 3)]
        except Exception:
            row += [float('nan')] * 3
        self.w.writerow(row); self.f.flush()


rclpy.init(); n = Trace(sys.argv[1])
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
