"""Twist-only odometry from the velocity command, for the EKF.

The robot has no encoders. When visual odometry is lost the EKF has no velocity input and keeps
extrapolating its last estimate, so the pose drifts although the robot stands still. The velocity
command (/cmd_vel, after Nav2's smoother) is a loose but unbiased-in-sign velocity measurement:
it tells the filter "stopped" when stopped, and bridges short blind stretches while driving.
The covariance is large on purpose, so VO dominates whenever it is available.
"""
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class CmdVelOdom(Node):
    def __init__(self):
        super().__init__('cmd_vel_odom')
        self.declare_parameter('linear_var', 0.05)
        self.declare_parameter('angular_var', 0.1)
        self.declare_parameter('rate', 20.0)
        self.lin_var = self.get_parameter('linear_var').value
        self.ang_var = self.get_parameter('angular_var').value
        self.cmd = Twist()
        self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'cmd', m), 10)
        self.pub = self.create_publisher(Odometry, '/cmd_odom', 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)

    def tick(self):
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
        m.twist.twist.linear.x = self.cmd.linear.x
        m.twist.twist.angular.z = self.cmd.angular.z
        cov = [0.0] * 36
        cov[0] = cov[7] = self.lin_var
        cov[14] = cov[21] = cov[28] = 1e6
        cov[35] = self.ang_var
        m.twist.covariance = cov
        m.pose.covariance = [1e6 if i % 7 == 0 else 0.0 for i in range(36)]
        self.pub.publish(m)


def main():
    rclpy.init()
    node = CmdVelOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
