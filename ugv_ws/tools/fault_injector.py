"""Fault injection: own the map->odom transform and apply artificial localization jumps.

Run with the stack launched using publish_map_tf:=false (so RTAB-Map does not publish map->odom).
It publishes map->odom (identity unless a jump was injected), imitating a SLAM that accepts a
wrong loop closure and moves the map frame under the robot.

  /fault/inject_jump  std_msgs/Float32MultiArray [dx, dy, dyaw]  jump = rotate by dyaw about the
                      map origin, then translate by (dx, dy); accumulates
  /fault/reset        std_msgs/Empty  back to identity
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Empty, Float32MultiArray
from tf2_ros import TransformBroadcaster


class Injector(Node):
    def __init__(self):
        super().__init__('fault_injector', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.x = self.y = self.yaw = 0.0
        self.br = TransformBroadcaster(self)
        self.create_subscription(Float32MultiArray, '/fault/inject_jump', self.on_jump, 10)
        self.create_subscription(Empty, '/fault/reset', self.on_reset, 10)
        self.create_timer(1 / 30.0, self.publish)

    def on_jump(self, m):
        dx, dy, dyaw = (list(m.data) + [0.0, 0.0, 0.0])[:3]
        c, s = math.cos(dyaw), math.sin(dyaw)
        self.x, self.y = c * self.x - s * self.y + dx, s * self.x + c * self.y + dy
        self.yaw += dyaw
        self.get_logger().warn(f'INJECTED JUMP dx={dx:.2f} dy={dy:.2f} dyaw={math.degrees(dyaw):.0f} deg')

    def on_reset(self, _m):
        self.x = self.y = self.yaw = 0.0
        self.get_logger().info('fault reset')

    def publish(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.br.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(Injector())


if __name__ == '__main__':
    main()
