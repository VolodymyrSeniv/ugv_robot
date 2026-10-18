"""Publish the robot pose in the map frame (map->base_link) on /ugv/pose for the web UI.

TF carries several transforms at different rates, which a browser client cannot throttle safely,
so the lookup is done here and the UI just subscribes to one small PoseStamped topic.
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class RobotPosePub(Node):
    def __init__(self):
        super().__init__('robot_pose_pub')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(PoseStamped, '/ugv/pose', 10)
        self.create_timer(0.1, self.tick)

    def tick(self):
        try:
            t = self.buf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return
        m = PoseStamped()
        m.header.stamp = t.header.stamp
        m.header.frame_id = self.map_frame
        m.pose.position.x = t.transform.translation.x
        m.pose.position.y = t.transform.translation.y
        m.pose.orientation = t.transform.rotation
        self.pub.publish(m)


def main():
    rclpy.init()
    node = RobotPosePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
