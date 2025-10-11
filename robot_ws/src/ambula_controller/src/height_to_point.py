#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped

class HeightToPointNode(Node):
    def __init__(self):
        super().__init__("height_to_point_node")

        # ---- Parameters ----
        self.declare_parameter("input_topic", "/height_input")
        self.declare_parameter("left_desired_topic",  "/left_desired_pose")
        self.declare_parameter("right_desired_topic", "/right_desired_pose")

        input_topic        = self.get_parameter("input_topic").value
        self.left_topic    = self.get_parameter("left_desired_topic").value
        self.right_topic   = self.get_parameter("right_desired_topic").value

        # ---- Subscriber ----
        self.create_subscription(Float32, input_topic, self.height_cb, 10)

        # ---- Publishers ----
        self.pub_left  = self.create_publisher(PointStamped, self.left_topic, 10)
        self.pub_right = self.create_publisher(PointStamped, self.right_topic, 10)

        self.get_logger().info(
            f"Subscribed to {input_topic}, publishing to {self.left_topic} and {self.right_topic}"
        )

    def height_cb(self, msg: Float32):
        h = -msg.data
        now = self.get_clock().now().to_msg()
        # self.get_logger().info(f"Received height: {h:.3f}")

        # ---- Left Point ----
        left_point = PointStamped()
        left_point.header.stamp = now
        left_point.header.frame_id = "base_link"
        left_point.point.x = -0.005
        left_point.point.y = 0.075
        left_point.point.z = h

        # ---- Right Point ----
        right_point = PointStamped()
        right_point.header.stamp = now
        right_point.header.frame_id = "base_link"
        right_point.point.x = -0.005
        right_point.point.y = -0.075
        right_point.point.z = h

        # Publish
        self.pub_left.publish(left_point)
        self.pub_right.publish(right_point)

def main():
    rclpy.init()
    node = HeightToPointNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
