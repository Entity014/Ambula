#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateDemo(Node):
    def __init__(self):
        super().__init__("joint_state_demo")

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(0.02, self.timer_cb)  # 50 Hz

        self.t = 0.0

        self.joint_names = [
            "left_hip_joint",
            "right_hip_joint",
            "left_knee_joint",
            "right_knee_joint",
            "left_wheel_joint",
            "right_wheel_joint",
        ]

        self.get_logger().info("JointState demo publisher started")

    def timer_cb(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # ตัวอย่าง motion ง่าย ๆ
        hip = -0.3 * math.sin(self.t)
        knee = -0.6 * math.sin(self.t)
        wheel = 2.0 * math.sin(self.t)

        msg.position = [
            hip,
            hip,
            knee,
            knee,
            wheel,
            wheel,
        ]

        # velocity / effort ใส่หรือไม่ใส่ก็ได้
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6

        self.pub.publish(msg)
        self.t += 0.02


def main():
    rclpy.init()
    node = JointStateDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
