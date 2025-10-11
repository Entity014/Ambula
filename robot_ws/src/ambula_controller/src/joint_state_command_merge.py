#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from typing import List

def _safe_list(x: List[float] | None) -> List[float]:
    return list(x) if x is not None else []

class JointStateCommandMerge(Node):
    def __init__(self):
        super().__init__("joint_state_command_merge_node")

        # ---- Parameters ----
        self.declare_parameter("left_topic",  "/joint_states/left/command")
        self.declare_parameter("right_topic", "/joint_states/right/command")
        self.declare_parameter("output_topic", "/joint_states/command")

        self.left_topic   = self.get_parameter("left_topic").value
        self.right_topic  = self.get_parameter("right_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        # ---- Storage ----
        self.left_msg: JointState | None  = None
        self.right_msg: JointState | None = None

        # ---- Subscriptions ----
        qos = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(JointState, self.left_topic,  self.left_cb,  qos)
        self.create_subscription(JointState, self.right_topic, self.right_cb, qos)

        # ---- Publisher ----
        self.pub = self.create_publisher(JointState, self.output_topic, 10)

        self.get_logger().info(
            f"Aggregating {self.left_topic} + {self.right_topic} -> {self.output_topic}"
        )

    def left_cb(self, msg: JointState):
        self.left_msg = msg
        self.publish_combined()

    def right_cb(self, msg: JointState):
        self.right_msg = msg
        self.publish_combined()

    def _latest_stamp(self, a, b):
        # เปรียบเทียบด้วย (sec, nanosec)
        return a if (a.sec, a.nanosec) >= (b.sec, b.nanosec) else b

    def publish_combined(self):
        if self.left_msg is None or self.right_msg is None:
            return

        out = JointState()
        out.header.stamp = self._latest_stamp(self.left_msg.header.stamp,
                                              self.right_msg.header.stamp)
        out.header.frame_id = self.left_msg.header.frame_id or self.right_msg.header.frame_id

        out.name     = list(self.left_msg.name) + list(self.right_msg.name)
        out.position = list(self.left_msg.position) + list(self.right_msg.position)
        out.velocity = _safe_list(self.left_msg.velocity) + _safe_list(self.right_msg.velocity)
        out.effort   = _safe_list(self.left_msg.effort)   + _safe_list(self.right_msg.effort)

        self.pub.publish(out)

def main():
    rclpy.init()
    node = JointStateCommandMerge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
