#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateHardwareOrder(Node):
    def __init__(self):
        super().__init__("joint_state_hardware_order")

        # ---------- declare parameters ----------
        self.declare_parameter(
            "joint_order",
            [
                "left_waist", "left_hip", "left_knee", "left_wheel",
                "right_waist", "right_hip", "right_knee", "right_wheel",
            ],
        )

        self.declare_parameter(
            "remove_names",
            [
                "world_to_base",
                "left_passive_wheel_joint",
                "right_passive_wheel_joint",
            ],
        )

        self.declare_parameter("input_topic", "/joint_states")
        self.declare_parameter("output_topic", "/joint_states/hardware")

        # ---------- read parameters ----------
        self.joint_order = list(
            self.get_parameter("joint_order").get_parameter_value().string_array_value
        )

        self.remove_names = set(
            self.get_parameter("remove_names").get_parameter_value().string_array_value
        )

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )

        # ---------- ROS interfaces ----------
        self.sub = self.create_subscription(
            JointState, input_topic, self.cb, 10
        )
        self.pub = self.create_publisher(
            JointState, output_topic, 10
        )

        self.get_logger().info("JointState hardware reorder node started")
        self.get_logger().info(f"Input topic : {input_topic}")
        self.get_logger().info(f"Output topic: {output_topic}")
        self.get_logger().info(f"Joint order : {self.joint_order}")
        self.get_logger().info(f"Remove joints: {sorted(self.remove_names)}")

    def normalize_name(self, name: str) -> str:
        # left_hip_joint -> left_hip
        return name[:-6] if name.endswith("_joint") else name

    def cb(self, msg: JointState):
        data = {}

        for i, raw_name in enumerate(msg.name):
            if raw_name in self.remove_names:
                continue

            n = self.normalize_name(raw_name)

            pos = msg.position[i] if i < len(msg.position) else 0.0
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            eff = msg.effort[i]   if i < len(msg.effort) else 0.0

            data[n] = (pos, vel, eff)

        out = JointState()
        out.header = msg.header
        out.name = list(self.joint_order)

        out.position = []
        out.velocity = []
        out.effort = []

        for j in self.joint_order:
            pos, vel, eff = data.get(j, (0.0, 0.0, 0.0))
            out.position.append(pos)
            out.velocity.append(vel)
            out.effort.append(eff)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = JointStateHardwareOrder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
