#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class LegJointCmdBridge(Node):
    """
    Subscribe:  /ambula/left_leg_joint_cmd   (sensor_msgs/JointState)
    Publish:    /ambula/left_hip_position_cmd   (std_msgs/Float64)
                /ambula/left_knee_position_cmd  (std_msgs/Float64)

    Assumption: JointState.position is in radians (ROS standard).
    """

    def __init__(self):
        super().__init__("left_leg_joint_cmd_bridge")

        # ---------- params ----------
        self.declare_parameter("in_topic", "/ambula/left_leg_joint_cmd")
        self.declare_parameter("hip_topic", "/ambula/left_hip_position_cmd")
        self.declare_parameter("knee_topic", "/ambula/left_knee_position_cmd")

        self.declare_parameter("hip_name", "left_hip")
        self.declare_parameter("knee_name", "left_knee")

        self.in_topic = self.get_parameter("in_topic").value
        self.hip_topic = self.get_parameter("hip_topic").value
        self.knee_topic = self.get_parameter("knee_topic").value

        self.hip_name = self.get_parameter("hip_name").value
        self.knee_name = self.get_parameter("knee_name").value

        # ---------- pubs/sub ----------
        self.sub = self.create_subscription(JointState, self.in_topic, self.cb, 10)
        self.pub_hip = self.create_publisher(Float64, self.hip_topic, 10)
        self.pub_knee = self.create_publisher(Float64, self.knee_topic, 10)

        self.get_logger().info(f"Sub: {self.in_topic}")
        self.get_logger().info(f"Pub hip: {self.hip_topic} (joint='{self.hip_name}')")
        self.get_logger().info(f"Pub knee: {self.knee_topic} (joint='{self.knee_name}')")

    def cb(self, msg: JointState):
        # build name -> index
        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        # helper: safely get position by joint name
        def get_pos(jn: str):
            i = name_to_idx.get(jn, None)
            if i is None:
                return None
            if i >= len(msg.position):
                return None
            return msg.position[i]

        hip_pos = get_pos(self.hip_name)
        knee_pos = get_pos(self.knee_name)

        if hip_pos is None or knee_pos is None:
            self.get_logger().warn(
                f"Missing joint in JointState. "
                f"Have={list(msg.name)} need=({self.hip_name},{self.knee_name})"
            )
            return

        m = Float64()
        m.data = float(hip_pos)
        self.pub_hip.publish(m)

        m2 = Float64()
        m2.data = float(knee_pos)
        self.pub_knee.publish(m2)


def main():
    rclpy.init()
    node = LegJointCmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
