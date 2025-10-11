#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState

from ambula_controller.kinematics import ik_leg, Link3, FootPose, JointAngles

class LegIKNode(Node):
    def __init__(self):
        super().__init__("leg_ik_node")

        # ---- parameters ----
        self.declare_parameter("left_joints",  ["left_waist","left_hip","left_knee"])
        self.declare_parameter("right_joints", ["right_waist","right_hip","right_knee"])
        self.declare_parameter("L1", 0.075)
        self.declare_parameter("L2", 0.200)
        self.declare_parameter("L3", 0.240)

        self.declare_parameter("left_desired_topic",  "/left_desired_pose")
        self.declare_parameter("right_desired_topic", "/right_desired_pose")
        self.declare_parameter("left_cmd_topic",      "/joint_states/left/command")
        self.declare_parameter("right_cmd_topic",     "/joint_states/right/command")

        self._update_params_from_server()
        self.add_on_set_parameters_callback(self._on_param_change)

        # ---- IO ----
        qos = rclpy.qos.qos_profile_sensor_data
        self.left_sub  = self.create_subscription(PointStamped,  self.left_desired_topic,  self._left_cb,  qos)
        self.right_sub = self.create_subscription(PointStamped,  self.right_desired_topic, self._right_cb, qos)

        self.left_pub  = self.create_publisher(JointState, self.left_cmd_topic,  10)
        self.right_pub = self.create_publisher(JointState, self.right_cmd_topic, 10)

        self.get_logger().info(
            f"IK ready | L1={self.link.L1:.3f} L2={self.link.L2:.3f} L3={self.link.L3:.3f} | "
            f"left:{self.left_joints} right:{self.right_joints}"
        )

    # ---------- params ----------
    def _update_params_from_server(self):
        self.left_joints  = list(self.get_parameter("left_joints").value)
        self.right_joints = list(self.get_parameter("right_joints").value)
        L1 = float(self.get_parameter("L1").value)
        L2 = float(self.get_parameter("L2").value)
        L3 = float(self.get_parameter("L3").value)
        self.link = Link3(L1, L2, L3)

        self.left_desired_topic  = self.get_parameter("left_desired_topic").value
        self.right_desired_topic = self.get_parameter("right_desired_topic").value
        self.left_cmd_topic      = self.get_parameter("left_cmd_topic").value
        self.right_cmd_topic     = self.get_parameter("right_cmd_topic").value

    def _on_param_change(self, params):
        changed = False
        for p in params:
            if p.name in ("L1","L2","L3") and p.type_ == p.TYPE_DOUBLE:
                setattr(self.link, p.name, p.value)
                changed = True
        return SetParametersResult(successful=True)

    # ---------- callbacks ----------
    def _left_cb(self, msg: PointStamped):
        self._handle_pose(msg, side="left",  joint_names=self.left_joints,  pub=self.left_pub)

    def _right_cb(self, msg: PointStamped):
        self._handle_pose(msg, side="right", joint_names=self.right_joints, pub=self.right_pub)

    def _handle_pose(self, msg: PointStamped, side: str, joint_names, pub):
        p = FootPose(x=float(msg.point.x), y=float(msg.point.y), z=float(msg.point.z))
        sol = ik_leg(p, self.link, side)   # ← เรียกจากไลบรารี
        if sol is None:
            self.get_logger().warning(f"[{side}] IK failed for ({p.x:.3f},{p.y:.3f},{p.z:.3f})")
            return

        out = JointState()
        out.header.stamp = msg.header.stamp
        out.name = list(joint_names)
        out.position = [sol.q1, sol.q2, sol.q3]
        pub.publish(out)

def main():
    rclpy.init()
    node = LegIKNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
