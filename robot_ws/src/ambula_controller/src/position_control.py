#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class PositionPDToVref(Node):
    """
    PD position controller -> velocity reference (v_ref) publisher.

    - Sub:  /odom  (nav_msgs/Odometry)
    - Pub:  /cmd_vel (geometry_msgs/Twist)
            linear.x  = v_ref
            angular.z = 0.0

    Control law (1D along x):
        e = x_goal - x
        v_ref = Kp * e + Kd * (0 - v_meas)   # D term uses measured velocity to add damping
    """

    def __init__(self):
        super().__init__("position_pd_to_vref")

        # ---- parameters ----
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("x_goal", 0.0)      # target position (m) in odom frame
        self.declare_parameter("kp", 0.8)          # [1/s]
        self.declare_parameter("kd", 0.2)          # [-]  (effectively damping)
        self.declare_parameter("max_v", 0.6)       # [m/s] saturation
        self.declare_parameter("deadband", 0.01)   # [m]  stop if close enough
        self.declare_parameter("stale_timeout", 0.5)  # [s] safety: if odom not updated -> command 0

        # ---- state ----
        self.last_odom_time = None
        self.x = 0.0
        self.vx = 0.0

        # ---- QoS ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        odom_topic = self.get_parameter("odom_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # publish rate
        self.timer = self.create_timer(0.02, self.control_step)  # 50 Hz

        self.get_logger().info(
            f"PositionPDToVref started. sub={odom_topic}, pub={cmd_vel_topic}"
        )

    def odom_cb(self, msg: Odometry):
        self.x = float(msg.pose.pose.position.x)
        self.vx = float(msg.twist.twist.linear.x)
        self.last_odom_time = self.get_clock().now()

    def control_step(self):
        # safety: if odom stale -> stop
        stale_timeout = float(self.get_parameter("stale_timeout").value)
        now = self.get_clock().now()
        if self.last_odom_time is None or (now - self.last_odom_time).nanoseconds * 1e-9 > stale_timeout:
            self.publish_cmd(0.0)
            return

        x_goal = float(self.get_parameter("x_goal").value)
        kp = float(self.get_parameter("kp").value)
        kd = float(self.get_parameter("kd").value)
        max_v = float(self.get_parameter("max_v").value)
        deadband = float(self.get_parameter("deadband").value)

        e = x_goal - self.x

        # deadband on position
        if abs(e) < deadband:
            self.publish_cmd(0.0)
            return

        # PD -> velocity reference
        # D term uses measured velocity for damping (no derivative of noisy position needed)
        v_ref = kp * e + kd * (0.0 - self.vx)

        # saturate
        v_ref = clamp(v_ref, -max_v, max_v)

        self.publish_cmd(v_ref)

    def publish_cmd(self, v_ref: float):
        msg = Twist()
        msg.linear.x = float(v_ref)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0  # ขอแค่ v_ref ก่อน
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PositionPDToVref()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
