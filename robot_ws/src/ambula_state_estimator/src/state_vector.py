#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

# custom msgs
from ambula_interfaces.msg import ImuEuler, StateEstimation


class StateVectorNode(Node):
    """
    Build state vector x = [pitch, pitch_rate, linear_velocity, yaw_rate, height]
    from:
      - ImuEuler (pitch, pitch_rate, yaw_rate) : /imu/data_raw/euler
      - Odometry (linear velocity)            : /odom/unfiltered
      - Float32 height                        : /ambula/height
    """

    def __init__(self):
        super().__init__("state_vector_node")

        # ---------------- Params ----------------
        self.declare_parameter("imu_euler_topic", "/imu/data_raw/euler")
        self.declare_parameter("odom_topic", "/odom/unfiltered")
        self.declare_parameter("height_topic", "/ambula/height")
        self.declare_parameter("out_topic", "/state/estimation")

        self.declare_parameter("frame_id", "base_link")

        # publish rate
        self.declare_parameter("publish_hz", 100.0)

        # linear velocity selection
        # if true: v = sqrt(vx^2 + vy^2) else: v = vx
        self.declare_parameter("use_speed_magnitude", False)

        # if true: require all inputs received at least once before publishing
        self.declare_parameter("require_all_inputs", True)

        self.imu_topic = self.get_parameter("imu_euler_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.height_topic = self.get_parameter("height_topic").value
        self.out_topic = self.get_parameter("out_topic").value

        self.frame_id = self.get_parameter("frame_id").value
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.use_speed_magnitude = bool(self.get_parameter("use_speed_magnitude").value)
        self.require_all = bool(self.get_parameter("require_all_inputs").value)

        # ---------------- Latest cache ----------------
        self.last_imu: Optional[ImuEuler] = None
        self.last_odom: Optional[Odometry] = None
        self.last_height: Optional[Float32] = None

        # ---------------- ROS I/O ----------------
        self.sub_imu = self.create_subscription(ImuEuler, self.imu_topic, self.cb_imu, 50)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 50)
        self.sub_h = self.create_subscription(Float32, self.height_topic, self.cb_height, 50)

        self.pub_state = self.create_publisher(StateEstimation, self.out_topic, 10)

        period = 1.0 / max(1e-6, self.publish_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            "StateVectorNode started\n"
            f"  imu:    {self.imu_topic}\n"
            f"  odom:   {self.odom_topic}\n"
            f"  height: {self.height_topic}\n"
            f"  out:    {self.out_topic}\n"
            f"  hz:     {self.publish_hz}\n"
            f"  use_speed_magnitude: {self.use_speed_magnitude}\n"
            f"  require_all_inputs:  {self.require_all}"
        )

    # ---------------- Callbacks ----------------
    def cb_imu(self, msg: ImuEuler):
        self.last_imu = msg

    def cb_odom(self, msg: Odometry):
        self.last_odom = msg

    def cb_height(self, msg: Float32):
        self.last_height = msg

    # ---------------- Main publish loop ----------------
    def on_timer(self):
        if self.require_all:
            if self.last_imu is None or self.last_odom is None or self.last_height is None:
                return

        # If some inputs missing and not required, fill with NaN (or 0.0 if you prefer)
        pitch = float("nan")
        pitch_rate = float("nan")
        yaw_rate = float("nan")
        linear_velocity = float("nan")
        height = float("nan")

        # IMU
        if self.last_imu is not None:
            pitch = float(self.last_imu.pitch)
            pitch_rate = float(self.last_imu.pitch_rate)
            yaw_rate = float(self.last_imu.yaw_rate)

        # Odom velocity
        if self.last_odom is not None:
            vx = float(self.last_odom.twist.twist.linear.x)
            vy = float(self.last_odom.twist.twist.linear.y)
            if self.use_speed_magnitude:
                linear_velocity = math.sqrt(vx * vx + vy * vy)
            else:
                linear_velocity = vx

        # Height
        if self.last_height is not None:
            height = float(self.last_height.data)

        # Build msg
        out = StateEstimation()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id

        out.pitch = pitch
        out.pitch_rate = pitch_rate
        out.linear_velocity = linear_velocity
        out.yaw_rate = yaw_rate
        out.height = height

        self.pub_state.publish(out)


def main():
    rclpy.init()
    node = StateVectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
