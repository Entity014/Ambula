#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import numpy as np
from rclpy.node import Node

from sensor_msgs.msg import Imu
from ambula_interfaces.msg import ImuEuler

import transforms3d.euler as euler


class ImuOrientationToEuler(Node):
    def __init__(self):
        super().__init__("imu_orientation_to_euler")

        self.declare_parameter("imu_in_topic", "/imu/data")
        self.declare_parameter("imu_euler_out_topic", "/imu/euler_state")
        self.declare_parameter("axes", "sxyz")

        self.imu_in = self.get_parameter("imu_in_topic").value
        self.out_topic = self.get_parameter("imu_euler_out_topic").value
        self.axes = self.get_parameter("axes").value

        self.sub = self.create_subscription(Imu, self.imu_in, self.cb, 10)
        self.pub = self.create_publisher(ImuEuler, self.out_topic, 10)

        self.get_logger().info("Sub: %s" % self.imu_in)
        self.get_logger().info("Pub: %s" % self.out_topic)
        self.get_logger().info("axes=%s" % self.axes)

    def cb(self, msg: Imu):
        # ROS quat: (x,y,z,w)
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # transforms3d quat: (w,x,y,z)
        quat_wxyz = (qw, qx, qy, qz)

        try:
            roll, pitch, yaw = euler.quat2euler(quat_wxyz, axes=self.axes)
        except Exception as ex:
            self.get_logger().warn("quat2euler failed: %s" % str(ex))
            return

        out = ImuEuler()
        out.header = msg.header  # stamp + frame_id

        out.roll = float(np.rad2deg(roll))
        out.pitch = float(np.rad2deg(pitch))
        out.yaw = float(np.rad2deg(yaw))

        # rates จาก gyro โดยตรง (rad/s)
        out.roll_rate = float(msg.angular_velocity.x)
        out.pitch_rate = float(msg.angular_velocity.y)
        out.yaw_rate = float(msg.angular_velocity.z)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuOrientationToEuler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
