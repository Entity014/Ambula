#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32


def quat_to_R_wb(qx: float, qy: float, qz: float, qw: float):
    """
    Rotation matrix R_wb (body/base -> world) from quaternion (x,y,z,w).
    Assumes quaternion is normalized (we normalize anyway for safety).
    """
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-12:
        return ((1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.0, 0.0, 1.0))
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz

    # Standard quaternion to rotation matrix
    return (
        (1.0 - 2.0*(yy + zz), 2.0*(xy - wz),       2.0*(xz + wy)),
        (2.0*(xy + wz),       1.0 - 2.0*(xx + zz), 2.0*(yz - wx)),
        (2.0*(xz - wy),       2.0*(yz + wx),       1.0 - 2.0*(xx + yy)),
    )


def mat_vec(R, v):
    return (
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    )


def mat_T(R):
    return (
        (R[0][0], R[1][0], R[2][0]),
        (R[0][1], R[1][1], R[2][1]),
        (R[0][2], R[1][2], R[2][2]),
    )


class HeightEstimator(Node):
    """
    Height estimator using:
      - Wheel center points in base frame from your FK (PointStamped)
      - IMU orientation quaternion (sensor_msgs/Imu)
    """

    def __init__(self):
        super().__init__("height_estimator")

        # -------- Parameters --------
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("left_wheel_center_topic", "/fk/left_wheel_center")
        self.declare_parameter("right_wheel_center_topic", "/fk/right_wheel_center")
        self.declare_parameter("height_topic", "/state/height")

        self.declare_parameter("wheel_radius", 0.08)  # meters
        self.declare_parameter("use_both_wheels", True)

        # simple EMA smoothing
        self.declare_parameter("enable_smoothing", True)
        self.declare_parameter("ema_alpha", 0.2)  # 0..1 (higher = less smooth)

        self.imu_topic = self.get_parameter("imu_topic").value
        self.l_topic = self.get_parameter("left_wheel_center_topic").value
        self.r_topic = self.get_parameter("right_wheel_center_topic").value
        self.out_topic = self.get_parameter("height_topic").value

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.use_both = bool(self.get_parameter("use_both_wheels").value)

        self.enable_smoothing = bool(self.get_parameter("enable_smoothing").value)
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)

        # -------- State --------
        self.R_wb = None  # latest rotation base->world
        self.left_p = None  # (x,y,z) wheel center in base
        self.right_p = None
        self.h_ema: Optional[float] = None

        # -------- ROS I/O --------
        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.cb_imu, 50)
        self.sub_l = self.create_subscription(PointStamped, self.l_topic, self.cb_left, 50)
        self.sub_r = self.create_subscription(PointStamped, self.r_topic, self.cb_right, 50)

        self.pub_h = self.create_publisher(Float32, self.out_topic, 10)

        self.timer = self.create_timer(0.01, self.on_timer)  # 100 Hz

        self.get_logger().info(
            f"Height estimator started.\n"
            f"  imu_topic={self.imu_topic}\n"
            f"  left_wheel_center_topic={self.l_topic}\n"
            f"  right_wheel_center_topic={self.r_topic}\n"
            f"  out={self.out_topic}\n"
            f"  wheel_radius={self.wheel_radius:.4f} m"
        )

    def cb_imu(self, msg: Imu):
        q = msg.orientation
        self.R_wb = quat_to_R_wb(q.x, q.y, q.z, q.w)

    def cb_left(self, msg: PointStamped):
        p = msg.point
        self.left_p = (p.x, p.y, p.z)

    def cb_right(self, msg: PointStamped):
        p = msg.point
        self.right_p = (p.x, p.y, p.z)

    def compute_height_from_wheel_center(self, p_wheel_b: Tuple[float, float, float]) -> Optional[float]:
        if self.R_wb is None:
            return None

        # down direction of world expressed in base frame: z_down_b = R_bw * [0,0,-1]
        R_bw = mat_T(self.R_wb)
        z_down_b = mat_vec(R_bw, (0.0, 0.0, -1.0))

        # contact point in base frame
        p_contact_b = (
            p_wheel_b[0] + self.wheel_radius * z_down_b[0],
            p_wheel_b[1] + self.wheel_radius * z_down_b[1],
            p_wheel_b[2] + self.wheel_radius * z_down_b[2],
        )

        # rotate into world and take z
        p_contact_w = mat_vec(self.R_wb, p_contact_b)
        h = -p_contact_w[2]  # ground z=0
        return h

    def on_timer(self):
        if self.R_wb is None:
            return

        hs = []

        if self.left_p is not None:
            hL = self.compute_height_from_wheel_center(self.left_p)
            if hL is not None and math.isfinite(hL):
                hs.append(hL)

        if self.use_both and (self.right_p is not None):
            hR = self.compute_height_from_wheel_center(self.right_p)
            if hR is not None and math.isfinite(hR):
                hs.append(hR)

        if not hs:
            return

        h = sum(hs) / len(hs)

        # optional smoothing (EMA)
        if self.enable_smoothing:
            if self.h_ema is None:
                self.h_ema = h
            else:
                a = self.ema_alpha
                self.h_ema = (1.0 - a) * self.h_ema + a * h
            h_out = self.h_ema
        else:
            h_out = h

        msg = Float32()
        msg.data = float(h_out)
        self.pub_h.publish(msg)


def main():
    rclpy.init()
    node = HeightEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
