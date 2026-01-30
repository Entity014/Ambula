#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from transforms3d.euler import euler2quat


def yaw_to_quat(yaw: float) -> Quaternion:
    # transforms3d returns (w, x, y, z)
    w, x, y, z = euler2quat(0.0, 0.0, yaw, axes='sxyz')  # roll,pitch,yaw
    q = Quaternion()
    q.x = float(x)
    q.y = float(y)
    q.z = float(z)
    q.w = float(w)
    return q


class DiffDriveKinematic(Node):
    """
    Subscribe JointState -> extract wheel angular velocities -> compute diff-drive odom
    Publish nav_msgs/Odometry on /odom/data_raw (or param).
    """

    def __init__(self):
        super().__init__("diff_drive_kinematic")

        # ---------------- Parameters ----------------
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/odom/data_raw")

        self.declare_parameter("left_wheel_joint", "left_wheel")
        self.declare_parameter("right_wheel_joint", "right_wheel")

        self.declare_parameter("wheel_radius", 0.075)       # m
        self.declare_parameter("wheel_separation", 0.40)    # m (track width)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        # If JointState.velocity is not populated, allow fallback to position differentiation
        self.declare_parameter("use_position_if_no_velocity", True)

        # Simple covariances (tune later)
        self.declare_parameter("pose_cov_xy", 0.02)         # m^2
        self.declare_parameter("pose_cov_yaw", 0.05)        # rad^2
        self.declare_parameter("twist_cov_v", 0.05)         # (m/s)^2
        self.declare_parameter("twist_cov_yawrate", 0.05)   # (rad/s)^2

        # read params
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.left_joint = self.get_parameter("left_wheel_joint").value
        self.right_joint = self.get_parameter("right_wheel_joint").value
        self.r = float(self.get_parameter("wheel_radius").value)
        self.L = float(self.get_parameter("wheel_separation").value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.use_pos_fallback = bool(self.get_parameter("use_position_if_no_velocity").value)

        self.pose_cov_xy = float(self.get_parameter("pose_cov_xy").value)
        self.pose_cov_yaw = float(self.get_parameter("pose_cov_yaw").value)
        self.twist_cov_v = float(self.get_parameter("twist_cov_v").value)
        self.twist_cov_yawrate = float(self.get_parameter("twist_cov_yawrate").value)

        # ---------------- State ----------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_stamp = None  # rclpy.time.Time

        # For position fallback
        self.last_left_pos: Optional[float] = None
        self.last_right_pos: Optional[float] = None

        # ---------------- Pub/Sub ----------------
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)
        self.sub_js = self.create_subscription(JointState, self.joint_state_topic, self.cb_joint_state, 10)

        self.get_logger().info(
            "wheel_jointstate_to_odom started\n"
            f"  joint_state_topic: {self.joint_state_topic}\n"
            f"  odom_topic       : {self.odom_topic}\n"
            f"  left_joint       : {self.left_joint}\n"
            f"  right_joint      : {self.right_joint}\n"
            f"  r={self.r:.4f} m, L={self.L:.4f} m"
        )

    def _find_joint(self, msg: JointState, joint_name: str) -> Tuple[Optional[int], str]:
        """Return (index, error_message_if_any)."""
        try:
            idx = msg.name.index(joint_name)
            return idx, ""
        except ValueError:
            return None, f"Joint '{joint_name}' not found in JointState.name"

    def _get_wheel_omega(self, msg: JointState, idx: int, side: str) -> Optional[float]:
        """
        Return wheel angular velocity (rad/s).
        Prefer msg.velocity[idx]. If not available, optionally compute from position.
        """
        # 1) velocity field
        if idx < len(msg.velocity):
            omega = msg.velocity[idx]
            if omega is not None:
                return float(omega)

        # 2) fallback to position differentiation
        if not self.use_pos_fallback:
            return None
        if idx >= len(msg.position):
            return None

        pos = float(msg.position[idx])
        if side == "L":
            if self.last_left_pos is None or self.last_stamp is None:
                self.last_left_pos = pos
                return None
            dt = (self.get_clock().now() - self.last_stamp).nanoseconds * 1e-9
            if dt <= 0.0:
                return None
            omega = (pos - self.last_left_pos) / dt
            self.last_left_pos = pos
            return omega

        if side == "R":
            if self.last_right_pos is None or self.last_stamp is None:
                self.last_right_pos = pos
                return None
            dt = (self.get_clock().now() - self.last_stamp).nanoseconds * 1e-9
            if dt <= 0.0:
                return None
            omega = (pos - self.last_right_pos) / dt
            self.last_right_pos = pos
            return omega

        return None

    def cb_joint_state(self, msg: JointState):
        # time
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if self.last_stamp is None:
            self.last_stamp = stamp
            return

        dt = (stamp - self.last_stamp).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 1.0:
            # dt too small/large → reset integration (กันกระโดดเวลา)
            self.last_stamp = stamp
            return

        li, le = self._find_joint(msg, self.left_joint)
        ri, re = self._find_joint(msg, self.right_joint)

        if li is None or ri is None:
            # log แบบไม่สแปม
            self.get_logger().warn(le if li is None else re)
            self.last_stamp = stamp
            return

        omega_l = self._get_wheel_omega(msg, li, "L")
        omega_r = self._get_wheel_omega(msg, ri, "R")
        if omega_l is None or omega_r is None:
            self.last_stamp = stamp
            return

        # ---------------- Differential drive kinematics ----------------
        # v = r*(wL + wR)/2
        # yaw_rate = r*(wR - wL)/L
        v = self.r * (omega_l + omega_r) * 0.5
        yaw_rate = self.r * (omega_r - omega_l) / self.L

        # integrate pose (unicycle model)
        # x += v*cos(yaw)*dt ; y += v*sin(yaw)*dt ; yaw += yaw_rate*dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += yaw_rate * dt

        # normalize yaw to [-pi, pi]
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi

        # ---------------- Publish Odometry ----------------
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quat(self.yaw)

        # pose covariance (6x6 row-major)
        pose_cov = [0.0] * 36
        pose_cov[0] = self.pose_cov_xy      # x
        pose_cov[7] = self.pose_cov_xy      # y
        pose_cov[35] = self.pose_cov_yaw    # yaw
        odom.pose.covariance = pose_cov

        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = float(yaw_rate)

        twist_cov = [0.0] * 36
        twist_cov[0] = self.twist_cov_v          # v
        twist_cov[35] = self.twist_cov_yawrate   # yaw_rate
        odom.twist.covariance = twist_cov

        self.pub_odom.publish(odom)

        self.last_stamp = stamp


def main():
    rclpy.init()
    node = DiffDriveKinematic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
