#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
    w, x, y, z = euler2quat(0.0, 0.0, yaw, axes="sxyz")  # roll,pitch,yaw
    q = Quaternion()
    q.x = float(x)
    q.y = float(y)
    q.z = float(z)
    q.w = float(w)
    return q


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def unwrap_delta(dtheta: float) -> float:
    """Map delta angle to (-pi, pi] to avoid wrap spikes when encoder wraps at 2π."""
    return (dtheta + math.pi) % (2.0 * math.pi) - math.pi


class DiffDriveKinematic(Node):
    """
    Subscribe JointState -> extract wheel angular velocities -> compute diff-drive odom
    Publish nav_msgs/Odometry and publish wheel JointState (computed velocity) in a separate topic.

    Improvements:
      - Prefer using position differentiation (encoder position) to compute wheel omega
      - Use msg.header.stamp consistently for dt (do NOT use now())
      - Optional unwrap for absolute encoders that wrap at 2π
      - Low-pass filter omega to suppress differentiation noise
    """

    def __init__(self):
        super().__init__("diff_drive_kinematic")

        # ---------------- Parameters ----------------
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/odom/data_raw")

        # publish computed wheel state (JointState)
        self.declare_parameter("wheel_state_topic", "/wheel_joint_states")
        self.declare_parameter("publish_wheel_joint_state", True)

        self.declare_parameter("left_wheel_joint", "left_wheel")
        self.declare_parameter("right_wheel_joint", "right_wheel")

        self.declare_parameter("wheel_radius", 0.075)       # m
        self.declare_parameter("wheel_separation", 0.40)    # m (track width)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        # --- Source selection ---
        # If True: compute omega from JointState.position (recommended when msg.velocity is noisy/unstable)
        # If False: try msg.velocity first; fallback to position if missing
        self.declare_parameter("prefer_position", True)

        # --- Filtering ---
        # Low-pass for wheel omega. 0..1 (higher = follow faster, lower = smoother)
        self.declare_parameter("omega_lpf_alpha", 0.35)

        # --- Encoder wrap handling ---
        # If your wheel position wraps at 2π (absolute encoder), enable unwrap
        # If your wheel position is cumulative (continuous), set False
        self.declare_parameter("unwrap_position", True)

        # Robustness
        self.declare_parameter("dt_max_s", 1.0)             # reset integration if dt too large

        # Simple covariances (tune later)
        self.declare_parameter("pose_cov_xy", 0.02)         # m^2
        self.declare_parameter("pose_cov_yaw", 0.05)        # rad^2
        self.declare_parameter("twist_cov_v", 0.05)         # (m/s)^2
        self.declare_parameter("twist_cov_yawrate", 0.05)   # (rad/s)^2

        # read params
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value

        self.wheel_state_topic = self.get_parameter("wheel_state_topic").value
        self.publish_wheel_js = bool(self.get_parameter("publish_wheel_joint_state").value)

        self.left_joint = self.get_parameter("left_wheel_joint").value
        self.right_joint = self.get_parameter("right_wheel_joint").value
        self.r = float(self.get_parameter("wheel_radius").value)
        self.L = float(self.get_parameter("wheel_separation").value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.prefer_position = bool(self.get_parameter("prefer_position").value)
        self.omega_alpha = float(self.get_parameter("omega_lpf_alpha").value)

        self.unwrap_position = bool(self.get_parameter("unwrap_position").value)
        self.dt_max_s = float(self.get_parameter("dt_max_s").value)

        self.pose_cov_xy = float(self.get_parameter("pose_cov_xy").value)
        self.pose_cov_yaw = float(self.get_parameter("pose_cov_yaw").value)
        self.twist_cov_v = float(self.get_parameter("twist_cov_v").value)
        self.twist_cov_yawrate = float(self.get_parameter("twist_cov_yawrate").value)

        # ---------------- State ----------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_stamp: Optional[rclpy.time.Time] = None

        # For per-wheel position-diff
        self.last_left_pos: Optional[float] = None
        self.last_right_pos: Optional[float] = None
        self.last_left_stamp: Optional[rclpy.time.Time] = None
        self.last_right_stamp: Optional[rclpy.time.Time] = None

        # Filtered omegas
        self.omega_l_filt: Optional[float] = None
        self.omega_r_filt: Optional[float] = None

        # ---------------- Pub/Sub ----------------
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)
        self.pub_wheel = self.create_publisher(JointState, self.wheel_state_topic, 10) if self.publish_wheel_js else None
        self.sub_js = self.create_subscription(JointState, self.joint_state_topic, self.cb_joint_state, 10)

        self.get_logger().info(
            "diff_drive_kinematic started\n"
            f"  joint_state_topic         : {self.joint_state_topic}\n"
            f"  odom_topic                : {self.odom_topic}\n"
            f"  publish_wheel_joint_state : {self.publish_wheel_js}\n"
            f"  wheel_state_topic         : {self.wheel_state_topic}\n"
            f"  left_joint                : {self.left_joint}\n"
            f"  right_joint               : {self.right_joint}\n"
            f"  r={self.r:.4f} m, L={self.L:.4f} m\n"
            f"  prefer_position           : {self.prefer_position}\n"
            f"  unwrap_position           : {self.unwrap_position}\n"
            f"  omega_lpf_alpha           : {self.omega_alpha:.3f}"
        )

    def _find_joint(self, msg: JointState, joint_name: str) -> Tuple[Optional[int], str]:
        try:
            idx = msg.name.index(joint_name)
            return idx, ""
        except ValueError:
            return None, f"Joint '{joint_name}' not found in JointState.name"

    def _lpf(self, x: float, x_f: Optional[float]) -> float:
        """Simple first-order IIR low-pass."""
        a = self.omega_alpha
        if x_f is None:
            return x
        return a * x + (1.0 - a) * x_f

    def _omega_from_velocity_field(self, msg: JointState, idx: int) -> Optional[float]:
        if idx < len(msg.velocity):
            try:
                return float(msg.velocity[idx])
            except Exception:
                return None
        return None

    def _omega_from_position_diff(
        self,
        pos: float,
        last_pos: Optional[float],
        stamp: rclpy.time.Time,
        last_stamp: Optional[rclpy.time.Time],
    ) -> Tuple[Optional[float], float, rclpy.time.Time]:
        """
        Compute omega = d(pos)/dt using message stamp.
        Returns: (omega_or_None, new_last_pos, new_last_stamp)
        """
        if last_pos is None or last_stamp is None:
            return None, pos, stamp

        dt = (stamp - last_stamp).nanoseconds * 1e-9
        if dt <= 0.0 or dt > self.dt_max_s:
            return None, pos, stamp

        dpos = pos - last_pos
        if self.unwrap_position:
            dpos = unwrap_delta(dpos)  # avoid 2π wrap spikes

        omega = dpos / dt
        return omega, pos, stamp

    def _publish_wheel_joint_state(self, header, pos_l: float, pos_r: float, omega_l: float, omega_r: float):
        if not self.publish_wheel_js or self.pub_wheel is None:
            return

        js = JointState()
        js.header = header
        js.name = [self.left_joint, self.right_joint]
        js.position = [float(pos_l), float(pos_r)]
        js.velocity = [float(omega_l), float(omega_r)]
        # effort optional
        self.pub_wheel.publish(js)

    def cb_joint_state(self, msg: JointState):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)

        # init time
        if self.last_stamp is None:
            self.last_stamp = stamp
            return

        # global dt for pose integration
        dt = (stamp - self.last_stamp).nanoseconds * 1e-9
        if dt <= 0.0 or dt > self.dt_max_s:
            self.last_stamp = stamp
            return

        li, le = self._find_joint(msg, self.left_joint)
        ri, re = self._find_joint(msg, self.right_joint)
        if li is None or ri is None:
            self.get_logger().warn(le if li is None else re)
            self.last_stamp = stamp
            return

        # ---------------- Get wheel omegas ----------------
        omega_l: Optional[float] = None
        omega_r: Optional[float] = None

        if not self.prefer_position:
            omega_l = self._omega_from_velocity_field(msg, li)
            omega_r = self._omega_from_velocity_field(msg, ri)

        # if prefer_position OR velocity missing -> use position diff
        if omega_l is None or omega_r is None:
            if li >= len(msg.position) or ri >= len(msg.position):
                self.last_stamp = stamp
                return

            pos_l = float(msg.position[li])
            pos_r = float(msg.position[ri])

            omega_l_raw, self.last_left_pos, self.last_left_stamp = self._omega_from_position_diff(
                pos_l, self.last_left_pos, stamp, self.last_left_stamp
            )
            omega_r_raw, self.last_right_pos, self.last_right_stamp = self._omega_from_position_diff(
                pos_r, self.last_right_pos, stamp, self.last_right_stamp
            )

            if omega_l_raw is None or omega_r_raw is None:
                self.last_stamp = stamp
                return

            # low-pass filter to reduce differentiation noise
            self.omega_l_filt = self._lpf(omega_l_raw, self.omega_l_filt)
            self.omega_r_filt = self._lpf(omega_r_raw, self.omega_r_filt)
            omega_l = self.omega_l_filt
            omega_r = self.omega_r_filt
        else:
            # if using msg.velocity and we also want to publish wheel JS, we still need positions for message
            if li < len(msg.position) and ri < len(msg.position):
                pos_l = float(msg.position[li])
                pos_r = float(msg.position[ri])
            else:
                pos_l = 0.0
                pos_r = 0.0

        # sanity
        if omega_l is None or omega_r is None:
            self.last_stamp = stamp
            return

        # ---------------- Publish wheel JointState (separate topic) ----------------
        # (pos_l/pos_r defined in both branches above)
        self._publish_wheel_joint_state(msg.header, pos_l, pos_r, omega_l, omega_r)

        # ---------------- Differential drive kinematics ----------------
        # v = r*(wL + wR)/2
        # yaw_rate = r*(wR - wL)/L
        v = self.r * (omega_l + omega_r) * 0.5
        yaw_rate = self.r * (omega_r - omega_l) / self.L

        # integrate pose (unicycle model)
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw = wrap_pi(self.yaw + yaw_rate * dt)

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
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
