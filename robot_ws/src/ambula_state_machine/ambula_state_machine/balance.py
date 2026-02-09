#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
from py_trees.common import Status

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import transforms3d.euler as euler


try:
    from scipy.linalg import solve_continuous_are
    _HAVE_SCIPY = True
except Exception:
    _HAVE_SCIPY = False

import numpy as np
import math

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def quat_to_R_body_to_world(qx, qy, qz, qw) -> np.ndarray:
    # ROS Imu: (x,y,z,w)
    x, y, z, w = float(qx), float(qy), float(qz), float(qw)
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-9:
        return np.eye(3, dtype=float)
    x, y, z, w = x/n, y/n, z/n, w/n

    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z

    # standard quaternion -> R (body->world)
    return np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)],
        [2*(xy + wz),         1 - 2*(xx + zz),   2*(yz - wx)],
        [2*(xz - wy),         2*(yz + wx),       1 - 2*(xx + yy)],
    ], dtype=float)

def projected_gravity_body(qx, qy, qz, qw):
    # world gravity (ENU): +z up => gravity is (0,0,-1) per REP-103 :contentReference[oaicite:3]{index=3}
    g_world = np.array([0.0, 0.0, -1.0], dtype=float)

    R_bw = quat_to_R_body_to_world(qx, qy, qz, qw)  # body->world
    R_wb = R_bw.T                                  # world->body (inverse)

    g_body = R_wb @ g_world
    g_body /= (np.linalg.norm(g_body) + 1e-9)
    return float(g_body[0]), float(g_body[1]), float(g_body[2])

def pitch_from_projected_gravity(gx, gy, gz):
    """
    สำหรับ REP-103 (x forward, y left, z up) :contentReference[oaicite:4]{index=4}
    - ถ้าหุ่นก้มหน้า (pitch +) => gx จะ “เพิ่ม” ในหลาย setup
    สูตรที่เสถียร: pitch ≈ atan2(gx, -gz)
    (หากกลับทิศ ให้ปรับ pitch_sign)
    """
    return math.atan2(gx, -gz)


# ------------------------------------------------------------
# PID Balance Leaf (แทน PDBalance)
# ------------------------------------------------------------
class PIDBalance(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node,
                 cmd_vel_topic="/cmd_vel",
                 imu_topic="/imu/data",

                 kp_pitch=1.2,
                 ki_pitch=0.4,        # <-- NEW: เริ่มเล็ก ๆ เช่น 0.05-0.8 แล้วจูน
                 kd_pitch=0.08,
                 v_limit=0.35,
                 wz_cmd=0.0,
                 pitch_soft_deg=35.0,
                 pitch_trim_deg=2.0,
                 pitch_sign=+1.0,
                 pitch_rate_sign=+1.0,

                 # ---- NEW: anti-windup / integrator ----
                 i_limit=0.25,        # จำกัดค่า I term (หน่วยเดียวกับ pitch_err*ki) ป้องกัน windup
                 i_leak=0.0,          # 0..0.2 (optional) ช่วยคาย I ช้า ๆ กัน drift

                 # ---- minimum effective velocity + stop band ----
                 v_min=0.08,
                 stop_band_deg=1.0,
                 stop_rate_deg_s=6.0,
                 min_vel_only_when_need=True
                 ):
        super().__init__(name)
        self.node = node
        self.pub = self.node.create_publisher(Twist, cmd_vel_topic, 10)
        self.node.create_subscription(Imu, imu_topic, self._cb_imu, 10)

        self.kp = float(kp_pitch)
        self.ki = float(ki_pitch)
        self.kd = float(kd_pitch)

        self.v_limit = abs(float(v_limit))
        self.wz_cmd = float(wz_cmd)
        self.pitch_soft = math.radians(float(pitch_soft_deg))

        self.pitch_sign = float(pitch_sign)
        self.pitch_rate_sign = float(pitch_rate_sign)

        self.pitch = 0.0
        self.pitch_rate = 0.0
        self.pitch_trim = math.radians(pitch_trim_deg)
        self._have_imu = False

        self.v_min = abs(float(v_min))
        self.stop_band = math.radians(float(stop_band_deg))
        self.stop_rate = math.radians(float(stop_rate_deg_s))
        self.min_vel_only_when_need = bool(min_vel_only_when_need)

        # --- integrator state ---
        self.i_limit = abs(float(i_limit))
        self.i_leak = float(i_leak)
        self.i_state = 0.0

        # --- dt ---
        self._t_prev = None

    def _publish(self, vx: float, wz: float):
        tw = Twist()
        tw.linear.x = float(vx)
        tw.angular.z = float(wz)
        self.pub.publish(tw)

    def _cb_imu(self, msg: Imu):
        q = msg.orientation
        gx, gy, gz = projected_gravity_body(q.x, q.y, q.z, q.w)
        pitch = pitch_from_projected_gravity(gx, gy, gz)

        self.pitch = self.pitch_sign * float(pitch)
        self.pitch_rate = self.pitch_rate_sign * float(msg.angular_velocity.y)
        self._have_imu = True

    def initialise(self):
        self.node.get_logger().info(
            f"[{self.name}] START PID Balance | kp={self.kp:.3f}, ki={self.ki:.3f}, kd={self.kd:.3f}, v_limit={self.v_limit:.2f}"
        )
        self.i_state = 0.0
        self._t_prev = None
        self._publish(0.0, self.wz_cmd)

    def terminate(self, new_status):
        self.node.get_logger().info(f"[{self.name}] TERMINATE → {new_status}")
        self.i_state = 0.0
        self._t_prev = None
        self._publish(0.0, 0.0)

    def update(self):
        if not self._have_imu:
            self._publish(0.0, self.wz_cmd)
            return Status.RUNNING

        # dt จาก ROS clock
        t_now = self.node.get_clock().now()
        if self._t_prev is None:
            dt = 0.0
        else:
            dt = (t_now - self._t_prev).nanoseconds * 1e-9
        self._t_prev = t_now

        # soft stop
        if abs(self.pitch) > self.pitch_soft:
            self.node.get_logger().warn(
                f"[{self.name}] pitch too large ({math.degrees(self.pitch):.1f} deg) -> soft stop"
            )
            self.i_state = 0.0  # รีเซ็ต I กันพอก
            self._publish(0.0, 0.0)
            return Status.RUNNING

        pitch_err = self.pitch - self.pitch_trim

        # -------------------------
        # STOP BAND: allow true zero (และคาย I)
        # -------------------------
        if (abs(pitch_err) < self.stop_band) and (abs(self.pitch_rate) < self.stop_rate):
            # คาย I ให้กลับศูนย์เร็วขึ้น เพื่อหยุดนิ่งจริง
            self.i_state *= 0.5
            vx = 0.0
            self._publish(vx, self.wz_cmd)
            return Status.RUNNING

        # -------------------------
        # Integrator update (with leak)
        # -------------------------
        if dt > 0.0 and self.ki != 0.0:
            # integrate error
            self.i_state += pitch_err * dt

            # optional leak (ช่วย drift): i_state *= (1 - leak*dt)
            if self.i_leak > 0.0:
                self.i_state *= max(0.0, 1.0 - self.i_leak * dt)

            # clamp I state by limiting ki*i_state (anti-windup แบบ clamping)
            if self.i_limit > 0.0:
                i_term = self.ki * self.i_state
                i_term = clamp(i_term, -self.i_limit, +self.i_limit)
                self.i_state = i_term / self.ki

        # PID raw command
        vx_raw = -(self.kp * pitch_err + self.ki * self.i_state + self.kd * self.pitch_rate)

        # saturation
        vx = clamp(vx_raw, -self.v_limit, +self.v_limit)

        # -------------------------
        # Anti-windup เพิ่มเติม: ถ้า saturated และ I “ดันไปผิดทิศ” ให้หยุดสะสม
        # (แนวคิด anti-reset windup / clamping ที่ใช้กันทั่วไป)
        # -------------------------
        if dt > 0.0 and self.ki != 0.0:
            sat = (abs(vx_raw) > self.v_limit + 1e-9)
            if sat:
                # ถ้า error ทำให้อยากดันแรงขึ้นอีกในทิศเดียวกับ saturation -> ไม่ต้องสะสมเพิ่ม
                if (vx_raw > 0.0 and pitch_err < 0.0) or (vx_raw < 0.0 and pitch_err > 0.0):
                    # ดันทิศเดียวกับการอิ่มตัว
                    self.i_state -= pitch_err * dt  # rollback 1 step

        # -------------------------
        # Minimum effective velocity (dead-zone/stiction workaround)
        # -------------------------
        if self.v_min > 0.0:
            need_min = True
            if self.min_vel_only_when_need:
                need_min = abs(pitch_err) >= self.stop_band

            if need_min and (abs(vx) > 1e-6) and (abs(vx) < self.v_min):
                vx = math.copysign(self.v_min, vx)

        self._publish(vx, self.wz_cmd)
        self.node.get_logger().info(
            f"Pitch={math.degrees(self.pitch):.2f}deg err={math.degrees(pitch_err):.2f}deg "
            f"rate={math.degrees(self.pitch_rate):.2f}deg/s I={self.i_state:.4f} vx={vx:.3f}"
        )
        return Status.RUNNING


def lqr_continuous(A, B, Q, R):
    """
    Continuous-time LQR: minimize ∫ (x^T Q x + u^T R u) dt, u = -Kx
    """
    if not _HAVE_SCIPY:
        raise RuntimeError("SciPy is required for solve_continuous_are in this version.")
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ (B.T @ P)
    return K, P


def ipm_cartpole_linear_matrices(M, m, l, I, b, g=9.81):
    """
    Linearized inverted pendulum on a cart (upright equilibrium).
    State: [x, x_dot, theta, theta_dot]
    Input: F (horizontal force on cart)
    Reference form (common in control literature):
      denom = I*(M+m) + M*m*l^2

    A =
    [ 0, 1, 0, 0
      0, -((I+m l^2)*b)/denom, (m^2 g l^2)/denom, 0
      0, 0, 0, 1
      0, -(m l b)/denom, (m g l (M+m))/denom, 0 ]

    B =
    [ 0
      (I+m l^2)/denom
      0
      (m l)/denom ]
    """
    M = float(M); m = float(m); l = float(l); I = float(I); b = float(b); g = float(g)
    denom = I * (M + m) + M * m * (l ** 2)
    if denom <= 1e-9:
        raise ValueError("Invalid denom for cart-pole. Check M,m,l,I.")

    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, -((I + m*l*l)*b)/denom, (m*m*g*l*l)/denom, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, -(m*l*b)/denom, (m*g*l*(M + m))/denom, 0.0]
    ], dtype=float)

    B = np.array([
        [0.0],
        [(I + m*l*l)/denom],
        [0.0],
        [(m*l)/denom]
    ], dtype=float)

    return A, B


class LQRBalance(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node,
                 cmd_vel_topic="/cmd_vel",
                 imu_topic="/imu/data",
                 odom_topic="/odom",

                 M=8.0, m=12.0, l=0.35, I=0.9, b=1.0, g=9.81,
                 q_x=0.2, q_xdot=0.8, q_theta=60.0, q_theta_dot=6.0, r_force=0.8,
                 kF=35.0,
                 v_limit=0.35, wz_cmd=0.0,
                 use_x_position=False,
                 pitch_soft_deg=35.0,

                 pitch_sign=+1.0,
                 pitch_rate_sign=+1.0,
                 x_sign=+1.0,
                 v_sign=+1.0,
                 force_sign=+1.0):
        super().__init__(name)
        self.node = node

        self.pub = self.node.create_publisher(Twist, cmd_vel_topic, 10)
        self.node.create_subscription(Imu, imu_topic, self._cb_imu, 10)
        self.node.create_subscription(Odometry, odom_topic, self._cb_odom, 10)

        self.M, self.m, self.l, self.I, self.b, self.g = map(float, (M,m,l,I,b,g))
        self.q_x, self.q_xdot, self.q_theta, self.q_theta_dot = map(float, (q_x,q_xdot,q_theta,q_theta_dot))
        self.r_force = float(r_force)

        self.kF = max(1e-6, float(kF))
        self.v_limit = abs(float(v_limit))
        self.wz_cmd = float(wz_cmd)

        self.use_x_position = bool(use_x_position)
        self.pitch_soft = math.radians(float(pitch_soft_deg))

        self.pitch_sign = float(pitch_sign)
        self.pitch_rate_sign = float(pitch_rate_sign)
        self.x_sign = float(x_sign)
        self.v_sign = float(v_sign)
        self.force_sign = float(force_sign)

        self.theta = 0.0
        self.theta_dot = 0.0
        self.x = 0.0
        self.x_dot = 0.0
        self._have_imu = False
        self._have_odom = False

        self.K = None  # (1,4)
        self._fallback_pd = None

    def _publish(self, vx: float, wz: float):
        tw = Twist()
        tw.linear.x = float(vx)
        tw.angular.z = float(wz)
        self.pub.publish(tw)

    def _cb_imu(self, msg: Imu):
        q = msg.orientation
        gx, gy, gz = projected_gravity_body(q.x, q.y, q.z, q.w)
        pitch = pitch_from_projected_gravity(gx, gy, gz)

        self.theta = self.pitch_sign * float(pitch)
        self.theta_dot = self.pitch_rate_sign * float(msg.angular_velocity.y)
        self._have_imu = True

    def _cb_odom(self, msg: Odometry):
        self.x = self.x_sign * float(msg.pose.pose.position.x)
        self.x_dot = self.v_sign * float(msg.twist.twist.linear.x)
        self._have_odom = True

    def initialise(self):
        if not _HAVE_SCIPY:
            # fallback เป็น PD แบบง่าย (กันล่ม)
            self.node.get_logger().warn(f"[{self.name}] SciPy not found -> fallback to PD-like control")
            self._fallback_pd = {"kp": 1.2, "kd": 0.08}
            return

        A, B = ipm_cartpole_linear_matrices(
            M=self.M, m=self.m, l=self.l, I=self.I, b=self.b, g=self.g
        )
        Q = np.diag([self.q_x, self.q_xdot, self.q_theta, self.q_theta_dot]).astype(float)
        R = np.array([[self.r_force]], dtype=float)

        self.K, _P = lqr_continuous(A, B, Q, R)

        self.node.get_logger().info(
            f"[{self.name}] START LQR-IPM | use_x={self.use_x_position} | K={self.K.flatten()}"
        )
        self._publish(0.0, self.wz_cmd)

    def terminate(self, new_status):
        self.node.get_logger().info(f"[{self.name}] TERMINATE → {new_status}")
        self._publish(0.0, 0.0)

    def update(self):
        if not (self._have_imu and self._have_odom):
            self._publish(0.0, self.wz_cmd)
            return Status.RUNNING

        if abs(self.theta) > self.pitch_soft:
            self.node.get_logger().warn(
                f"[{self.name}] pitch too large ({math.degrees(self.theta):.1f} deg) -> soft stop"
            )
            self._publish(0.0, 0.0)
            return Status.RUNNING

        # ---- fallback PD ----
        if self._fallback_pd is not None:
            kp = self._fallback_pd["kp"]
            kd = self._fallback_pd["kd"]
            v_cmd = -(kp * self.theta + kd * self.theta_dot)
            v_cmd = clamp(v_cmd, -self.v_limit, +self.v_limit)
            self._publish(v_cmd, self.wz_cmd)
            return Status.RUNNING

        if self.K is None:
            self._publish(0.0, self.wz_cmd)
            return Status.RUNNING

        x_pos = self.x if self.use_x_position else 0.0
        X = np.array([[x_pos],
                      [self.x_dot],
                      [self.theta],
                      [self.theta_dot]], dtype=float)

        # Force from LQR
        F_cmd = -float((self.K @ X).item())
        F_cmd *= self.force_sign

        # Force -> velocity mapping (ต้องจูน kF)
        v_cmd = self.x_dot + (F_cmd / self.kF)
        v_cmd = clamp(v_cmd, -self.v_limit, +self.v_limit)

        self._publish(v_cmd, self.wz_cmd)
        return Status.RUNNING
