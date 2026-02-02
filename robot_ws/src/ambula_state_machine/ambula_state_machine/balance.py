import math
import py_trees
from py_trees.common import Status

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import transforms3d.euler as euler

import numpy as np

try:
    from scipy.linalg import solve_continuous_are
    _HAVE_SCIPY = True
except Exception:
    _HAVE_SCIPY = False


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class PDBalance(py_trees.behaviour.Behaviour):
    """
    BT Leaf: Balance state (continuous).
    - Sub IMU, compute pitch via transforms3d.quat2euler
    - PD control -> publish cmd_vel (linear.x) to keep pitch near 0

    Notes:
      - This leaf returns RUNNING continuously (until external BT switches state)
      - Quaternion order for transforms3d is (w, x, y, z)
    """

    def __init__(self, name: str, node,
                 cmd_vel_topic="/cmd_vel",
                 imu_topic="/imu/data",
                 euler_axes="sxyz",

                 # PD gains (start conservative แล้วค่อยเพิ่ม)
                 kp_pitch=1.2,          # [ (m/s)/rad ]  mapping pitch -> base velocity
                 kd_pitch=0.08,         # [ (m/s)/(rad/s) ]

                 # command limits
                 v_limit=0.35,          # [m/s]
                 wz_cmd=0.0,            # [rad/s] (ถ้าจะเลี้ยวค่อยต่อจาก planner ทีหลัง)

                 # safety soft-stop (optional)
                 pitch_soft_deg=35.0,   # ถ้าเอนเกินนี้ -> สั่งหยุด (แต่ยัง RUNNING)
                 ):
        super().__init__(name)
        self.node = node

        self.pub = self.node.create_publisher(Twist, cmd_vel_topic, 10)
        self.node.create_subscription(Imu, imu_topic, self._cb_imu, 10)

        self.euler_axes = str(euler_axes)

        self.kp = float(kp_pitch)
        self.kd = float(kd_pitch)
        self.v_limit = abs(float(v_limit))
        self.wz_cmd = float(wz_cmd)

        self.pitch_soft = math.radians(float(pitch_soft_deg))

        # states from IMU
        self.pitch = 0.0
        self.pitch_rate = 0.0
        self._have_imu = False

        # time for numerical derivative fallback (กรณี imu.angular_velocity ไม่เสถียร)
        self._last_pitch = 0.0
        self._last_time = None

    def initialise(self):
        self._last_time = self._now_s()
        self._last_pitch = self.pitch
        self.node.get_logger().info(
            f"[{self.name}] START Balance | kp={self.kp:.3f}, kd={self.kd:.3f}, "
            f"v_limit={self.v_limit:.2f} m/s, euler_axes={self.euler_axes}"
        )
        self._publish(0.0, self.wz_cmd)

    def terminate(self, new_status):
        self.node.get_logger().info(f"[{self.name}] TERMINATE → {new_status}")
        self._publish(0.0, 0.0)

    def _now_s(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _publish(self, vx: float, wz: float):
        tw = Twist()
        tw.linear.x = float(vx)
        tw.angular.z = float(wz)
        self.pub.publish(tw)

    def _cb_imu(self, msg: Imu):
        q = msg.orientation
        qw, qx, qy, qz = q.w, q.x, q.y, q.z  # reorder to (w, x, y, z)

        ax, ay, az = euler.quat2euler((qw, qx, qy, qz), axes=self.euler_axes)

        # สำหรับ 'sxyz' โดยทั่วไป ay มักถูกใช้เป็น pitch (แกน y)
        self.pitch = float(ay)

        # ใช้ gyro แกนที่สอดคล้องกับ pitch rate
        # (ในหลาย setup pitch คือ rotation about Y -> ใช้ angular_velocity.y)
        # ถ้าทิศกลับด้าน ค่อย flip sign ที่นี่ทีเดียว
        self.pitch_rate = float(msg.angular_velocity.y)

        self._have_imu = True

    def update(self):
        # ถ้า IMU ยังไม่มา อย่าสั่งอะไรเสี่ยง ๆ
        if not self._have_imu:
            self._publish(0.0, self.wz_cmd)
            return Status.RUNNING

        # soft-stop ถ้าเอียงมากเกิน (กันพัง) — ยัง RUNNING เพื่อให้ BT มีโอกาสไป recovery leaf อื่น
        if abs(self.pitch) > self.pitch_soft:
            self.node.get_logger().warn(
                f"[{self.name}] pitch too large ({math.degrees(self.pitch):.1f} deg) -> soft stop"
            )
            self._publish(0.0, 0.0)
            return Status.RUNNING

        # PD: อยากให้ pitch -> 0
        # สัญญาณ vx ให้ “ดันฐาน” ไปทิศที่ลด pitch (อาจต้องสลับเครื่องหมายตาม convention จริง)
        vx = -(self.kp * self.pitch + self.kd * self.pitch_rate)
        vx = clamp(vx, -self.v_limit, +self.v_limit)

        self._publish(vx, self.wz_cmd)
        return Status.RUNNING

def _lqr_continuous(A, B, Q, R):
    """
    Continuous-time LQR:
      minimize ∫ (x^T Q x + u^T R u) dt
      u = -K x
    If SciPy exists: solve CARE. Else: simple iterative approximation.
    """
    if _HAVE_SCIPY:
        P = solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ (B.T @ P)
        return K, P

    # ---- fallback: iterative "Kleinman-like" approach (rough but usable) ----
    # Start with a stabilizing K0 (small), then iterate:
    # P from Lyapunov approx; K <- R^-1 B^T P
    K = np.zeros((B.shape[1], A.shape[0]), dtype=float)
    P = np.eye(A.shape[0], dtype=float)

    # small step to bias stability
    K[0, 0] = 0.1
    K[0, 1] = 0.05

    alpha = 0.2
    for _ in range(200):
        # Closed-loop
        Ac = A - B @ K
        # crude Riccati residual gradient step:
        # Pdot = Ac^T P + P Ac + Q + K^T R K  (want = 0)
        resid = (Ac.T @ P + P @ Ac + Q + K.T @ R @ K)
        P = P - alpha * resid
        # keep P symmetric
        P = 0.5 * (P + P.T)
        # update K
        K_new = np.linalg.inv(R) @ (B.T @ P)
        # smooth
        K = 0.7 * K + 0.3 * K_new

    return K, P

class LQRBalance(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node,
                 cmd_vel_topic="/cmd_vel",
                 imu_topic="/imu/data",
                 odom_topic="/odom",
                 euler_axes="sxyz",

                 # model (theta_ddot = a*theta + b*theta_dot + c*v)
                 a=12.0,
                 b=-2.0,
                 c=20.0,

                 # velocity loop approx: v_dot = (u - v)/tau_v
                 tau_v=0.20,          # [s]  ลองเริ่ม 0.2

                 # LQR weights
                 q_pitch=25.0,
                 q_pitch_rate=2.0,
                 q_v=3.0,             # << เพิ่มลงโทษความเร็ว
                 r_vx=1.0,

                 v_limit=0.35,
                 wz_cmd=0.0,
                 pitch_soft_deg=35.0,

                 pitch_sign=+1.0,
                 pitch_rate_sign=+1.0,
                 vx_sign=+1.0):
        super().__init__(name)
        self.node = node

        self.pub = self.node.create_publisher(Twist, cmd_vel_topic, 10)
        self.node.create_subscription(Imu, imu_topic, self._cb_imu, 10)
        self.node.create_subscription(Odometry, odom_topic, self._cb_odom, 10)

        self.euler_axes = str(euler_axes)

        self.a = float(a)
        self.b = float(b)
        self.c = float(c)
        self.tau_v = max(1e-3, float(tau_v))

        self.q_pitch = float(q_pitch)
        self.q_pitch_rate = float(q_pitch_rate)
        self.q_v = float(q_v)
        self.r_vx = float(r_vx)

        self.v_limit = abs(float(v_limit))
        self.wz_cmd = float(wz_cmd)
        self.pitch_soft = math.radians(float(pitch_soft_deg))

        self.pitch_sign = float(pitch_sign)
        self.pitch_rate_sign = float(pitch_rate_sign)
        self.vx_sign = float(vx_sign)

        self.pitch = 0.0
        self.pitch_rate = 0.0
        self.v = 0.0                 # << เพิ่ม
        self._have_imu = False
        self._have_odom = False       # << เพิ่ม

        self.K = None  # shape (1,3)

    def _cb_odom(self, msg: Odometry):
        # ใช้ความเร็วฐานจริง
        self.v = float(msg.twist.twist.linear.x)
        self._have_odom = True

    def initialise(self):
        # 3-state A,B
        A = np.array([[0.0, 1.0, 0.0],
                      [self.a, self.b, self.c],
                      [0.0, 0.0, -1.0/self.tau_v]], dtype=float)

        B = np.array([[0.0],
                      [0.0],
                      [1.0/self.tau_v]], dtype=float)

        Q = np.diag([self.q_pitch, self.q_pitch_rate, self.q_v]).astype(float)
        R = np.array([[self.r_vx]], dtype=float)

        self.K, _P = _lqr_continuous(A, B, Q, R)

        self.node.get_logger().info(
            f"[{self.name}] START LQR(3-state) | axes={self.euler_axes} "
            f"| a={self.a:.3f}, b={self.b:.3f}, c={self.c:.3f}, tau_v={self.tau_v:.3f} "
            f"| Q=diag({self.q_pitch:.2f},{self.q_pitch_rate:.2f},{self.q_v:.2f}), R={self.r_vx:.2f} "
            f"| K={self.K.flatten()}"
        )
        self._publish(0.0, self.wz_cmd)

    def update(self):
        if (not self._have_imu) or (not self._have_odom) or (self.K is None):
            self._publish(0.0, self.wz_cmd)
            return Status.RUNNING

        if abs(self.pitch) > self.pitch_soft:
            self.node.get_logger().warn(
                f"[{self.name}] pitch too large ({math.degrees(self.pitch):.1f} deg) -> soft stop"
            )
            self._publish(0.0, 0.0)
            return Status.RUNNING

        x = np.array([[self.pitch],
                      [self.pitch_rate],
                      [self.v]], dtype=float)

        vx_cmd = -float((self.K @ x).item())
        vx_cmd = self.vx_sign * vx_cmd
        vx_cmd = clamp(vx_cmd, -self.v_limit, +self.v_limit)

        self._publish(vx_cmd, self.wz_cmd)
        return Status.RUNNING
