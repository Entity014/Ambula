import math
import py_trees
from py_trees.common import Status

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

import transforms3d.euler as euler  # <--- ใช้ transforms3d


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class FeedForward(py_trees.behaviour.Behaviour):
    """
    BT Leaf: publish cmd_vel forward->backward one cycle then SUCCESS.
    Uses real dt from ROS clock (robust to tick_period jitter).

    Modification:
      - Removed "IMU safety as FAILURE"
      - Added "Pitch trigger" to early-exit with SUCCESS (switch to next BT state)
      - IMU pitch computed via transforms3d.euler.quat2euler
    """

    def __init__(self, name: str, node,
                 cmd_vel_topic="/cmd_vel",
                 control_hz=100.0,          # nominal loop for dt clamp reference
                 v_peak=0.20,
                 forward_time=0.35,
                 backward_time=0.35,
                 ramp_time=0.10,
                 yaw_rate=0.0,

                 # -----------------------------
                 # Pitch trigger (state switch)
                 # -----------------------------
                 use_pitch_trigger=True,
                 imu_topic="/imu/data",
                 pitch_trigger_deg=12.0,     # threshold to end early
                 pitch_trigger_mode="either",# "either" | "positive" | "negative"
                 pitch_hold_s=0.10,          # must exceed threshold continuously

                 # transforms3d convention
                 # axes string: see transforms3d docs (e.g. 'sxyz', 'szyx', ...)
                 euler_axes="sxyz",

                 dt_max_scale=3.0):          # allow up to 3x nominal dt before clamping
        super().__init__(name)
        self.node = node

        # pubs/subs
        self.pub = self.node.create_publisher(Twist, cmd_vel_topic, 10)

        # -----------------------------
        # Pitch trigger setup
        # -----------------------------
        self.use_pitch_trigger = bool(use_pitch_trigger)
        self.pitch_rad = 0.0
        self.pitch_trigger = math.radians(float(pitch_trigger_deg))
        self.pitch_trigger_mode = str(pitch_trigger_mode)
        self.pitch_hold_s = max(0.0, float(pitch_hold_s))
        self._pitch_over_time = 0.0

        # transforms3d euler convention
        self.euler_axes = str(euler_axes)

        if self.use_pitch_trigger:
            self.node.create_subscription(Imu, imu_topic, self._cb_imu, 10)

        # -----------------------------
        # profile params
        # -----------------------------
        self.hz = float(control_hz)
        self.dt_nominal = 1.0 / max(1e-6, self.hz)
        self.dt_max = max(self.dt_nominal, dt_max_scale * self.dt_nominal)

        self.v_peak = float(v_peak)
        self.Tf = float(forward_time)
        self.Tb = float(backward_time)
        self.Tr = max(1e-3, float(ramp_time))
        self.wz = float(yaw_rate)

        # internal
        self.phase = "IDLE"
        self.t_in_phase = 0.0
        self._last_time = None  # seconds (float)

    # -----------------------------
    # py_trees lifecycle
    # -----------------------------
    def initialise(self):
        self.phase = "FORWARD"
        self.t_in_phase = 0.0
        self._pitch_over_time = 0.0
        self._last_time = self._now_s()

        self.node.get_logger().info(
            f"[{self.name}] START FeedForward\n"
            f"  cmd_vel_topic     : {self.pub.topic_name}\n"
            f"  nominal_hz        : {self.hz:.1f} Hz\n"
            f"  v_peak            : {self.v_peak:.3f} m/s\n"
            f"  forward_time      : {self.Tf:.3f} s\n"
            f"  backward_time     : {self.Tb:.3f} s\n"
            f"  ramp_time         : {self.Tr:.3f} s\n"
            f"  yaw_rate          : {self.wz:.3f} rad/s\n"
            f"  pitch_trigger     : {self.use_pitch_trigger}\n"
            f"  pitch_trigger_deg : {math.degrees(self.pitch_trigger):.1f} deg\n"
            f"  pitch_mode        : {self.pitch_trigger_mode}\n"
            f"  pitch_hold_s      : {self.pitch_hold_s:.2f} s\n"
            f"  euler_axes        : {self.euler_axes}\n"
            f"  dt_nominal        : {self.dt_nominal:.4f} s, dt_max_clamp: {self.dt_max:.4f} s"
        )

        self._publish(0.0, 0.0)

    def terminate(self, new_status):
        self.node.get_logger().info(f"[{self.name}] TERMINATE → {new_status}")
        self._publish(0.0, 0.0)

    # -----------------------------
    # time / imu
    # -----------------------------
    def _now_s(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _cb_imu(self, msg: Imu):
        """
        Use transforms3d to convert quaternion -> euler.
        Important:
          - transforms3d expects quaternion order (w, x, y, z) :contentReference[oaicite:2]{index=2}
          - msg.orientation is (x, y, z, w) fields, so we must reorder
        """
        q = msg.orientation
        qw, qx, qy, qz = q.w, q.x, q.y, q.z  # reorder to (w, x, y, z)

        # quat2euler returns 3 angles (ai, aj, ak) matching axes sequence
        # default axes='sxyz' :contentReference[oaicite:3]{index=3}
        ax, ay, az = euler.quat2euler((qw, qx, qy, qz), axes=self.euler_axes)

        # For 'sxyz', ay corresponds to rotation about y (often used as "pitch")
        self.pitch_rad = float(ay)

    # -----------------------------
    # helpers
    # -----------------------------
    def _publish(self, vx: float, wz: float):
        tw = Twist()
        tw.linear.x = float(vx)
        tw.angular.z = float(wz)
        self.pub.publish(tw)

    def _trapezoid(self, t: float, T: float, peak: float) -> float:
        if T <= 1e-6:
            return 0.0
        Tr = min(self.Tr, 0.49 * T)
        if t < 0.0:
            return 0.0
        if t < Tr:
            return peak * (t / Tr)
        if t > (T - Tr):
            return peak * max(0.0, (T - t) / Tr)
        return peak

    def _step_dt(self) -> float:
        now = self._now_s()
        if self._last_time is None:
            self._last_time = now
            return 0.0

        dt = now - self._last_time
        self._last_time = now

        if dt < 0.0:
            self.node.get_logger().warn(f"[{self.name}] time went backwards (dt={dt:.6f}s), reset dt=0")
            return 0.0

        if dt > self.dt_max:
            self.node.get_logger().warn(
                f"[{self.name}] dt too large ({dt:.4f}s) clamp to {self.dt_max:.4f}s"
            )
            dt = self.dt_max

        return dt

    def _pitch_over_threshold(self) -> bool:
        p = self.pitch_rad
        th = self.pitch_trigger
        mode = self.pitch_trigger_mode
        if mode == "positive":
            return p > th
        if mode == "negative":
            return p < -th
        return abs(p) < th

    # -----------------------------
    # main tick
    # -----------------------------
    def update(self):
        dt = self._step_dt()

        # 1) Pitch-trigger: end early (SUCCESS) to switch BT state
        if self.use_pitch_trigger:
            if self._pitch_over_threshold():
                self._pitch_over_time += dt
            else:
                self._pitch_over_time = 0.0

            if self._pitch_over_time >= self.pitch_hold_s:
                self.node.get_logger().warn(
                    f"[{self.name}] PITCH trigger → early SUCCESS | "
                    f"pitch={math.degrees(self.pitch_rad):.1f} deg, "
                    f"th={math.degrees(self.pitch_trigger):.1f} deg, "
                    f"hold={self.pitch_hold_s:.2f}s"
                )
                self.phase = "DONE"
                self._publish(0.0, 0.0)
                return Status.SUCCESS

        # 2) Normal feedforward profile (forward -> backward -> SUCCESS)
        if self.phase == "FORWARD":
            self.t_in_phase += dt
            vx = self._trapezoid(self.t_in_phase, self.Tf, +self.v_peak)
            self._publish(vx, self.wz)

            if self.t_in_phase >= self.Tf:
                self.node.get_logger().info(f"[{self.name}] FORWARD done ({self.Tf:.2f}s) → BACKWARD")
                self.phase = "BACKWARD"
                self.t_in_phase = 0.0
            return Status.RUNNING

        if self.phase == "BACKWARD":
            self.t_in_phase += dt
            vx = self._trapezoid(self.t_in_phase, self.Tb, -self.v_peak)
            self._publish(vx, self.wz)

            if self.t_in_phase >= self.Tb:
                self.node.get_logger().info(f"[{self.name}] BACKWARD done ({self.Tb:.2f}s) → SUCCESS")
                self.phase = "DONE"
                self._publish(0.0, 0.0)
                return Status.SUCCESS
            return Status.RUNNING

        self._publish(0.0, 0.0)
        return Status.SUCCESS
