#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import py_trees
from py_trees.common import Status

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

import transforms3d.euler as euler  # quat2euler


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class FeedForward(py_trees.behaviour.Behaviour):
    """
    BT Leaf: publish cmd_vel forward->backward one cycle then SUCCESS.
    Uses real dt from ROS clock (robust to tick_period jitter).

    Features:
      - Forward/Backward trapezoid profile
      - Optional "Pitch trigger" to early-exit with SUCCESS (switch to next BT state)
      - Robust IMU gating:
          * WAIT_IMU phase: do not evaluate pitch trigger until first IMU callback arrives
          * Optional imu_stale_timeout_s: if IMU becomes stale, disables trigger until refreshed

    Notes:
      - transforms3d expects quaternion order (w, x, y, z)
      - sensor_msgs/Imu orientation fields are (x, y, z, w)
      - Euler axes depends on your IMU frame & convention; default 'sxyz'
    """

    def __init__(
        self,
        name: str,
        node,
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
        pitch_trigger_deg=12.0,      # threshold to end early
        pitch_trigger_mode="either", # "either" | "positive" | "negative"
        pitch_hold_s=0.10,           # must exceed threshold continuously

        # transforms3d convention
        euler_axes="sxyz",

        # -----------------------------
        # Robustness knobs
        # -----------------------------
        wait_imu_on_start=True,      # if True: start in WAIT_IMU (no profile until IMU arrives)
        imu_wait_timeout_s=0.50,     # if IMU never arrives, after this allow profile (but still no trigger)
        imu_stale_timeout_s=0.25,    # if IMU older than this -> treat as stale (disable trigger)
        dt_max_scale=3.0,            # allow up to 3x nominal dt before clamping
    ):
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
        self.pitch_trigger_mode = str(pitch_trigger_mode).lower()
        self.pitch_hold_s = max(0.0, float(pitch_hold_s))
        self._pitch_over_time = 0.0

        # transforms3d euler convention
        self.euler_axes = str(euler_axes)

        # IMU gating state
        self._imu_received = False
        self._last_imu_time = None
        self.wait_imu_on_start = bool(wait_imu_on_start)
        self.imu_wait_timeout_s = max(0.0, float(imu_wait_timeout_s))
        self.imu_stale_timeout_s = max(0.0, float(imu_stale_timeout_s))

        if self.use_pitch_trigger or self.wait_imu_on_start:
            # subscribe if either trigger needs IMU or we want to WAIT_IMU phase
            self.node.create_subscription(Imu, imu_topic, self._cb_imu, 10)

        # -----------------------------
        # profile params
        # -----------------------------
        self.hz = float(control_hz)
        self.dt_nominal = 1.0 / max(1e-6, self.hz)
        self.dt_max = max(self.dt_nominal, float(dt_max_scale) * self.dt_nominal)

        self.v_peak = float(v_peak)
        self.Tf = max(0.0, float(forward_time))
        self.Tb = max(0.0, float(backward_time))
        self.Tr = max(1e-3, float(ramp_time))
        self.wz = float(yaw_rate)

        # internal
        self.phase = "IDLE"
        self.t_in_phase = 0.0
        self._last_time = None  # seconds (float)
        self._start_time = None  # seconds (float)

    # -----------------------------
    # py_trees lifecycle
    # -----------------------------
    def initialise(self):
        self._start_time = self._now_s()
        self._last_time = self._start_time
        self.t_in_phase = 0.0
        self._pitch_over_time = 0.0

        # Start phase
        if self.wait_imu_on_start and not self._imu_received:
            self.phase = "WAIT_IMU"
        else:
            self.phase = "FORWARD"

        self.node.get_logger().info(
            f"[{self.name}] START FeedForward\n"
            f"  cmd_vel_topic        : {self.pub.topic_name}\n"
            f"  nominal_hz           : {self.hz:.1f} Hz\n"
            f"  v_peak               : {self.v_peak:.3f} m/s\n"
            f"  forward_time         : {self.Tf:.3f} s\n"
            f"  backward_time        : {self.Tb:.3f} s\n"
            f"  ramp_time            : {self.Tr:.3f} s\n"
            f"  yaw_rate             : {self.wz:.3f} rad/s\n"
            f"  pitch_trigger         : {self.use_pitch_trigger}\n"
            f"  pitch_trigger_deg     : {math.degrees(self.pitch_trigger):.1f} deg\n"
            f"  pitch_mode            : {self.pitch_trigger_mode}\n"
            f"  pitch_hold_s          : {self.pitch_hold_s:.2f} s\n"
            f"  euler_axes            : {self.euler_axes}\n"
            f"  wait_imu_on_start     : {self.wait_imu_on_start}\n"
            f"  imu_wait_timeout_s    : {self.imu_wait_timeout_s:.2f} s\n"
            f"  imu_stale_timeout_s   : {self.imu_stale_timeout_s:.2f} s\n"
            f"  dt_nominal            : {self.dt_nominal:.4f} s, dt_max_clamp: {self.dt_max:.4f} s\n"
            f"  start_phase           : {self.phase}"
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
        transforms3d expects quaternion order (w, x, y, z).
        sensor_msgs/Imu gives orientation fields x,y,z,w.
        """
        q = msg.orientation
        # reorder to (w, x, y, z)
        qw, qx, qy, qz = q.w, q.x, q.y, q.z

        ax, ay, az = euler.quat2euler((qw, qx, qy, qz), axes=self.euler_axes)

        # For 'sxyz', ay corresponds to rotation about y (often used as "pitch")
        self.pitch_rad = float(ay)

        self._imu_received = True
        self._last_imu_time = self._now_s()

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

    def _imu_is_fresh(self) -> bool:
        if not self._imu_received or self._last_imu_time is None:
            return False
        if self.imu_stale_timeout_s <= 0.0:
            return True
        age = self._now_s() - self._last_imu_time
        return age <= self.imu_stale_timeout_s

    def _pitch_over_threshold(self) -> bool:
        p = float(self.pitch_rad)
        th = float(self.pitch_trigger)
        mode = self.pitch_trigger_mode
        self.node.get_logger().info(f"[{self.name}] pitch={math.degrees(p):.1f} deg, th={math.degrees(th):.1f} deg")

        if mode == "positive":
            return p > th
        if mode == "negative":
            return p < -th
        # either
        return abs(p) < th

    # -----------------------------
    # main tick
    # -----------------------------
    def update(self):
        dt = self._step_dt()
        now = self._now_s()

        # 0) WAIT_IMU phase (optional): do not start profile until first IMU arrives
        if self.phase == "WAIT_IMU":
            self._publish(0.0, 0.0)

            # If IMU arrived, start FORWARD immediately
            if self._imu_received:
                self.node.get_logger().info(f"[{self.name}] IMU received → FORWARD")
                self.phase = "FORWARD"
                self.t_in_phase = 0.0
                self._pitch_over_time = 0.0
                return Status.RUNNING

            # If IMU never arrives but timeout exceeded: start profile anyway (but trigger stays disabled until IMU fresh)
            if self._start_time is not None and (now - self._start_time) >= self.imu_wait_timeout_s:
                self.node.get_logger().warn(
                    f"[{self.name}] IMU wait timeout ({self.imu_wait_timeout_s:.2f}s) → FORWARD (trigger disabled until IMU fresh)"
                )
                self.phase = "FORWARD"
                self.t_in_phase = 0.0
                self._pitch_over_time = 0.0
                return Status.RUNNING

            return Status.RUNNING

        # 1) Pitch-trigger: end early (SUCCESS) to switch BT state
        if self.use_pitch_trigger:
            if not self._imu_is_fresh():
                # No fresh IMU -> do not accumulate hold time (prevents "default 0.0" trigger)
                self._pitch_over_time = 0.0
            else:
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

        # DONE / IDLE fallback
        self._publish(0.0, 0.0)
        return Status.SUCCESS
