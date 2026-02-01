#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class CmdVelOneCycleStandFF(Node):
    """
    Feedforward stand assist (one cycle):
      forward for Tf -> backward for Tb -> stop

    Publishes geometry_msgs/Twist to /cmd_vel.
    """

    def __init__(self):
        super().__init__("cmdvel_onecycle_stand_ff")

        # ---- Params ----
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("control_hz", 100.0)

        # Profile (assuming /cmd_vel linear.x in m/s)
        self.declare_parameter("v_peak", 0.20)          # m/s
        self.declare_parameter("forward_time", 0.35)    # sec
        self.declare_parameter("backward_time", 0.35)   # sec
        self.declare_parameter("ramp_time", 0.10)       # sec (within each segment)
        self.declare_parameter("yaw_rate", 0.0)         # rad/s (keep 0 for stand assist)

        # Safety (optional)
        self.declare_parameter("use_imu_safety", True)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("pitch_limit_deg", 25.0)

        # ---- Setup ----
        self.cmd_topic = self.get_parameter("cmd_vel_topic").value
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.hz = float(self.get_parameter("control_hz").value)
        self.dt = 1.0 / self.hz

        self.v_peak = float(self.get_parameter("v_peak").value)
        self.Tf = float(self.get_parameter("forward_time").value)
        self.Tb = float(self.get_parameter("backward_time").value)
        self.Tr = max(1e-3, float(self.get_parameter("ramp_time").value))
        self.wz = float(self.get_parameter("yaw_rate").value)

        # IMU safety
        self.use_imu = bool(self.get_parameter("use_imu_safety").value)
        self.pitch_rad = 0.0
        self.pitch_limit = math.radians(float(self.get_parameter("pitch_limit_deg").value))
        if self.use_imu:
            imu_topic = self.get_parameter("imu_topic").value
            self.create_subscription(Imu, imu_topic, self.cb_imu, 50)

        # State
        self.phase = "FORWARD"   # FORWARD -> BACKWARD -> DONE/ABORT
        self.t_in_phase = 0.0

        self.timer = self.create_timer(self.dt, self.step)
        self.get_logger().info(f"One-cycle cmd_vel FF started -> publishing to {self.cmd_topic}")

    def cb_imu(self, msg: Imu):
        # quaternion -> pitch (standard conversion)
        q = msg.orientation
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = clamp(sinp, -1.0, 1.0)
        self.pitch_rad = math.asin(sinp)

    def publish(self, vx: float, wz: float):
        tw = Twist()
        tw.linear.x = float(vx)
        tw.angular.z = float(wz)
        self.pub.publish(tw)

    def abort(self, reason: str):
        if self.phase != "ABORT":
            self.get_logger().error(f"ABORT: {reason}")
        self.phase = "ABORT"
        self.publish(0.0, 0.0)

    def trapezoid(self, t: float, T: float, peak: float) -> float:
        """
        Simple trapezoid profile in a segment duration T:
          ramp up (Tr) -> hold -> ramp down (Tr)
        """
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

    def step(self):
        # Safety
        if self.use_imu and abs(self.pitch_rad) > self.pitch_limit:
            self.abort(f"Pitch too large: {math.degrees(self.pitch_rad):.1f} deg")
            return

        if self.phase in ["DONE", "ABORT"]:
            self.publish(0.0, 0.0)
            return

        self.t_in_phase += self.dt

        if self.phase == "FORWARD":
            vx = self.trapezoid(self.t_in_phase, self.Tf, +self.v_peak)
            self.publish(vx, self.wz)
            if self.t_in_phase >= self.Tf:
                self.phase = "BACKWARD"
                self.t_in_phase = 0.0

        elif self.phase == "BACKWARD":
            vx = self.trapezoid(self.t_in_phase, self.Tb, -self.v_peak)
            self.publish(vx, self.wz)
            if self.t_in_phase >= self.Tb:
                self.phase = "DONE"
                self.publish(0.0, 0.0)
                self.get_logger().info("DONE: one-cycle finished, cmd_vel = 0")


def main():
    rclpy.init()
    node = CmdVelOneCycleStandFF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
