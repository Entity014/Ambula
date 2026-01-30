#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Float64

# ปรับให้ตรงกับ package/msg จริงของคุณ
from ambula_interfaces.msg import StateEstimation


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class PDBalanceFromState(Node):
    """
    PD balance controller for Ambula using StateEstimation.msg:
      state: pitch, pitch_rate, linear_velocity, yaw_rate, height
    output: wheel velocity commands (left/right same for straight balance)
    """

    def __init__(self):
        super().__init__("pd_balance_from_state")

        # ---------- params ----------
        self.declare_parameter("state_topic", "/state_estimation")
        self.declare_parameter("state_timeout_sec", 0.2)

        self.declare_parameter("left_wheel_cmd_topic", "/left_wheel_velocity_controller/command")
        self.declare_parameter("right_wheel_cmd_topic", "/right_wheel_velocity_controller/command")

        # PD on pitch
        self.declare_parameter("kp_pitch", 6.0)
        self.declare_parameter("kd_pitch", 0.25)

        # (optional) velocity outer loop → bias pitch_ref
        self.declare_parameter("use_velocity_outer_loop", False)
        self.declare_parameter("kv", 0.0)  # gain mapping (v_ref - v) -> pitch_ref_bias [rad]
        self.declare_parameter("v_ref", 0.0)  # m/s

        # refs & limits
        self.declare_parameter("pitch_ref_deg", 0.0)
        self.declare_parameter("pitch_ref_max_deg", 8.0)  # limit pitch_ref magnitude
        self.declare_parameter("abort_pitch_deg", 25.0)

        # wheel cmd limits (หน่วยต้องตรงกับ controller ของคุณ: rad/s หรือ m/s)
        self.declare_parameter("cmd_max", 8.0)
        self.declare_parameter("dcmd_max", 60.0)  # rate limit per second

        # control timing
        self.declare_parameter("control_rate_hz", 200.0)

        # ---------- load params ----------
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.state_timeout = float(self.get_parameter("state_timeout_sec").value)

        self.kp = float(self.get_parameter("kp_pitch").value)
        self.kd = float(self.get_parameter("kd_pitch").value)

        self.use_outer = bool(self.get_parameter("use_velocity_outer_loop").value)
        self.kv = float(self.get_parameter("kv").value)
        self.v_ref = float(self.get_parameter("v_ref").value)

        self.pitch_ref0 = math.radians(float(self.get_parameter("pitch_ref_deg").value))
        self.pitch_ref_max = math.radians(float(self.get_parameter("pitch_ref_max_deg").value))
        self.abort_pitch = math.radians(float(self.get_parameter("abort_pitch_deg").value))

        self.cmd_max = float(self.get_parameter("cmd_max").value)
        self.dcmd_max = float(self.get_parameter("dcmd_max").value)

        rate = float(self.get_parameter("control_rate_hz").value)
        self.dt = 1.0 / max(rate, 1.0)

        # ---------- pubs ----------
        self.pub_l = self.create_publisher(Float64, str(self.get_parameter("left_wheel_cmd_topic").value), 10)
        self.pub_r = self.create_publisher(Float64, str(self.get_parameter("right_wheel_cmd_topic").value), 10)

        # ---------- state cache ----------
        self.last_state: Optional[StateEstimation] = None
        self.last_state_time = None
        self.last_cmd = 0.0
        self.latched_abort = False

        # ---------- sub ----------
        self.sub = self.create_subscription(StateEstimation, self.state_topic, self.cb_state, 10)

        # ---------- timer ----------
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f"PD Balance (from StateEstimation) started | topic={self.state_topic}, "
            f"kp={self.kp}, kd={self.kd}, rate={rate}Hz"
        )

    def cb_state(self, msg: StateEstimation):
        self.last_state = msg
        self.last_state_time = self.get_clock().now()

    def publish_cmd(self, u: float):
        m = Float64()
        m.data = float(u)
        self.pub_l.publish(m)
        self.pub_r.publish(m)

    def publish_zero(self):
        self.last_cmd = 0.0
        self.publish_cmd(0.0)

    def control_loop(self):
        now = self.get_clock().now()

        # timeout
        if self.last_state is None or self.last_state_time is None:
            self.publish_zero()
            return
        if (now - self.last_state_time) > Duration(seconds=self.state_timeout):
            self.get_logger().warn_throttle(2000, "StateEstimation timeout -> zero")
            self.publish_zero()
            return

        s = self.last_state
        pitch = float(math.radians(s.pitch))
        pitch_rate = float(s.pitch_rate)
        v = float(s.linear_velocity)

        # abort safety
        if abs(pitch) > self.abort_pitch:
            if not self.latched_abort:
                self.get_logger().error(
                    f"ABORT: |pitch|={math.degrees(abs(pitch)):.1f}deg > {math.degrees(self.abort_pitch):.1f}deg"
                )
            self.latched_abort = True
            self.publish_zero()
            return

        # pitch reference (optionally biased by velocity error)
        pitch_ref = self.pitch_ref0
        if self.use_outer and self.kv != 0.0:
            pitch_ref += self.kv * (self.v_ref - v)
        pitch_ref = clamp(pitch_ref, -self.pitch_ref_max, self.pitch_ref_max)

        # PD on pitch
        e = pitch_ref - pitch
        cmd = self.kp * e - self.kd * pitch_rate

        # clamp magnitude
        cmd = clamp(cmd, -self.cmd_max, self.cmd_max)

        # rate limit
        d_allow = self.dcmd_max * self.dt
        cmd = clamp(cmd, self.last_cmd - d_allow, self.last_cmd + d_allow)
        self.last_cmd = cmd

        self.publish_cmd(cmd)


def main():
    rclpy.init()
    node = PDBalanceFromState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
