#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Bool, Float32


def apply_deadzone(x: float, dz: float) -> float:
    return 0.0 if abs(x) < dz else x


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class JoyToCmdVel(Node):
    def __init__(self):
        super().__init__("joy_to_cmd_vel")

        # =============================
        # Topics
        # =============================
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("robot_state_topic", "/robot/state")
        self.declare_parameter("payload_state_topic", "/payload/state")
        self.declare_parameter("led_state_topic", "/led/state")

        # ── Wall-proximity guard ──────────────────────────────────────────────
        # Sub topics from opening_detector:
        #   estop_topic    : Bool    — false=GO(PASS), true=stop(CROUCH/BLOCKED/WALL)
        #   distance_topic : Float32 — distance to detected wall plane [m]
        #
        # Guard logic (same for CROUCH, BLOCKED, and WALL):
        #   estop=false                          → no restriction
        #   estop=true  AND dist > wall_stop_distance → no restriction (still far)
        #   estop=true  AND dist <= wall_stop_distance → block linear.x > 0
        #
        # Reverse (linear.x < 0) and rotation (angular.z) always allowed.
        self.declare_parameter("estop_topic",         "/opening_detection/estop")
        self.declare_parameter("distance_topic",      "/opening_detection/wall_distance")
        self.declare_parameter("wall_stop_distance",  0.5)   # [m] — full stop threshold
        self.declare_parameter("wall_slowdown_distance", 1.5) # [m] — begin slowing down here

        # =============================
        # cmd_vel mapping
        # =============================
        self.declare_parameter("axis_linear", 1)
        self.declare_parameter("axis_angular", 0)
        self.declare_parameter("scale_linear", 0.75)
        self.declare_parameter("scale_angular", 1.5)
        self.declare_parameter("deadzone", 0.05)

        self.declare_parameter("enable_button", 4)
        self.declare_parameter("turbo_button", 5)
        self.declare_parameter("turbo_multiplier", 2.0)

        self.declare_parameter("use_enable_button", True)
        self.declare_parameter("use_turbo", True)

        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("joy_timeout_sec", 0.25)

        # =============================
        # Height command
        # =============================
        self.declare_parameter("height_min", 0.15)
        self.declare_parameter("height_max", 0.30)
        self.declare_parameter("height_default", 0.15)
        self.declare_parameter("height_step", 0.01)

        self.declare_parameter("dpad_vertical_axis", 7)
        self.declare_parameter("dpad_threshold", 0.5)
        self.declare_parameter("invert_dpad_axis", False)

        self.declare_parameter("dpad_up_button", -1)
        self.declare_parameter("dpad_down_button", -1)

        self.declare_parameter("require_enable_for_height_change", True)
        self.declare_parameter("publish_height_always", True)

        # =============================
        # Robot state (/robot/state)
        # =============================
        self.declare_parameter("publish_state_always", True)
        self.declare_parameter("require_enable_for_state_change", False)

        self.declare_parameter("x_btn0", 0)
        self.declare_parameter("x_btn1", 1)
        self.declare_parameter("x_btn2", 2)

        self.declare_parameter("y_trigger_btn", 3)
        self.declare_parameter("y_pulse_duration_ms", 120.0)

        self.declare_parameter("z_cycle_btn", 6)

        self.declare_parameter("state_x_default", 0)
        self.declare_parameter("state_z_default", 0)

        # =============================
        # ROS 2 Setup
        # =============================
        qos_cmd = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        joy_topic          = self.get_parameter("joy_topic").value
        cmd_vel_topic      = self.get_parameter("cmd_vel_topic").value
        robot_state_topic  = self.get_parameter("robot_state_topic").value
        payload_state_topic = self.get_parameter("payload_state_topic").value
        led_state_topic    = self.get_parameter("led_state_topic").value
        estop_topic        = self.get_parameter("estop_topic").value
        distance_topic     = self.get_parameter("distance_topic").value

        self.pub_cmd          = self.create_publisher(Twist, cmd_vel_topic, qos_cmd)
        self.pub_robot_state  = self.create_publisher(Int8, robot_state_topic, qos_cmd)
        self.pub_payload_state = self.create_publisher(Int8, payload_state_topic, qos_cmd)
        self.pub_led_state    = self.create_publisher(Int8, led_state_topic, qos_cmd)

        self.sub_joy = self.create_subscription(Joy, joy_topic, self.on_joy, 10)

        # ── Wall-proximity guard subscribers ─────────────────────────────────
        self._wall_estop    = False          # latest Bool from estop_topic
        self._wall_distance = math.inf       # latest Float32 from distance_topic

        if estop_topic:
            self.create_subscription(
                Bool, estop_topic,
                lambda m: setattr(self, '_wall_estop', bool(m.data)),
                10)

        if distance_topic:
            self.create_subscription(
                Float32, distance_topic,
                lambda m: setattr(self, '_wall_distance', float(m.data)),
                10)

        self.last_joy      = None
        self.last_joy_time = self.get_clock().now()

        # Internal States
        h0   = float(self.get_parameter("height_default").value)
        hmin = float(self.get_parameter("height_min").value)
        hmax = float(self.get_parameter("height_max").value)
        self.height_cmd = clamp(h0, hmin, hmax)

        self._prev_up   = False
        self._prev_down = False

        self.state_x = int(self.get_parameter("state_x_default").value)
        self.state_z = int(self.get_parameter("state_z_default").value)

        self.y_pulse_remaining_cycles = 0
        self._btn_prev = {}

        publish_hz = float(self.get_parameter("publish_hz").value)
        self.timer = self.create_timer(1.0 / publish_hz, self.on_timer)

        self.get_logger().info(
            f"Node started.\n"
            f"  Wall guard  estop          : {estop_topic or '(disabled)'}\n"
            f"  Wall guard  distance       : {distance_topic or '(disabled)'}\n"
            f"  wall_slowdown_distance     : {self.get_parameter('wall_slowdown_distance').value} m\n"
            f"  wall_stop_distance         : {self.get_parameter('wall_stop_distance').value} m"
        )

    # ---------- helper ----------
    def _get_axis(self, joy: Joy, i: int) -> float:
        axes = joy.axes if joy.axes is not None else []
        return float(axes[i]) if 0 <= i < len(axes) else 0.0

    def _get_button(self, joy: Joy, i: int) -> int:
        buttons = joy.buttons if joy.buttons is not None else []
        return int(buttons[i]) if 0 <= i < len(buttons) else 0

    def _enabled(self, joy: Joy) -> bool:
        if not bool(self.get_parameter("use_enable_button").value):
            return True
        enable_btn = int(self.get_parameter("enable_button").value)
        return self._get_button(joy, enable_btn) == 1

    def _rising(self, joy: Joy, btn_idx: int) -> bool:
        if btn_idx < 0:
            return False
        now  = (self._get_button(joy, btn_idx) == 1)
        prev = bool(self._btn_prev.get(btn_idx, False))
        self._btn_prev[btn_idx] = now
        return now and (not prev)

    # ---------- wall guard ----------
    def _wall_speed_scale(self) -> float:
        """
        Return a [0.0, 1.0] forward-speed multiplier based on wall distance.

          estop=false                              → 1.0  (no restriction)
          estop=true, dist >= slowdown_distance    → 1.0  (wall detected but far)
          estop=true, stop_dist < dist < slowdown  → linear ramp 0.0 → 1.0
          estop=true, dist <= stop_distance        → 0.0  (full stop)
        """
        if not self._wall_estop:
            return 1.0

        stop_dist     = float(self.get_parameter("wall_stop_distance").value)
        slowdown_dist = float(self.get_parameter("wall_slowdown_distance").value)
        dist          = self._wall_distance

        if dist <= stop_dist:
            return 0.0
        if dist >= slowdown_dist:
            return 1.0

        # linear ramp between stop_dist and slowdown_dist
        return (dist - stop_dist) / (slowdown_dist - stop_dist)

    # ---------- callbacks ----------
    def on_joy(self, msg: Joy):
        self.last_joy      = msg
        self.last_joy_time = self.get_clock().now()
        self._update_height_from_dpad(msg)
        self._update_robot_state(msg)

    def _update_height_from_dpad(self, joy: Joy):
        require_enable = bool(self.get_parameter("require_enable_for_height_change").value)
        if require_enable and (not self._enabled(joy)):
            return

        hmin  = float(self.get_parameter("height_min").value)
        hmax  = float(self.get_parameter("height_max").value)
        step  = float(self.get_parameter("height_step").value)
        thr   = float(self.get_parameter("dpad_threshold").value)
        invert = bool(self.get_parameter("invert_dpad_axis").value)
        up_btn   = int(self.get_parameter("dpad_up_button").value)
        down_btn = int(self.get_parameter("dpad_down_button").value)
        dpad_axis = int(self.get_parameter("dpad_vertical_axis").value)

        if up_btn >= 0 or down_btn >= 0:
            up_now   = (self._get_button(joy, up_btn) == 1)
            down_now = (self._get_button(joy, down_btn) == 1)
        else:
            y = self._get_axis(joy, dpad_axis)
            if invert:
                y = -y
            up_now, down_now = (y > thr), (y < -thr)

        if up_now   and not self._prev_up:   self.height_cmd = clamp(self.height_cmd + step, hmin, hmax)
        if down_now and not self._prev_down: self.height_cmd = clamp(self.height_cmd - step, hmin, hmax)

        self._prev_up, self._prev_down = up_now, down_now

    def _update_robot_state(self, joy: Joy):
        if bool(self.get_parameter("require_enable_for_state_change").value) and (not self._enabled(joy)):
            return

        for i in range(3):
            if self._rising(joy, int(self.get_parameter(f"x_btn{i}").value)):
                self.state_x = i

        y_btn = int(self.get_parameter("y_trigger_btn").value)
        if self._rising(joy, y_btn):
            hz          = float(self.get_parameter("publish_hz").value)
            duration_ms = float(self.get_parameter("y_pulse_duration_ms").value)
            self.y_pulse_remaining_cycles = int((duration_ms / 1000.0) * hz)
            if self.y_pulse_remaining_cycles < 1:
                self.y_pulse_remaining_cycles = 1
            self.get_logger().info(f"Y Pulse Triggered: {duration_ms}ms")

        if self._rising(joy, int(self.get_parameter("z_cycle_btn").value)):
            self.state_z = (self.state_z + 1) % 2

    def _publish_state(self):
        robot_msg   = Int8()
        payload_msg = Int8()
        led_msg     = Int8()
        robot_msg.data = int(self.state_x)
        led_msg.data   = int(self.state_z)

        if self.y_pulse_remaining_cycles > 0:
            payload_msg.data = 1
            self.y_pulse_remaining_cycles -= 1
        else:
            payload_msg.data = 0

        self.pub_robot_state.publish(robot_msg)
        self.pub_payload_state.publish(payload_msg)
        self.pub_led_state.publish(led_msg)

    def on_timer(self):
        now     = self.get_clock().now()
        timeout = float(self.get_parameter("joy_timeout_sec").value)
        cmd     = Twist()

        if bool(self.get_parameter("publish_height_always").value):
            cmd.linear.z = float(self.height_cmd)

        if bool(self.get_parameter("publish_state_always").value):
            self._publish_state()

        if self.last_joy is None or (now - self.last_joy_time).nanoseconds * 1e-9 > timeout:
            self.pub_cmd.publish(cmd)
            return

        joy = self.last_joy
        if not self._enabled(joy):
            self.pub_cmd.publish(cmd)
            return

        # Linear / Angular
        lin = apply_deadzone(
            self._get_axis(joy, int(self.get_parameter("axis_linear").value)),
            float(self.get_parameter("deadzone").value))
        ang = apply_deadzone(
            self._get_axis(joy, int(self.get_parameter("axis_angular").value)),
            float(self.get_parameter("deadzone").value))

        scale_lin = float(self.get_parameter("scale_linear").value)
        scale_ang = float(self.get_parameter("scale_angular").value)

        if bool(self.get_parameter("use_turbo").value) and \
                (self._get_button(joy, int(self.get_parameter("turbo_button").value)) == 1):
            mul        = float(self.get_parameter("turbo_multiplier").value)
            scale_lin *= mul
            scale_ang *= mul

        cmd.linear.x  = scale_lin * lin
        cmd.angular.z = scale_ang * ang

        if not bool(self.get_parameter("publish_height_always").value):
            cmd.linear.z = float(self.height_cmd)

        # ── Wall-proximity guard ──────────────────────────────────────────────
        # Scale forward speed linearly from 1.0 → 0.0 as wall approaches.
        #   dist >= wall_slowdown_distance  → full speed
        #   wall_stop_distance < dist < wall_slowdown_distance → ramp down
        #   dist <= wall_stop_distance      → full stop (0.0)
        # Reverse (linear.x < 0) and rotation (angular.z) always unaffected.
        if cmd.linear.x > 0.0:
            scale = self._wall_speed_scale()
            if scale < 1.0:
                cmd.linear.x *= scale
                if scale == 0.0:
                    self.get_logger().warn(
                        f"Wall guard: STOP "
                        f"(dist={self._wall_distance:.2f}m <= "
                        f"{self.get_parameter('wall_stop_distance').value}m)",
                        throttle_duration_sec=1.0)
                else:
                    self.get_logger().info(
                        f"Wall guard: slowing to {scale*100:.0f}% "
                        f"(dist={self._wall_distance:.2f}m)",
                        throttle_duration_sec=1.0)

        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = JoyToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()