#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import yaml


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def rev_to_deg(rev: float) -> float:
    return rev * 360.0


def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rad_to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def _direction_sign(v: Any) -> float:
    """
    direction: +1 or -1 (or any number; >=0 -> +1, <0 -> -1)
    Applied at the very end (after conversion/mapping).
    """
    try:
        x = float(v)
    except Exception:
        x = 1.0
    return 1.0 if x >= 0.0 else -1.0


class JointStateConverter(Node):
    """
    Sub: /joint_states/hardware (sensor_msgs/JointState)
    Pub: /joint_states/converted (sensor_msgs/JointState)

    Parameters:
      - output_unit: "rad" or "deg"  (apply to all joints)
      - joint_config_yaml: YAML string -> dict

    Joint config fields (per joint):
      - mode: waist_rad | map_rev | wheel_rev
      - offset_deg: float  (applied to position only, in deg)
      - flip: bool         (flip direction: pos, vel multiply by -1)  [applied to input]
      - direction: int     (+1 or -1)  [applied at the end to outputs]
      - flip_limits: bool  (map_rev only; if flip, auto negate & swap rev_min/rev_max)
      - rev_min, rev_max, deg_max (map_rev only)

    Units expected at input:
      - waist_rad: pos rad, vel rad/s
      - map_rev:   pos rev, vel rev/s
      - wheel_rev: pos rev, vel rev/s
    """

    def __init__(self):
        super().__init__("joint_converter")

        self.declare_parameter("input_topic", "/joint_states/hardware")
        self.declare_parameter("output_topic", "/joint_states/converted")
        self.declare_parameter("output_unit", "rad")

        self.declare_parameter(
            "joint_config_yaml",
            """
left_waist:  {mode: waist_rad, offset_deg: 0.0, flip: false, direction: 1}
left_hip:    {mode: map_rev, rev_min: -1.6, rev_max: 7.6, deg_max: 180.0, offset_deg: 0.0, flip: false, flip_limits: true, direction: 1}
left_knee:   {mode: map_rev, rev_min: -1.8, rev_max: 0.0, deg_max: 90.0,  offset_deg: 0.0, flip: false, flip_limits: true, direction: 1}
left_wheel:  {mode: wheel_rev, offset_deg: 0.0, flip: false, direction: 1}

right_waist: {mode: waist_rad, offset_deg: 0.0, flip: true,  direction: 1}
right_hip:   {mode: map_rev, rev_min: -1.6, rev_max: 7.6, deg_max: 180.0, offset_deg: 0.0, flip: true,  flip_limits: true, direction: 1}
right_knee:  {mode: map_rev, rev_min: -1.8, rev_max: 0.0, deg_max: 90.0,  offset_deg: 0.0, flip: true,  flip_limits: true, direction: 1}
right_wheel: {mode: wheel_rev, offset_deg: 0.0, flip: true,  direction: 1}
""".strip()
        )

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)

        self.output_unit = str(self.get_parameter("output_unit").value).lower().strip()
        if self.output_unit not in ("rad", "deg"):
            self.get_logger().warn("output_unit must be 'rad' or 'deg'. Fallback to 'rad'.")
            self.output_unit = "rad"

        joint_config_yaml = str(self.get_parameter("joint_config_yaml").value)
        try:
            cfg = yaml.safe_load(joint_config_yaml) or {}
            if not isinstance(cfg, dict):
                raise ValueError("joint_config_yaml must be a YAML mapping (dict at top-level).")
            self.joint_config: Dict[str, Any] = cfg
        except Exception as e:
            self.get_logger().error(f"Failed to parse joint_config_yaml: {e}")
            self.joint_config = {}

        if not self.joint_config:
            self.get_logger().warn("joint_config is empty. Node will publish empty/zero JointState.")

        self.sub = self.create_subscription(JointState, self.input_topic, self.cb, 10)
        self.pub = self.create_publisher(JointState, self.output_topic, 10)

        self.get_logger().info(f"Sub: {self.input_topic}")
        self.get_logger().info(f"Pub: {self.output_topic}")
        self.get_logger().info(f"Output unit: {self.output_unit}")
        self.get_logger().info(f"Configured joints: {list(self.joint_config.keys())}")

    # Internal base unit for outputs is deg / deg/s (then convert to rad if needed)
    def _pos_out(self, pos_deg: float) -> float:
        return deg_to_rad(pos_deg) if self.output_unit == "rad" else pos_deg

    def _vel_out(self, vel_deg_s: float) -> float:
        return deg_to_rad(vel_deg_s) if self.output_unit == "rad" else vel_deg_s

    def cb(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        out = JointState()
        out.header = msg.header

        out_names = list(self.joint_config.keys())
        out.name = out_names
        out.position = [0.0] * len(out_names)
        out.velocity = [0.0] * len(out_names)
        out.effort = []

        for j, joint_name in enumerate(out_names):
            cfg = self.joint_config.get(joint_name, {})
            mode = str(cfg.get("mode", "wheel_rev")).strip()

            # flip (เดิม) -> คูณที่ input
            flip = bool(cfg.get("flip", False))
            flip_sign = -1.0 if flip else 1.0

            # direction (ใหม่) -> คูณที่ output (ตอนท้ายสุด)
            dir_sign = _direction_sign(cfg.get("direction", 1))

            offset_deg = float(cfg.get("offset_deg", 0.0))

            if joint_name not in name_to_idx:
                continue

            i = name_to_idx[joint_name]
            pos_in = float(msg.position[i]) if i < len(msg.position) else 0.0
            vel_in = float(msg.velocity[i]) if i < len(msg.velocity) else 0.0

            # Apply flip to raw input (pos + vel)
            pos_in *= flip_sign
            vel_in *= flip_sign

            # ---------------- waist: input rad ----------------
            if mode == "waist_rad":
                pos_deg = rad_to_deg(pos_in) + offset_deg
                vel_deg_s = rad_to_deg(vel_in)

                out.position[j] = self._pos_out(pos_deg)
                out.velocity[j] = self._vel_out(vel_deg_s)

            # -------------- hip/knee: input rev -> map 0..deg_max ----------------
            elif mode == "map_rev":
                rev_min = float(cfg.get("rev_min", 0.0))
                rev_max = float(cfg.get("rev_max", 1.0))
                deg_max = float(cfg.get("deg_max", 180.0))

                # flip_limits ผูกกับ flip เท่านั้น
                flip_limits = bool(cfg.get("flip_limits", True))
                if flip and flip_limits:
                    rev_min, rev_max = (-rev_max), (-rev_min)

                if rev_max <= rev_min:
                    self.get_logger().warn(
                        f"{joint_name}: invalid rev_min/rev_max after flip_limits (rev_min={rev_min}, rev_max={rev_max})"
                    )
                    continue

                pos_c = clamp(pos_in, rev_min, rev_max)
                deg = (pos_c - rev_min) / (rev_max - rev_min) * deg_max
                deg += offset_deg

                deg_per_rev = deg_max / (rev_max - rev_min)
                vel_deg_s = vel_in * deg_per_rev

                out.position[j] = self._pos_out(deg)
                out.velocity[j] = self._vel_out(vel_deg_s)

            # ---------------- wheel: input rev, no clamp ----------------
            elif mode == "wheel_rev":
                pos_deg = rev_to_deg(pos_in) + offset_deg
                vel_deg_s = rev_to_deg(vel_in)

                out.position[j] = self._pos_out(pos_deg)
                out.velocity[j] = self._vel_out(vel_deg_s)

            else:
                # fallback: treat input as rad
                pos_deg = rad_to_deg(pos_in) + offset_deg
                vel_deg_s = rad_to_deg(vel_in)

                out.position[j] = self._pos_out(pos_deg)
                out.velocity[j] = self._vel_out(vel_deg_s)

            # ===== APPLY direction AT THE VERY END =====
            out.position[j] *= dir_sign
            out.velocity[j] *= dir_sign

        self.pub.publish(out)


def main():
    rclpy.init()
    node = JointStateConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
