#!/usr/bin/env python3
"""
ROS2 rclpy node for Ambula ONNX Actor (obs_dim=36) that matches Isaac Lab env:

Actor obs layout (36):
  stack_policy (10) x 3 frames  -> 30
    per frame: wheel_joint_vel(2) + base_ang_vel(3) + projected_gravity(3) + actions(2)
  none_stack_policy (6) at t     -> 6
    velocity_commands(4) + base_lin_vel(1) + base_pos_z(1)

Notes:
- ONNX outputs normalized actions (2)
- optional clip to [-1, 1]
- multiply by action_scale_rad_s (=20.0 in env) -> wheel rad/s
- clamp wheel rad/s (safety; env wheels velocity_limit ~30 rad/s)
- convert rad/s -> rev/s and publish packed in cmd_vel.linear.x/y (left/right)

Env references: action_scale, wheel velocity limits in exported env.pdf. :contentReference[oaicite:2]{index=2}
PPO reference: Schulman et al., 2017.
"""

import math
import time
from collections import deque
from typing import Optional

import numpy as np
import onnxruntime as ort

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

RAD2REV = 1.0 / (2.0 * math.pi)

def quat_to_rotmat(qx, qy, qz, qw) -> np.ndarray:
    # quaternion (x,y,z,w) -> rotation matrix (world -> body) if quaternion represents world->body
    x, y, z, w = float(qx), float(qy), float(qz), float(qw)
    # guard against zero quaternion
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-9:
        return np.eye(3, dtype=np.float32)
    x, y, z, w = x/n, y/n, z/n, w/n

    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z

    return np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)],
        [2*(xy + wz),         1 - 2*(xx + zz),   2*(yz - wx)],
        [2*(xz - wy),         2*(yz + wx),       1 - 2*(xx + yy)],
    ], dtype=np.float32)

def projected_gravity_from_quat(qx, qy, qz, qw):
    R_bw = quat_to_rotmat(qx, qy, qz, qw)   # body -> world (สมมติ)
    R_wb = R_bw.T                           # world -> body (inverse)
    g_world = np.array([0.0, 0.0, -1.0], dtype=np.float32)
    g_body = R_wb @ g_world
    g_body /= (np.linalg.norm(g_body) + 1e-9)
    return float(g_body[0]), float(g_body[1]), float(g_body[2])


class OnnxCmdVelWheelPackedNode(Node):
    def __init__(self):
        super().__init__("reinforcement_node")

        # -------- params --------
        self.declare_parameter("onnx_path", "/home/xero/Ambula/robot_ws/src/ambula_rl/src/policy.onnx")
        self.declare_parameter("rate_hz", 100.0)

        # Match env: JointVelocityActionCfg(scale=20.0) :contentReference[oaicite:3]{index=3}
        self.declare_parameter("action_scale_rad_s", 20.0)
        self.declare_parameter("clip_norm_action", True)
        # Safety clamp after scaling (env wheel velocity_limit ~30 rad/s) :contentReference[oaicite:4]{index=4}
        self.declare_parameter("clip_wheel_rad_s", 30.0)

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/odom")          # for base_lin_vel + base_pos_z
        self.declare_parameter("rl_cmd_topic", "/rl_cmd")      # for velocity_commands(4)

        self.declare_parameter("left_wheel_joint", "left_wheel_joint")
        self.declare_parameter("right_wheel_joint", "right_wheel_joint")

        # If you don't have odom/height, these defaults will be used.
        self.declare_parameter("default_base_lin_vel", 0.0)
        self.declare_parameter("default_base_pos_z", 0.0)
        self.declare_parameter("default_cmd_z_hold", 0.0)

        # -------- expected obs structure --------
        self.frame_dim = 10
        self.stack_len = 3  # 10*3 = 30
        self.nonstack_dim = 6  # 6
        self.expected_obs_dim = self.frame_dim * self.stack_len + self.nonstack_dim  # 36

        # -------- ORT --------
        onnx_path = self.get_parameter("onnx_path").value
        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.sess = ort.InferenceSession(onnx_path, sess_options=so, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0]

        # Determine model obs dim (may be dynamic)
        self.obs_dim = self.expected_obs_dim
        try:
            if isinstance(self.inp.shape, (list, tuple)) and len(self.inp.shape) == 2:
                feat = self.inp.shape[1]
                if isinstance(feat, int):
                    self.obs_dim = feat
        except Exception:
            pass

        if self.obs_dim != self.expected_obs_dim:
            self.get_logger().warn(
                f"ONNX expects obs_dim={self.obs_dim}, but node builds {self.expected_obs_dim}. "
                f"Will pad/trim to fit, but ideally make them match."
            )

        # warmup
        dummy = np.zeros((1, self.obs_dim), dtype=np.float32)
        for _ in range(50):
            _ = self.sess.run(None, {self.inp.name: dummy})
        self.get_logger().info(f"Loaded ONNX: {onnx_path} | input={self.inp.name} shape={self.inp.shape}")

        # -------- runtime signals --------
        self.wheel_vel_l: Optional[float] = None
        self.wheel_vel_r: Optional[float] = None

        self.ang_x: Optional[float] = None
        self.ang_y: Optional[float] = None
        self.ang_z: Optional[float] = None

        self.gx: Optional[float] = None
        self.gy: Optional[float] = None
        self.gz: Optional[float] = None

        # non-stack signals
        self.cmd_vx: float = 0.0
        self.cmd_vy: float = 0.0
        self.cmd_wz: float = 0.0
        self.cmd_z_hold: float = float(self.get_parameter("default_cmd_z_hold").value)

        self.base_lin_vel: float = float(self.get_parameter("default_base_lin_vel").value)
        self.base_pos_z: float = float(self.get_parameter("default_base_pos_z").value)

        # -------- buffers --------
        # Each frame: [wheel_vel(2), ang_vel(3), grav(3), last_action(2)] = 10
        self.frame_buf = deque(maxlen=self.stack_len)  # newest at left
        self.last_action = np.zeros((2,), dtype=np.float32)

        # Pre-fill frames with zeros so obs starts stable
        for _ in range(self.stack_len):
            self.frame_buf.append(np.zeros((self.frame_dim,), dtype=np.float32))

        # -------- joint index cache --------
        self._js_name_to_idx = None  # dict cached after first valid JointState with names

        # -------- ROS I/O --------
        imu_topic = self.get_parameter("imu_topic").value
        js_topic  = self.get_parameter("joint_states_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        rl_cmd_topic = self.get_parameter("rl_cmd_topic").value
        cmd_topic = self.get_parameter("cmd_vel_topic").value

        self.create_subscription(Imu, imu_topic, self.cb_imu, 10)
        self.create_subscription(JointState, js_topic, self.cb_joint_states, 10)
        self.create_subscription(Odometry, odom_topic, self.cb_odom, 10)
        self.create_subscription(Twist, rl_cmd_topic, self.cb_rl_cmd, 10)

        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate_hz, 1e-6), self.on_timer)

        self.step = 0
        self.get_logger().warn("NOTE: cmd_vel.linear.x/y carry (left,right) wheel commands [rev/s].")

    def cb_rl_cmd(self, msg: Twist):
        # velocity_commands(4) = [vx, vy, wz, z_hold]
        self.cmd_vx = float(msg.linear.x)
        self.cmd_vy = float(msg.linear.y)
        self.cmd_wz = float(msg.angular.z)
        self.cmd_z_hold = float(msg.linear.z)

    def cb_odom(self, msg: Odometry):
        # base_lin_vel (1): you can choose x-speed in base frame; odom twist is usually in child frame
        # We'll use forward speed magnitude in x for simplicity.
        self.base_lin_vel = float(msg.twist.twist.linear.x)
        self.base_pos_z = float(msg.pose.pose.position.z)

    def cb_imu(self, msg: Imu):
        # angular vel in body frame (assume IMU aligned with base_link)
        self.ang_x = float(msg.angular_velocity.x)
        self.ang_y = float(msg.angular_velocity.y)
        self.ang_z = float(msg.angular_velocity.z)

        q = msg.orientation
        self.gx, self.gy, self.gz = projected_gravity_from_quat(q.x, q.y, q.z, q.w)

    def cb_joint_states(self, msg: JointState):
        left_name  = self.get_parameter("left_wheel_joint").value
        right_name = self.get_parameter("right_wheel_joint").value

        if not msg.name or len(msg.velocity) != len(msg.name):
            return

        # cache indices once (if name list stable)
        if self._js_name_to_idx is None:
            self._js_name_to_idx = {n: i for i, n in enumerate(msg.name)}

        idx_map = self._js_name_to_idx
        if left_name in idx_map:
            i = idx_map[left_name]
            if i < len(msg.velocity):
                self.wheel_vel_l = float(msg.velocity[i])  # rad/s
        if right_name in idx_map:
            i = idx_map[right_name]
            if i < len(msg.velocity):
                self.wheel_vel_r = float(msg.velocity[i])  # rad/s

    def ready(self) -> bool:
        return all(v is not None for v in [
            self.wheel_vel_l, self.wheel_vel_r,
            self.ang_x, self.ang_y, self.ang_z,
            self.gx, self.gy, self.gz
        ])

    def build_frame10(self) -> np.ndarray:
        # 10 = wheel(2) + ang(3) + grav(3) + actions(2)
        return np.array([
            self.wheel_vel_l, self.wheel_vel_r,
            self.ang_x, self.ang_y, self.ang_z,
            self.gx, self.gy, self.gz,
            float(self.last_action[0]), float(self.last_action[1]),
        ], dtype=np.float32)

    def build_obs(self) -> np.ndarray:
        self.frame_buf.appendleft(self.build_frame10())

        stack_part = np.concatenate([self.frame_buf[i] for i in range(self.stack_len)], axis=0)  # (30,)
        nonstack_part = np.array([
            self.cmd_vx, self.cmd_vy, self.cmd_wz, self.cmd_z_hold,
            self.base_lin_vel, self.base_pos_z
        ], dtype=np.float32)  # (6,)

        obs = np.concatenate([stack_part, nonstack_part], axis=0).astype(np.float32)  # (36,)
        assert obs.shape == (36,), f"BUG: obs shape is {obs.shape}, expected (36,)"

        return obs

    def infer_action_norm(self, obs: np.ndarray):
        x = obs[None, :].astype(np.float32)

        t0 = time.perf_counter_ns()
        out = self.sess.run(None, {self.inp.name: x})[0]
        t1 = time.perf_counter_ns()
        infer_ms = (t1 - t0) / 1e6

        act = np.asarray(out, dtype=np.float32).reshape(-1)
        if act.size < 2:
            raise RuntimeError(f"ONNX output has size {act.size}, expected >=2")

        act = act[:2]
        if bool(self.get_parameter("clip_norm_action").value):
            act = np.clip(act, -1.0, 1.0)

        return act, infer_ms

    def publish_stop(self):
        self.pub_cmd.publish(Twist())

    def on_timer(self):
        if not self.ready():
            return

        try:
            obs = self.build_obs()
            assert obs.shape[0] == 36, f"BUG: feeding obs shape={obs.shape}"
            act_norm, infer_ms = self.infer_action_norm(obs)

            # update last_action (for next frame)
            self.last_action = act_norm.copy()

            # normalized -> rad/s (scale=20 in env) :contentReference[oaicite:5]{index=5}
            scale = float(self.get_parameter("action_scale_rad_s").value)
            wl_rad = float(act_norm[0]) * scale
            wr_rad = float(act_norm[1]) * scale

            # safety clamp rad/s (env velocity_limit ~30 rad/s) :contentReference[oaicite:6]{index=6}
            clip_w = float(self.get_parameter("clip_wheel_rad_s").value)
            wl_rad = max(-clip_w, min(clip_w, wl_rad))
            wr_rad = max(-clip_w, min(clip_w, wr_rad))

            # rad/s -> rev/s
            wl_rev = wl_rad * RAD2REV
            wr_rev = wr_rad * RAD2REV

            msg = Twist()
            msg.linear.x = wl_rev
            msg.linear.y = wr_rev
            self.pub_cmd.publish(msg)

            self.step += 1
            if self.step % 50 == 0:
                # self.get_logger().info(f"{obs.shape}")
                self.get_logger().info(
                    f"L/R rad/s=({wl_rad:.3f},{wr_rad:.3f}) | infer={infer_ms:.3f} ms | "
                    f"wheel(rad/s)=({self.wheel_vel_l:.2f},{self.wheel_vel_r:.2f}) "
                    f"cmd(vx,vy,wz,z)=({self.cmd_vx:.2f},{self.cmd_vy:.2f},{self.cmd_wz:.2f},{self.cmd_z_hold:.2f}) "
                    f"ang_z={self.ang_z:.2f} g=({self.gx:.2f},{self.gy:.2f},{self.gz:.2f})"
                )

        except Exception as e:
            self.get_logger().error(f"error: {e}")
            self.publish_stop()

def main():
    rclpy.init()
    node = OnnxCmdVelWheelPackedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    