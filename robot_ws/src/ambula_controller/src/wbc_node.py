#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
import pinocchio as pin
import inspect

from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from ambula_controller.wbc_core import (
    load_model,
    build_actuation_selection,
    IK_CONTROLLED_JOINTS,
    require_frame_id
)

try:
    from ambula_controller.wbc_core import solve_wbc_idqp_full_dynamic_leg_only as SOLVE_WBC_FN
except Exception:
    from ambula_controller.wbc_core import solve_wbc_idqp_full_dynamic as SOLVE_WBC_FN

from ambula_controller.wbc_kinematic import AmbulaLegIK


# ======================================================
# IMU Geometry Helpers
# ======================================================

def get_quaternion_from_gravity(q_raw_xyzw):
    """
    สร้าง Quaternion ที่ Align เฉพาะแกน Z (Gravity)
    เพื่อป้องกันแกน X/Y หมุน (Yaw Drift) ขณะหุ่นเอียง
    """
    x, y, z, w = q_raw_xyzw

    gx = 2.0 * (x * z - w * y)
    gy = 2.0 * (w * x + y * z)
    gz = w * w - x * x - y * y + z * z

    n = np.sqrt(gx**2 + gy**2 + gz**2)
    if n < 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

    up_body = np.array([-gx/n, -gy/n, -gz/n], dtype=float)
    world_z = np.array([0.0, 0.0, -1.0], dtype=float)
    q_aligned = pin.Quaternion.FromTwoVectors(world_z, up_body)
    return np.array([q_aligned.x, q_aligned.y, q_aligned.z, q_aligned.w], dtype=float)


def _quat_normalize_xyzw(q):
    q = np.asarray(q, dtype=float).reshape(4)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


def _smoothstep01(x: float) -> float:
    x = float(np.clip(x, 0.0, 1.0))
    return x * x * (3.0 - 2.0 * x)


def _parse_tau_override_string(s: str):
    out = {}
    if not s:
        return out
    for item in str(s).split(","):
        item = item.strip()
        if not item or ":" not in item:
            continue
        k, v = item.split(":", 1)
        try:
            out[k.strip()] = float(v.strip())
        except ValueError:
            pass
    return out


# ======================================================
# Main WBC Node
# ======================================================

class AmbulaWBCNode(Node):
    def __init__(self):
        super().__init__('ambula_wbc_node')

        # ======================================================
        # Parameters
        # ======================================================
        self.declare_parameter('urdf_path', '/home/robot_ws/src/ambula_simulation/ambula_description/urdf/ambula_bot_v3/ambula.urdf')
        self.declare_parameter('control_rate_hz', 200.0)
        self.declare_parameter('publish_all_joints', True)
        self.declare_parameter('non_actuated_effort_mode', 'zero')

        # --- State source ---
        self.declare_parameter('state_source', 'ekf')  # 'ekf' or 'raw'
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('raw_zero_gyro_z', False)

        # WBC Weights / Gains
        self.declare_parameter('weight_height',  60.0)
        self.declare_parameter('weight_roll',     0.0)
        self.declare_parameter('weight_posture',  5.0)

        self.declare_parameter('kp_z',    120.0)
        self.declare_parameter('kd_z',      8.0)
        self.declare_parameter('kp_roll',  30.0)
        self.declare_parameter('kd_roll',  15.0)

        self.declare_parameter('kp_hip',  300.0)
        self.declare_parameter('kd_hip',   30.0)
        self.declare_parameter('kp_knee',  50.0)
        self.declare_parameter('kd_knee',  14.0)

        # Hardware params
        self.declare_parameter('tau_limit',     35.0)
        self.declare_parameter('friction_mu',    0.6)
        self.declare_parameter('tau_limits',     [])
        self.declare_parameter('tau_override',   "")
        self.declare_parameter('knee_tau_scale',  5.0)
        self.declare_parameter('hip_tau_scale',   1.0)
        self.declare_parameter('waist_tau_scale', 5.0)
        self.declare_parameter('posture_qdd_clip', 200.0)

        # Standup / Boost
        self.declare_parameter('standup_enable', False)
        self.declare_parameter('standup_height', 0.35)
        self.declare_parameter('standup_duration', 1.2)

        self.declare_parameter('height_boost_enable', True)
        self.declare_parameter('height_boost_duration', 0.6)
        self.declare_parameter('height_boost_kp_mult', 2.0)
        self.declare_parameter('height_boost_w_mult',  2.0)
        self.declare_parameter('height_boost_zdd_clip', 20.0)
        self.declare_parameter('height_boost_knee_tau_scale', 1.0)
        self.declare_parameter('height_boost_tau_override', "")

        # Knee Spring
        self.declare_parameter('knee_spring_k',          0.5)
        self.declare_parameter('knee_spring_b',          0.3)
        self.declare_parameter('knee_spring_q0_left',    1.57)
        self.declare_parameter('knee_spring_q0_right',   1.57)
        self.declare_parameter('knee_spring_sign_left',  1.0)
        self.declare_parameter('knee_spring_sign_right', 1.0)

        # NEW: Hip Spring
        self.declare_parameter('hip_spring_k',           2.0)
        self.declare_parameter('hip_spring_b',           0.3)
        self.declare_parameter('hip_spring_q0_left',     1.57)
        self.declare_parameter('hip_spring_q0_right',    1.57)
        self.declare_parameter('hip_spring_sign_left',   1.0)
        self.declare_parameter('hip_spring_sign_right',  1.0)

        # NEW: Hip Gravity Compensation
        # 0.0 = off, 0.5-0.8 = recommended range
        self.declare_parameter('hip_grav_comp_scale',    0.6)

        # Robustness
        self.declare_parameter('w_roll_slack', 1e4)
        self.declare_parameter('alpha_bg',     20.0)
        # NOTE: w_dyn_slack default lowered to 500 (was 1e4)
        self.declare_parameter('w_dyn_slack',  500.0)

        # ======================================================
        # Initialization
        # ======================================================
        urdf_path = self.get_parameter('urdf_path').value
        self.rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.dt = 1.0 / max(self.rate_hz, 1e-6)

        self.state_source = str(self.get_parameter('state_source').value).strip().lower()
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.imu_topic  = str(self.get_parameter('imu_topic').value)

        self.get_logger().info(f"Loading Model: {urdf_path}")
        self.model, self.data = load_model(urdf_path, use_free_flyer=True)
        self.leg_ik = AmbulaLegIK(self.model)

        self.q_current = pin.neutral(self.model)
        self.u_current = np.zeros(self.model.nv)

        self.imu_ready   = (self.state_source == 'ekf')
        self.odom_ready  = False
        self.joint_ready = False

        self.target_base_height = 0.35
        self.current_z_ref = 0.35
        self.target_X = -0.05

        self._stand_t0 = None
        self._stand_h0 = None
        self._stand_done = False
        self._boost_t0 = None

        # Joint Mapping
        self.all_joint_names = []
        self.joint_info = {}
        for jid in range(1, self.model.njoints):
            j = self.model.joints[jid]
            if j.nv == 1:
                name = self.model.names[jid]
                self.all_joint_names.append(name)
                self.joint_info[name] = {"jid": jid, "iq": j.idx_q, "iv": j.idx_v, "nq": j.nq}

        _, self.na, self.actuated_ivs, _ = build_actuation_selection(self.model, exclude_keywords=("wheel",))
        self.q_ref_actuated = np.zeros(self.na)

        self.actuated = []
        self.act_name_to_act_idx = {}
        act_idx = 0
        for jid in range(1, self.model.njoints):
            j = self.model.joints[jid]
            if j.nv != 1:
                continue
            name = self.model.names[jid]
            if "wheel" in name.lower():
                continue
            self.actuated.append({
                "name": name, "jid": jid, "iq": j.idx_q, "iv": j.idx_v, "nq": j.nq, "act_idx": act_idx
            })
            self.act_name_to_act_idx[name] = act_idx
            act_idx += 1

        self.actuated_names = [a["name"] for a in self.actuated]
        self.actuated_name_set = set(self.actuated_names)
        self._seen_joint_states = set()

        self.solve_wbc = SOLVE_WBC_FN
        self.solve_sig = inspect.signature(self.solve_wbc)

        # ======================================================
        # Pubs/Subs
        # ======================================================
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 50)

        if self.state_source == 'raw':
            self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 200)
            self.get_logger().info(f"[WBC] state_source=raw | odom={self.odom_topic} imu={self.imu_topic}")
        else:
            self.sub_imu = None
            self.get_logger().info(f"[WBC] state_source=ekf | odom={self.odom_topic} (IMU topic not used in WBC)")

        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 20)
        self.sub_joint_states = self.create_subscription(JointState, '/joint_states/hardware', self.joint_state_callback, 50)

        self.pub_joint_commands = self.create_publisher(JointState, '/cmd_joint', 10)
        self.timer = self.create_timer(self.dt, self.control_loop)

    # ======================================================
    # CALLBACKS
    # ======================================================
    def imu_callback(self, msg: Imu):
        q_raw = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        q_aligned = get_quaternion_from_gravity(q_raw)
        self.q_current[3:7] = _quat_normalize_xyzw(q_aligned)

        wz = msg.angular_velocity.z
        if bool(self.get_parameter('raw_zero_gyro_z').value):
            wz = 0.0

        self.u_current[3:6] = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            wz
        ]
        self.imu_ready = True

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.q_current[0:3] = [p.x, p.y, p.z]

        v = msg.twist.twist.linear
        self.u_current[0:3] = [v.x, v.y, v.z]

        if self.state_source == 'ekf':
            oq = msg.pose.pose.orientation
            self.q_current[3:7] = _quat_normalize_xyzw([oq.x, oq.y, oq.z, oq.w])
            self.q_current[3:7] = _quat_normalize_xyzw([0.0, 0.0, oq.z, oq.w])
            w = msg.twist.twist.angular
            self.u_current[3:6] = [w.x, w.y, w.z]
            self.u_current[3:6] = [0.0, 0.0, w.z]
            self.imu_ready = True

        self.odom_ready = True

    def cmd_vel_callback(self, msg: Twist):
        if abs(msg.linear.z) > 0.05:
            wheel_radius   = 0.07
            base_to_hip_z  = 0.1165
            total_offset   = wheel_radius + base_to_hip_z

            min_base_h = self.leg_ik.L_MIN_REACHABLE + total_offset
            max_base_h = self.leg_ik.L_MAX_REACHABLE + total_offset

            new_h = float(np.clip(msg.linear.z, min_base_h, max_base_h))

            if new_h > self.target_base_height + 1e-4:
                self._boost_t0 = self.get_clock().now()
            self.target_base_height = new_h

    def joint_state_callback(self, msg: JointState):
        name_to_i = {n: i for i, n in enumerate(msg.name)}
        updated_names = set()

        for j_name, info in self.joint_info.items():
            if j_name not in name_to_i:
                continue
            i = name_to_i[j_name]

            if len(msg.position) > i:
                pos = float(msg.position[i])
                iq, nq = info["iq"], info["nq"]
                if nq == 1:
                    self.q_current[iq] = pos
                elif nq == 2:
                    self.q_current[iq]     = float(np.cos(pos))
                    self.q_current[iq + 1] = float(np.sin(pos))

            if len(msg.velocity) > i:
                self.u_current[info["iv"]] = float(msg.velocity[i])

            updated_names.add(j_name)

        if updated_names:
            self._seen_joint_states |= updated_names
            if self.actuated_name_set.issubset(self._seen_joint_states):
                self.joint_ready = True

    # ======================================================
    # CONTROL LOOP
    # ======================================================
    def control_loop(self):
        if self.state_source == 'ekf':
            if not (self.odom_ready and self.joint_ready):
                return
        else:
            if not (self.imu_ready and self.odom_ready and self.joint_ready):
                return

        self._apply_startup_standup()

        # Rate Limiter for Z
        max_dz = 0.15 * self.dt
        dz = self.target_base_height - self.current_z_ref
        if dz > max_dz:
            self.current_z_ref += max_dz
        elif dz < -max_dz:
            self.current_z_ref -= max_dz
        else:
            self.current_z_ref = self.target_base_height

        q, u = self.q_current.copy(), self.u_current.copy()
        self._refresh_posture_ref_from_current(q)

        # ── Height debug log ──────────────────────────────────────────────────
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        wheel_radius  = 0.07
        base_to_hip_z = 0.1165

        try:
            base_fid  = require_frame_id(self.model, "base_link")
            wheel_fid = require_frame_id(self.model, "left_wheel_link")
            base_z_kin  = self.data.oMf[base_fid].translation[2]
            wheel_z_kin = self.data.oMf[wheel_fid].translation[2]
            true_base_height = (base_z_kin - wheel_z_kin) + wheel_radius
        except Exception:
            true_base_height = 0.0

        z_ik_target = self.current_z_ref - wheel_radius - base_to_hip_z

        self.get_logger().info(
            f"\n[WBC Z-Check] Target={self.current_z_ref:.4f}m | "
            f"Current(kin)={true_base_height:.4f}m | IK(Hip->Wheel)={z_ik_target:.4f}m",
            throttle_duration_sec=0.5,
        )

        # ── IK reference ─────────────────────────────────────────────────────
        q_hip_l, q_knee_l = self.leg_ik.compute(self.target_X, -z_ik_target)

        self.set_q_ref_actuated("left_hip_joint",   q_hip_l)
        self.set_q_ref_actuated("left_knee_joint",  q_knee_l)
        self.set_q_ref_actuated("right_hip_joint",  q_hip_l)
        self.set_q_ref_actuated("right_knee_joint", q_knee_l)

        # Update hip spring q0 to track IK target in real time
        # This prevents the spring from fighting the IK reference.
        # Comment this out if you want a fixed q0 from the YAML.
        # self.set_parameter(rclpy.parameter.Parameter(
        #     'hip_spring_q0_left',  rclpy.parameter.Parameter.Type.DOUBLE, q_hip_l))
        # self.set_parameter(rclpy.parameter.Parameter(
        #     'hip_spring_q0_right', rclpy.parameter.Parameter.Type.DOUBLE, q_hip_l))

        # ── Boost / Regular gains ─────────────────────────────────────────────
        w_height_base = float(self.get_parameter('weight_height').value)
        kp_z_base     = float(self.get_parameter('kp_z').value)
        kd_z_base     = float(self.get_parameter('kd_z').value)
        zdd_clip_use          = 8.0
        knee_tau_scale_use    = float(self.get_parameter('knee_tau_scale').value)
        tau_override          = self._build_tau_override_dict()

        boosting = self._boost_active() or (
            bool(self.get_parameter('standup_enable').value) and not self._stand_done
        )
        if boosting:
            kp_z_use     = kp_z_base * float(self.get_parameter('height_boost_kp_mult').value)
            w_height_use = w_height_base * float(self.get_parameter('height_boost_w_mult').value)
            zdd_clip_use = float(self.get_parameter('height_boost_zdd_clip').value)
            knee_tau_scale_use = float(self.get_parameter('height_boost_knee_tau_scale').value)

            boost_tau_s = str(self.get_parameter('height_boost_tau_override').value)
            if boost_tau_s.strip():
                tau_override.update(_parse_tau_override_string(boost_tau_s))
        else:
            kp_z_use, w_height_use = kp_z_base, w_height_base

        params = {
            'w_height':  w_height_use,
            'w_roll':    float(self.get_parameter('weight_roll').value),
            'w_posture': float(self.get_parameter('weight_posture').value),

            'kp_z':    kp_z_use,
            'kd_z':    kd_z_base,
            'kp_roll': float(self.get_parameter('kp_roll').value),
            'kd_roll': float(self.get_parameter('kd_roll').value),

            'kp_hip':  float(self.get_parameter('kp_hip').value),
            'kd_hip':  float(self.get_parameter('kd_hip').value),
            'kp_knee': float(self.get_parameter('kp_knee').value),
            'kd_knee': float(self.get_parameter('kd_knee').value),

            'tau_limit':  float(self.get_parameter('tau_limit').value),
            'mu':         float(self.get_parameter('friction_mu').value),

            'zdd_clip':   float(zdd_clip_use),
        }

        try:
            out = self._call_solver_safely(dict(
                model=self.model, data=self.data, q=q, u=u,
                wheel_frames={"left": "left_wheel_link", "right": "right_wheel_link"},
                dt=self.dt,
                target_base_height=float(self.current_z_ref),
                target_roll=0.0,
                q_ref_actuated=self.q_ref_actuated,
                tau_override=tau_override,

                alpha_bg=float(self.get_parameter('alpha_bg').value),
                w_roll_slack=float(self.get_parameter('w_roll_slack').value),
                w_dyn_slack=float(self.get_parameter('w_dyn_slack').value),  # ← now tunable

                knee_tau_scale=float(knee_tau_scale_use),
                hip_tau_scale=float(self.get_parameter('hip_tau_scale').value),
                waist_tau_scale=float(self.get_parameter('waist_tau_scale').value),
                posture_qdd_clip=float(self.get_parameter('posture_qdd_clip').value),

                # Knee spring
                knee_spring_k=float(self.get_parameter('knee_spring_k').value),
                knee_spring_b=float(self.get_parameter('knee_spring_b').value),
                knee_spring_q0_left=float(self.get_parameter('knee_spring_q0_left').value),
                knee_spring_q0_right=float(self.get_parameter('knee_spring_q0_right').value),
                knee_spring_sign_left=float(self.get_parameter('knee_spring_sign_left').value),
                knee_spring_sign_right=float(self.get_parameter('knee_spring_sign_right').value),

                # NEW: Hip spring
                hip_spring_k=float(self.get_parameter('hip_spring_k').value),
                hip_spring_b=float(self.get_parameter('hip_spring_b').value),
                hip_spring_q0_left=float(self.get_parameter('hip_spring_q0_left').value),
                hip_spring_q0_right=float(self.get_parameter('hip_spring_q0_right').value),
                hip_spring_sign_left=float(self.get_parameter('hip_spring_sign_left').value),
                hip_spring_sign_right=float(self.get_parameter('hip_spring_sign_right').value),

                # NEW: Hip gravity compensation
                hip_grav_comp_scale=float(self.get_parameter('hip_grav_comp_scale').value),

                **params
            ))

            self._publish_commands(out)

        except Exception as e:
            self.get_logger().error(f"WBC Fail: {e}")
            self._publish_emergency_stop()

    # ======================================================
    # Helpers
    # ======================================================
    def _publish_commands(self, wbc_output):
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        target_names = [
            "left_waist_joint",
            "left_hip_joint",
            "left_knee_joint",
            "left_wheel_joint",
            "right_waist_joint",
            "right_hip_joint",
            "right_knee_joint",
            "right_wheel_joint",
        ]
        cmd.name = target_names

        tau_map  = {name: float(wbc_output['tau'][i])
                    for i, name in enumerate(wbc_output['actuated_names'])}
        qref_map = {a["name"]: float(self.q_ref_actuated[a["act_idx"]])
                    for a in self.actuated}

        cmd.position = []
        cmd.velocity = []
        cmd.effort   = []

        for name in target_names:
            cmd.position.append(qref_map.get(name, 0.0))
            cmd.velocity.append(0.0)
            cmd.effort.append(tau_map.get(name, 0.0))

        self.pub_joint_commands.publish(cmd)

    def _publish_emergency_stop(self):
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        target_names = [
            "left_waist_joint",  "left_hip_joint",  "left_knee_joint",  "left_wheel_joint",
            "right_waist_joint", "right_hip_joint", "right_knee_joint", "right_wheel_joint",
        ]
        cmd.name     = target_names
        cmd.position = [0.0] * 8
        cmd.velocity = [0.0] * 8
        cmd.effort   = [0.0] * 8

        self.pub_joint_commands.publish(cmd)

    def _apply_startup_standup(self):
        if not bool(self.get_parameter('standup_enable').value) or self._stand_done:
            return
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self._stand_t0 is None:
            self._stand_t0 = now_s
            self._stand_h0 = float(self.target_base_height)

        dur = max(float(self.get_parameter('standup_duration').value), 1e-6)
        s   = _smoothstep01((now_s - self._stand_t0) / dur)

        h_target = float(self.get_parameter('standup_height').value)
        self.target_base_height = float(self._stand_h0 + s * (h_target - self._stand_h0))

        if s >= 1.0:
            self._stand_done = True
            self._stand_t0   = None

    def _boost_active(self) -> bool:
        if not bool(self.get_parameter('height_boost_enable').value) or self._boost_t0 is None:
            return False
        dt_boost = (self.get_clock().now() - self._boost_t0).nanoseconds * 1e-9
        if dt_boost < float(self.get_parameter('height_boost_duration').value):
            return True
        self._boost_t0 = None
        return False

    def set_q_ref_actuated(self, joint_name, value):
        idx = self.act_name_to_act_idx.get(joint_name, None)
        if idx is not None:
            self.q_ref_actuated[idx] = float(value)

    def _refresh_posture_ref_from_current(self, q_local):
        for a in self.actuated:
            if a["name"] in IK_CONTROLLED_JOINTS:
                continue
            act_idx, iq, nq = a["act_idx"], a["iq"], a["nq"]
            if nq == 1:
                self.q_ref_actuated[act_idx] = float(q_local[iq])
            elif nq == 2:
                self.q_ref_actuated[act_idx] = float(np.arctan2(q_local[iq+1], q_local[iq]))

    def _build_tau_override_dict(self):
        tau_override = {}
        try:
            arr = self.get_parameter('tau_limits').value
            if isinstance(arr, (list, tuple)) and len(arr) >= self.na:
                for i, a in enumerate(self.actuated):
                    tau_override[a["name"]] = float(arr[i])
        except Exception:
            pass

        try:
            s = self.get_parameter('tau_override').value
            tau_override.update(_parse_tau_override_string(str(s)))
        except Exception:
            pass

        return tau_override

    def _call_solver_safely(self, kwargs: dict):
        valid = {k: v for k, v in kwargs.items() if k in self.solve_sig.parameters}
        return self.solve_wbc(**valid)


def main(args=None):
    rclpy.init(args=args)
    node = AmbulaWBCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()