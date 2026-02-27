#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
import pinocchio as pin

from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Core Library ของคุณ (ต้องเป็นเวอร์ชันที่รองรับ knee spring params ใน solver)
from ambula_controller.wbc_core import (
    load_model,
    solve_wbc_idqp_full_dynamic,
    LQRLookupTable,
    build_actuation_selection,
    wrap_pi
)
from ambula_controller.wbc_kinematic import AmbulaLegIK


def _parse_tau_override_string(s: str):
    """
    รูปแบบ: "left_hip_joint:60,right_hip_joint:60,left_knee_joint:45,..."
    คืนค่า dict {name: float}
    """
    out = {}
    if not s:
        return out
    for item in s.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" not in item:
            continue
        k, v = item.split(":", 1)
        k = k.strip()
        v = v.strip()
        if not k:
            continue
        try:
            out[k] = float(v)
        except ValueError:
            pass
    return out


class AmbulaWBCNode(Node):
    def __init__(self):
        super().__init__('ambula_wbc_node')

        # ========================================================
        # 1) Parameters
        # ========================================================
        self.declare_parameter(
            'urdf_path',
            '/home/xero/Ambula/robot_ws/src/ambula_simulation/ambula_description/urdf/ambula_bot_v3/ambula.urdf'
        )
        self.declare_parameter('control_rate_hz', 400.0)

        # Weights
        self.declare_parameter('weight_height', 80.0)
        self.declare_parameter('weight_roll', 80.0)
        self.declare_parameter('weight_lqr_pitch', 120.0)
        self.declare_parameter('weight_yaw', 10.0)
        self.declare_parameter('weight_posture', 2.0)

        # PD gains
        self.declare_parameter('kp_z', 200.0)
        self.declare_parameter('kd_z', 40.0)
        self.declare_parameter('kp_roll', 200.0)
        self.declare_parameter('kd_roll', 40.0)
        self.declare_parameter('kp_yaw', 50.0)
        self.declare_parameter('kd_yaw', 10.0)
        self.declare_parameter('kp_posture', 50.0)
        self.declare_parameter('kd_posture', 10.0)

        # Limits
        self.declare_parameter('tau_limit', 40.0)      # fallback
        self.declare_parameter('friction_mu', 0.6)

        # NEW: per-joint torque limits
        self.declare_parameter('tau_limits', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('tau_override', "")     # "joint:Nm,joint:Nm,..."

        # ========================================================
        # NEW: Knee Spiral Torsion Spring (parallel at knee joints)
        # tau_spring = sign * ( -k*(q-q0) - b*qdot )
        # units: k [Nm/rad], b [Nms/rad], q0 [rad]
        # ========================================================
        self.declare_parameter('knee_spring_k', 0.0)            # same for L/R (ง่ายสุด)
        self.declare_parameter('knee_spring_b', 0.0)
        self.declare_parameter('knee_spring_q0_left', 0.0)
        self.declare_parameter('knee_spring_q0_right', 0.0)
        self.declare_parameter('knee_spring_sign_left', 1.0)   # flip direction if needed
        self.declare_parameter('knee_spring_sign_right', 1.0)

        # ========================================================
        # 2) Load Model
        # ========================================================
        urdf_path = self.get_parameter('urdf_path').value
        self.rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.dt = 1.0 / self.rate_hz

        self.get_logger().info(f"Loading Model: {urdf_path}")
        self.model, self.data = load_model(urdf_path, use_free_flyer=True)

        # LQR LUT + IK
        self.lqr_table = LQRLookupTable(L_min=0.1, L_max=0.6, num_points=100, dt=self.dt)
        self.leg_ik = AmbulaLegIK(self.model)

        # State
        self.q_current = pin.neutral(self.model)
        self.u_current = np.zeros(self.model.nv)

        self.imu_ready = False
        self.odom_ready = False
        self.joint_ready = False

        # Targets
        self.v_ref = 0.0
        self.w_ref = 0.0
        self.yaw_ref = 0.0
        self.target_L = 0.25
        self.target_X = 0.0

        _, self.na = build_actuation_selection(self.model)
        self.q_ref_actuated = np.zeros(self.na)

        # ========================================================
        # 2.1) Build ACTUATED JOINT LIST (สำคัญมาก: ทำให้ order ตรงกันหมด)
        # ========================================================
        self.actuated = []              # list of dicts: {name, jid, iq, iv, act_idx, nq}
        self.act_name_to_act_idx = {}   # name -> act_idx
        act_idx = 0
        for jid in range(1, self.model.njoints):
            j = self.model.joints[jid]
            if j.nv != 1:
                continue
            name = self.model.names[jid]
            item = {
                "name": name,
                "jid": jid,
                "iq": j.idx_q,
                "iv": j.idx_v,
                "nq": j.nq,
                "act_idx": act_idx,
            }
            self.actuated.append(item)
            self.act_name_to_act_idx[name] = act_idx
            act_idx += 1

        if act_idx != self.na:
            self.get_logger().warn(
                f"Actuated joint count mismatch: built={act_idx} vs na={self.na}. "
                "Check URDF / free-flyer / joint definitions."
            )

        self.actuated_names = [a["name"] for a in self.actuated]
        self.get_logger().info(f"Actuated joints (na={self.na}): {self.actuated_names}")

        # ========================================================
        # 3) Subscribers & Publishers
        # ========================================================
        self.sub_imu = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 50)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 20)
        self.sub_joint_states = self.create_subscription(JointState, '/joint_states/hardware', self.joint_state_callback, 50)

        self.pub_joint_commands = self.create_publisher(JointState, '/cmd_joint', 10)

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"WBC Node running at {self.rate_hz}Hz with Fast IMU Path.")

    # ========================================================
    # Callbacks
    # ========================================================
    def imu_callback(self, msg: Imu):
        # Free-flyer quaternion (x,y,z,w)
        self.q_current[3:7] = [-msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # Base angular velocity in u[3:6] (ตาม convention ที่คุณใช้อยู่)
        self.u_current[3:6] = [-msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        self.imu_ready = True

    def odom_callback(self, msg: Odometry):
        self.q_current[0:2] = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.u_current[0:3] = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        self.odom_ready = True

    def cmd_vel_callback(self, msg: Twist):
        self.v_ref = float(msg.linear.x)
        self.w_ref = float(msg.angular.z)

        if abs(msg.linear.z) > 0.05:
            self.target_L = float(np.clip(
                msg.linear.z,
                self.leg_ik.L_MIN_REACHABLE,
                self.leg_ik.L_MAX_REACHABLE
            ))

    def joint_state_callback(self, msg: JointState):
        """
        อัปเดตเฉพาะ actuated joints ในลำดับ/ชื่อที่ตรงกับโมเดล
        NOTE: ถ้า joint เป็น continuous ใน Pinocchio (nq==2) จะไม่ set position (ต้องใช้ sin/cos)
              แต่จะ set velocity ให้ (ซึ่งพอสำหรับ control ส่วนใหญ่)
        """
        name_to_i = {n: i for i, n in enumerate(msg.name)}

        updated_any = False
        for a in self.actuated:
            n = a["name"]
            if n not in name_to_i:
                continue
            i = name_to_i[n]

            # position
            if len(msg.position) > i and a["nq"] == 1:
                self.q_current[a["iq"]] = msg.position[i]

            # velocity
            if len(msg.velocity) > i:
                self.u_current[a["iv"]] = msg.velocity[i]

            updated_any = True

        if updated_any:
            self.joint_ready = True

    # ========================================================
    # Helpers
    # ========================================================
    def set_q_ref_actuated(self, joint_name, value):
        """
        หยอดค่าใส่ q_ref_actuated ให้ถูกตำแหน่ง "act_idx" (robust)
        """
        idx = self.act_name_to_act_idx.get(joint_name, None)
        if idx is None:
            return
        self.q_ref_actuated[idx] = float(value)

    def _build_tau_override_dict(self):
        """
        รวม per-joint torque limits จาก:
        - tau_limits (array ตาม order actuated)
        - tau_override (string map ตามชื่อ) ซึ่งทับค่าที่ซ้ำ
        สุดท้ายคืน dict {joint_name: tau_max}
        """
        tau_override = {}

        # 1) array ตามลำดับ actuated
        arr = self.get_parameter('tau_limits').value
        if isinstance(arr, (list, tuple)) and len(arr) == self.na:
            for i, a in enumerate(self.actuated):
                try:
                    tau_override[a["name"]] = float(arr[i])
                except Exception:
                    pass

        # 2) string map ทับค่า
        s = self.get_parameter('tau_override').value
        tau_override.update(_parse_tau_override_string(str(s)))

        return tau_override

    # ========================================================
    # Control loop
    # ========================================================
    def control_loop(self):
        if not (self.imu_ready and self.odom_ready and self.joint_ready):
            return

        # 1) Read tuning params
        params = {
            'w_height': float(self.get_parameter('weight_height').value),
            'w_roll': float(self.get_parameter('weight_roll').value),
            'w_lqr_pitch': float(self.get_parameter('weight_lqr_pitch').value),
            'w_yaw': float(self.get_parameter('weight_yaw').value),
            'w_posture': float(self.get_parameter('weight_posture').value),

            'kp_z': float(self.get_parameter('kp_z').value),
            'kd_z': float(self.get_parameter('kd_z').value),
            'kp_roll': float(self.get_parameter('kp_roll').value),
            'kd_roll': float(self.get_parameter('kd_roll').value),
            'kp_yaw': float(self.get_parameter('kp_yaw').value),
            'kd_yaw': float(self.get_parameter('kd_yaw').value),
            'kp_post': float(self.get_parameter('kp_posture').value),
            'kd_post': float(self.get_parameter('kd_posture').value),

            'tau_limit': float(self.get_parameter('tau_limit').value),
            'mu': float(self.get_parameter('friction_mu').value),
        }

        # 2) Update yaw ref + IK
        self.yaw_ref = wrap_pi(self.yaw_ref + self.w_ref * self.dt)

        q_hip, q_knee = self.leg_ik.compute(self.target_X, -self.target_L)
        for side in ["left", "right"]:
            self.set_q_ref_actuated(f"{side}_hip_joint", q_hip)
            self.set_q_ref_actuated(f"{side}_knee_joint", q_knee)

        # 3) Per-joint torque limits dict
        tau_override = self._build_tau_override_dict()

        # 4) Knee spring params
        knee_k = float(self.get_parameter('knee_spring_k').value)
        knee_b = float(self.get_parameter('knee_spring_b').value)
        knee_q0_L = float(self.get_parameter('knee_spring_q0_left').value)
        knee_q0_R = float(self.get_parameter('knee_spring_q0_right').value)
        knee_sgn_L = float(self.get_parameter('knee_spring_sign_left').value)
        knee_sgn_R = float(self.get_parameter('knee_spring_sign_right').value)

        try:
            out = solve_wbc_idqp_full_dynamic(
                model=self.model,
                data=self.data,
                q=self.q_current,
                u=self.u_current,
                wheel_frames={"left": "left_wheel_link", "right": "right_wheel_link"},
                lqr_lut=self.lqr_table,
                dt=self.dt,
                target_L_pendulum=self.target_L,
                yaw_ref=self.yaw_ref,
                v_ref=self.v_ref,
                q_ref_actuated=self.q_ref_actuated,
                alpha_bg=20.0,

                # torque limits
                tau_override=tau_override,

                # knee spiral torsion spring (parallel)
                knee_spring_k=knee_k,
                knee_spring_b=knee_b,
                knee_spring_q0_left=knee_q0_L,
                knee_spring_q0_right=knee_q0_R,
                knee_spring_sign_left=knee_sgn_L,
                knee_spring_sign_right=knee_sgn_R,

                # weights/gains/limits
                **params
            )

            # 5) Publish command (name/effort/position ต้องตรง order เดียวกัน)
            cmd = JointState()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.name = self.actuated_names
            cmd.effort = out['tau'].tolist()                         # len = na
            cmd.position = self.q_ref_actuated.tolist()              # len = na
            cmd.velocity = [0.0] * self.na
            self.pub_joint_commands.publish(cmd)

        except Exception as e:
            self.get_logger().error(f"WBC Fail: {e}")

            # Safe mode: torque = 0 เฉพาะ actuated joints
            cmd = JointState()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.name = self.actuated_names
            cmd.effort = [0.0] * self.na
            cmd.position = self.q_ref_actuated.tolist()
            cmd.velocity = [0.0] * self.na
            self.pub_joint_commands.publish(cmd)


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