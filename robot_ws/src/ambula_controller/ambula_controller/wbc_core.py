#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ascento-style WBC / ID-QP with sigma-based rolling contact
Pitch Task removed — torso pitch managed externally (e.g. LQR on wheels).
"""

import numpy as np
import pinocchio as pin
import osqp
from scipy import sparse

# =========================================================
# Joints whose posture reference must NOT be overwritten
# =========================================================
IK_CONTROLLED_JOINTS = {
    "left_hip_joint",
    "right_hip_joint",
    "left_knee_joint",
    "right_knee_joint",
}

# =========================================================
# Math utils
# =========================================================
def skew(v):
    v = np.asarray(v, dtype=float).reshape(3)
    vx, vy, vz = v
    return np.array([[0.0, -vz,  vy],
                     [vz,  0.0, -vx],
                     [-vy, vx,  0.0]], dtype=float)

def normalize(v, eps=1e-12):
    v = np.asarray(v, dtype=float).reshape(-1)
    n = np.linalg.norm(v)
    if n < eps:
        return np.zeros_like(v)
    return v / n

def require_frame_id(model: pin.Model, frame_name: str) -> int:
    fid = model.getFrameId(frame_name)
    if fid == len(model.frames):
        raise ValueError(f"Frame '{frame_name}' not found.")
    return fid

def require_joint_id(model: pin.Model, joint_name: str) -> int:
    jid = model.getJointId(joint_name)
    if jid == 0:
        raise ValueError(f"Joint '{joint_name}' not found.")
    return jid

def rpy_from_R_zyx(R):
    sp = -R[2, 0]
    sp = np.clip(sp, -1.0, 1.0)
    pitch = np.arcsin(sp)
    if abs(np.cos(pitch)) < 1e-9:
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    else:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw  = np.arctan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw], dtype=float)

# =========================================================
# Model & Frame Helpers
# =========================================================
def load_model(urdf_path: str, use_free_flyer: bool = True):
    if use_free_flyer:
        model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
    else:
        model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    return model, data

def set_freeflyer_pose(model: pin.Model, q: np.ndarray,
                       position_xyz=(0.0, 0.0, 0.0),
                       quat_xyzw=(0.0, 0.0, 0.0, 1.0)):
    q[0:3] = np.array(position_xyz, dtype=float)
    quat = np.array(quat_xyzw, dtype=float)
    quat = quat / (np.linalg.norm(quat) + 1e-12)
    q[3:7] = quat

def set_joint_position_by_name(model: pin.Model, q: np.ndarray, joint_name: str, value: float):
    jid = require_joint_id(model, joint_name)
    iq = model.idx_qs[jid]
    q[iq] = float(value)

def set_joint_velocity_by_name(model: pin.Model, u: np.ndarray, joint_name: str, value: float):
    jid = require_joint_id(model, joint_name)
    iv = model.idx_vs[jid]
    u[iv] = float(value)


def compute_adjusted_height_reference(L_ref, theta_pitch, phi_roll, cos_min=0.35):
    # กันกรณีล้ม/เอียงมากจน cos ~ 0 แล้ว height task ตาย
    cp = max(np.cos(theta_pitch), cos_min)
    cr = max(np.cos(phi_roll),  cos_min)
    return L_ref * cp * cr

def build_contact_frame_R_CI(n_I: np.ndarray, wheel_forward_I: np.ndarray) -> np.ndarray:
    z = normalize(n_I)
    f = wheel_forward_I - (z @ wheel_forward_I) * z
    x = normalize(f)
    if np.linalg.norm(x) < 1e-9:
        ex = np.array([1.0, 0.0, 0.0])
        x = normalize(ex - (z @ ex) * z)
    y = normalize(np.cross(z, x))
    x = normalize(np.cross(y, z))
    return np.column_stack((x, y, z)).T

def build_control_frame_N(pC_L_I, pC_R_I, heading_I, n_I=np.array([0.0, 0.0, 1.0])):
    n_I = normalize(n_I)
    
    # === แก้ตรงนี้สำคัญที่สุด ===
    y = normalize(pC_L_I - pC_R_I)          # ← เปลี่ยนจาก pC_R - pC_L เป็น pC_L - pC_R
    
    h = normalize(heading_I - (n_I @ heading_I) * n_I)
    x = normalize(h - (y @ h) * y)
    
    if np.linalg.norm(x) < 1e-9:
        x = normalize(np.array([1.0, 0.0, 0.0]) - (y @ np.array([1.0, 0.0, 0.0])) * y)
    
    z = normalize(np.cross(x, y))
    if z @ n_I < 0.0:
        z = -z
        x = -x                      # ยังคง right-handed
    
    pG_I = 0.5 * (pC_L_I + pC_R_I)
    return np.column_stack((x, y, z)), pG_I

def compute_heading_from_base(model, data, base_frame, n_I):
    R_IB = data.oMf[require_frame_id(model, base_frame)].rotation
    h = R_IB[:, 0]
    return normalize(h - (n_I @ h) * n_I)

def axis_from_frame_rotation(R_world_frame: np.ndarray, local_axis: str) -> np.ndarray:
    if local_axis == "x": return R_world_frame[:, 0]
    if local_axis == "y": return R_world_frame[:, 1]
    if local_axis == "z": return R_world_frame[:, 2]
    raise ValueError("local_axis must be one of {'x','y','z'}")

def sigma_from_ground_normal(n_I: np.ndarray, R_IW: np.ndarray) -> float:
    n_I = normalize(n_I)
    R_WI = R_IW.T
    x = R_WI @ n_I
    return float(np.arctan2(x[0], x[2]))

def sigma_dot_from_omega(n_I: np.ndarray, R_IW: np.ndarray, omega_IW_I: np.ndarray) -> float:
    n_I = normalize(n_I)
    R_WI = R_IW.T
    x = R_WI @ n_I
    omega_W = R_WI @ omega_IW_I
    x_dot = -np.cross(omega_W, x)
    denom = x[0]**2 + x[2]**2
    if denom < 1e-12:
        return 0.0
    return float((x[2] * x_dot[0] - x[0] * x_dot[2]) / denom)

def r_WC_in_W(sigma: float, rho: float) -> np.ndarray:
    return np.array([rho * np.sin(sigma), 0.0, rho * np.cos(sigma)], dtype=float)

def dr_WC_dsigma_in_W(sigma: float, rho: float) -> np.ndarray:
    return np.array([rho * np.cos(sigma), 0.0, -rho * np.sin(sigma)], dtype=float)

# =========================================================
# Wheel Kinematics
# =========================================================
def wheel_contact_kinematics_fast(model, data, q, v, wheel_frame: str, rho: float,
                                  n_world: np.ndarray, wheel_forward_axis_local: str,
                                  ref=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED):
    fid = require_frame_id(model, wheel_frame)
    oMw = data.oMf[fid]
    p_IW = oMw.translation.copy()
    R_IW = oMw.rotation.copy()

    fwd_I = axis_from_frame_rotation(R_IW, wheel_forward_axis_local)
    R_CI = build_contact_frame_R_CI(n_world, fwd_I)
    R_IC = R_CI.T

    J6 = pin.getFrameJacobian(model, data, fid, ref)
    v6 = pin.getFrameVelocity(model, data, fid, ref)
    a6 = pin.getFrameClassicalAcceleration(model, data, fid, ref)

    sigma = sigma_from_ground_normal(n_world, R_IW)
    sigma_dot = sigma_dot_from_omega(n_world, R_IW, v6.angular)

    r_W  = r_WC_in_W(sigma, rho)
    dr_W = dr_WC_dsigma_in_W(sigma, rho)

    r_I = R_IW @ r_W
    if (r_I @ normalize(n_world)) > 0.0:
        r_I = -r_I
        dr_W = -dr_W

    t_I = R_IW @ dr_W

    Jv = J6[0:3, :]
    Jw = J6[3:6, :]
    Jc_I = Jv + skew(r_I) @ Jw

    v_c_I = v6.linear + np.cross(v6.angular, r_I) + t_I * sigma_dot
    a_c0_I = (
        a6.linear
        + np.cross(a6.angular, r_I)
        + np.cross(v6.angular, np.cross(v6.angular, r_I))
        + np.cross(v6.angular, t_I * sigma_dot)
    )

    return dict(
        wheel_frame=wheel_frame, R_CI=R_CI, R_IC=R_IC,
        r_I=r_I, p_IW=p_IW, p_IC=p_IW + r_I,
        Jc_I=Jc_I, v_c_I=v_c_I, a_c0_I=a_c0_I,
        sigma=sigma, sigma_dot=sigma_dot, v_hub_I=v6.linear
    )

def build_rolling_constraints_fast(model, data, q, u, wheel_frames: dict, rho: float,
                                   n_world: np.ndarray, wheel_forward_axis_local: str,
                                   alpha_bg: float = 50.0,
                                   ref=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED):
    v = u.copy()
    pin.forwardKinematics(model, data, q, v, np.zeros_like(v))
    pin.computeJointJacobians(model, data, q)
    pin.updateFramePlacements(model, data)

    Sxz = np.array([[1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0]], dtype=float)

    A_list, rhs_list, contacts = [], [], []

    for side in ["left", "right"]:
        info = wheel_contact_kinematics_fast(
            model, data, q, v, wheel_frames[side], rho, n_world, wheel_forward_axis_local, ref
        )
        contacts.append(info)

        Jc_C   = info["R_CI"] @ info["Jc_I"]
        v_c_C  = info["R_CI"] @ info["v_c_I"]
        a0_c_C = info["R_CI"] @ info["a_c0_I"]

        A_sel = Sxz @ Jc_C
        rhs_sel = -(Sxz @ (a0_c_C + alpha_bg * v_c_C))

        A_list.append(A_sel)
        rhs_list.append(rhs_sel.reshape(-1))

    return np.vstack(A_list), np.hstack(rhs_list).reshape(-1), contacts

# =========================================================
# Actuation & Limits
# =========================================================
def build_actuation_selection(model: pin.Model, exclude_keywords=("wheel",)):
    actuated_ivs = []
    actuated_names = []
    for joint_id in range(1, model.njoints):
        joint = model.joints[joint_id]
        jname = model.names[joint_id]
        if joint.nv == 1:
            if any(k in jname.lower() for k in exclude_keywords):
                continue
            actuated_ivs.append(joint.idx_v)
            actuated_names.append(jname)

    na = len(actuated_ivs)
    S = np.zeros((na, model.nv), dtype=float)
    for i, iv in enumerate(actuated_ivs):
        S[i, iv] = 1.0
    return S, na, actuated_ivs, actuated_names

def build_wheel_ivs(model: pin.Model, wheel_keywords=("wheel",)):
    wheel_ivs = []
    wheel_names = []
    for joint_id in range(1, model.njoints):
        joint = model.joints[joint_id]
        jname = model.names[joint_id]
        if joint.nv == 1 and any(k in jname.lower() for k in wheel_keywords):
            wheel_ivs.append(joint.idx_v)
            wheel_names.append(jname)
    return wheel_ivs, wheel_names

def add_least_squares_cost(H, g, A_full, b, w):
    if w > 0.0 and A_full.size > 0:
        H += w * (A_full.T @ A_full)
        g += -w * (A_full.T @ b)
    return H, g

def build_tau_limits_per_actuated_joint(model: pin.Model, actuated_ivs: list,
                                        default_tau: float = 40.0, tau_override: dict | None = None):
    if tau_override is None: tau_override = {}
    na = len(actuated_ivs)
    tau_max = np.zeros(na, dtype=float)
    iv_to_info = {}
    for j in range(1, model.njoints):
        if model.joints[j].nv == 1:
            iv = model.joints[j].idx_v
            effort = float(model.effortLimit[iv]) if iv < len(model.effortLimit) else 0.0
            iv_to_info[iv] = {"name": model.names[j], "effort": effort}

    for i, iv in enumerate(actuated_ivs):
        info = iv_to_info[iv]
        jname = info["name"]
        lim = float(tau_override.get(jname, info["effort"]))
        if lim <= 1e-6: lim = float(default_tau)
        tau_max[i] = lim
    return np.maximum(tau_max, 1e-6)

def build_tau_regularization_weights(actuated_names: list, w_tau_base: float = 1e-3,
                                     knee_scale: float = 30.0, hip_scale: float = 1.0,
                                     waist_scale: float = 5.0) -> np.ndarray:
    w = np.ones(len(actuated_names), dtype=float) * w_tau_base
    for i, name in enumerate(actuated_names):
        n = name.lower()
        if "knee" in n: w[i] = w_tau_base * knee_scale
        elif "hip" in n: w[i] = w_tau_base * hip_scale
        elif "waist" in n: w[i] = w_tau_base * waist_scale
    return w

def knee_spiral_spring_tau_nv(model: pin.Model, q: np.ndarray, u: np.ndarray,
                              knee_spring_k: float = 0.0, knee_spring_b: float = 0.0,
                              knee_spring_q0_left: float = 0.0, knee_spring_q0_right: float = 0.0,
                              knee_spring_sign_left: float = 1.0, knee_spring_sign_right: float = 1.0,
                              left_knee_joint: str = "left_knee_joint", right_knee_joint: str = "right_knee_joint"):
    tau = np.zeros(model.nv, dtype=float)
    k, b = float(knee_spring_k), float(knee_spring_b)
    if abs(k) < 1e-12 and abs(b) < 1e-12: return tau
    try:
        jL = model.joints[require_joint_id(model, left_knee_joint)]
        if jL.nv == 1 and jL.nq == 1:
            tau[jL.idx_v] += float(knee_spring_sign_left) * (-k * (float(q[jL.idx_q]) - float(knee_spring_q0_left)) - b * float(u[jL.idx_v]))
    except Exception: pass
    try:
        jR = model.joints[require_joint_id(model, right_knee_joint)]
        if jR.nv == 1 and jR.nq == 1:
            tau[jR.idx_v] += float(knee_spring_sign_right) * (-k * (float(q[jR.idx_q]) - float(knee_spring_q0_right)) - b * float(u[jR.idx_v]))
    except Exception: pass
    return tau

# =========================================================
# Tasks definition
# =========================================================
def build_base_height_task_N(model, data, q, u, base_frame, R_IN, pG_I, z_ref, kp, kd,
                             ref=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED):
    fid = require_frame_id(model, base_frame)
    p_NB = R_IN.T @ (data.oMf[fid].translation - pG_I)
    v_NB = R_IN.T @ pin.getFrameVelocity(model, data, fid, ref).linear
    a0_NB = R_IN.T @ pin.getFrameClassicalAcceleration(model, data, fid, ref).linear

    zdd_des = kp * (z_ref - float(p_NB[2])) - kd * float(v_NB[2])
    zdd_des = float(np.clip(zdd_des, -8.0, 8.0))

    Jv = pin.getFrameJacobian(model, data, fid, ref)[0:3, :]
    A = np.array([[0.0, 0.0, 1.0]]) @ (R_IN.T @ Jv)
    b = np.array([zdd_des - float(np.array([0.0, 0.0, 1.0]) @ a0_NB)], dtype=float)
    return A, b

def build_base_lateral_tilt_task(model, data, base_frame, target_tilt=0.0, kp=80.0, kd=28.0):
    fid = require_frame_id(model, base_frame)
    R_WB = data.oMf[fid].rotation.copy()

    g_body = R_WB.T @ np.array([0.0, 0.0, -1.0])
    if g_body[2] < 0.0:          # หากหัวคว่ำ (กรณีหุ่นล้ม)
        g_body = -g_body

    # === FLIP SIGN ที่สำคัญที่สุด ===
    tilt = np.arctan2(g_body[0], g_body[2])   

    omega = pin.getFrameVelocity(model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).angular
    tilt_rate = omega[0]                      

    dd_des = kp * (target_tilt - tilt) - kd * tilt_rate
    dd_des = np.clip(dd_des, -20.0, 20.0)

    Jw = pin.getFrameJacobian(model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[3:6, :]
    A = np.array([[1.0, 0.0, 0.0]]) @ Jw      

    alpha0 = pin.getFrameClassicalAcceleration(model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).angular
    b = np.array([dd_des - float(alpha0[0])])  

    return A, b, tilt, tilt_rate

def build_posture_task(model, q, u, q_ref_actuated, actuated_ivs, actuated_names,
                       kp_hip=250.0, kd_hip=20.0,
                       kp_knee=100.0, kd_knee=10.0,
                       qdd_clip: float = 300.0):
    A_rows = []
    b_rows = []

    iv_to_iq = {model.joints[j].idx_v: model.joints[j].idx_q
                for j in range(1, model.njoints) if model.joints[j].nv == 1}

    for i, iv in enumerate(actuated_ivs):
        iq = iv_to_iq[iv]
        jname = actuated_names[i].lower()

        row = np.zeros(model.nv, dtype=float)
        row[iv] = 1.0

        if "hip" in jname:
            kp_use, kd_use = kp_hip, kd_hip
        else:
            kp_use, kd_use = kp_knee, kd_knee

        err = float(q_ref_actuated[i] - q[iq])
        raw_qdd = kp_use * err - kd_use * float(u[iv])
        qdd_des = float(np.clip(raw_qdd, -qdd_clip, qdd_clip))

        A_rows.append(row)
        b_rows.append(qdd_des)

    if len(A_rows) == 0:
        return np.zeros((0, model.nv)), np.zeros((0,), dtype=float)

    return np.vstack(A_rows), np.array(b_rows, dtype=float)

def wheel_ddq_meas_to_rows(model: pin.Model, wheel_ddq_meas: dict | None, wheel_ivs: list):
    if not wheel_ddq_meas: return [], np.zeros((0,), dtype=float)
    ivs_used, vals = [], []
    for jname, ddq in wheel_ddq_meas.items():
        try:
            j = model.joints[require_joint_id(model, jname)]
            if j.nv == 1 and (j.idx_v in wheel_ivs):
                ivs_used.append(j.idx_v)
                vals.append(float(ddq))
        except Exception: pass
    if not ivs_used: return [], np.zeros((0,), dtype=float)
    return ivs_used, np.array(vals, dtype=float)

# =========================================================
# Main Full Dynamic ID-QP (LEG ONLY) — no pitch task
# =========================================================
def solve_wbc_idqp_full_dynamic_leg_only(
        model, data, q, u, wheel_frames: dict,
        rho=0.07, mu=0.6, tau_limit=40.0, tau_override=None,
        alpha_bg=50.0, wheel_forward_axis_local="x", n_world=np.array([0.0, 0.0, 1.0]),
        base_frame="base_link", dt=0.0025,
        target_base_height=0.40, target_roll=0.0,
        q_ref_actuated=None,
        kp_z=200.0, kd_z=40.0,
        kp_roll=200.0, kd_roll=40.0,
        kp_hip=250.0, kd_hip=20.0,
        kp_knee=100.0, kd_knee=10.0,
        w_height=80.0, w_roll=80.0, w_posture=2.0,
        w_ddq=1.0, w_tau=1e-3, w_f=1e-3, w_roll_slack=1e4,
        knee_tau_scale: float = 5.0, hip_tau_scale: float = 1.0, waist_tau_scale: float = 5.0,
        use_dyn_slack=True, w_dyn_slack=1e4,
        wheel_ddq_mode="soft", wheel_ddq_meas: dict | None = None, w_wheel_ddq=1e3,
        knee_spring_k: float = 0.0, knee_spring_b: float = 0.0,
        knee_spring_q0_left: float = 0.0, knee_spring_q0_right: float = 0.0,
        knee_spring_sign_left: float = 1.0, knee_spring_sign_right: float = 1.0,
        left_knee_joint: str = "left_knee_joint", right_knee_joint: str = "right_knee_joint",
        posture_qdd_clip: float = 300.0,
):
    nv = model.nv

    S, na, actuated_ivs, actuated_names = build_actuation_selection(model, exclude_keywords=("wheel",))
    wheel_ivs, wheel_joint_names = build_wheel_ivs(model, wheel_keywords=("wheel",))

    if q_ref_actuated is None:
        q_ref_actuated = np.zeros(na, dtype=float)
        iv_to_iq = {model.joints[j].idx_v: model.joints[j].idx_q for j in range(1, model.njoints) if model.joints[j].nv == 1}
        for i, iv in enumerate(actuated_ivs): q_ref_actuated[i] = q[iv_to_iq[iv]]

    pin.computeAllTerms(model, data, q, u)
    M = data.M.copy()
    h = data.nle.copy()

    tau_spring_nv = knee_spiral_spring_tau_nv(
        model, q, u, knee_spring_k=knee_spring_k, knee_spring_b=knee_spring_b,
        knee_spring_q0_left=knee_spring_q0_left, knee_spring_q0_right=knee_spring_q0_right,
        knee_spring_sign_left=knee_spring_sign_left, knee_spring_sign_right=knee_spring_sign_right,
        left_knee_joint=left_knee_joint, right_knee_joint=right_knee_joint
    )

    Aroll, rhs_roll, contacts = build_rolling_constraints_fast(
        model, data, q, u, wheel_frames, rho=rho, n_world=n_world,
        wheel_forward_axis_local=wheel_forward_axis_local, alpha_bg=alpha_bg
    )
    nc = len(contacts)
    Jc = np.vstack([c["Jc_I"] for c in contacts])
    Rblk = np.zeros((3 * nc, 3 * nc), dtype=float)
    for i in range(nc): Rblk[3*i:3*i+3, 3*i:3*i+3] = contacts[i]["R_IC"]

    n_f = 3 * nc
    n_roll = Aroll.shape[0]
    n_sdyn = nv if use_dyn_slack else 0
    n_x = nv + na + n_f + n_roll + n_sdyn

    idx_ddq0, idx_tau0, idx_f0 = 0, nv, nv + na
    idx_sroll0, idx_sdyn0 = nv + na + n_f, nv + na + n_f + n_roll

    A_dyn = np.zeros((nv, n_x), dtype=float)
    A_dyn[:, idx_ddq0:idx_ddq0+nv] = M
    A_dyn[:, idx_tau0:idx_tau0+na] = -S.T
    A_dyn[:, idx_f0:idx_f0+n_f] = -(Jc.T @ Rblk)
    if use_dyn_slack: A_dyn[:, idx_sdyn0:idx_sdyn0+nv] = np.eye(nv)

    A_roll_eq = np.zeros((n_roll, n_x), dtype=float)
    A_roll_eq[:, idx_ddq0:idx_ddq0+nv] = Aroll
    A_roll_eq[:, idx_sroll0:idx_sroll0+n_roll] = np.eye(n_roll)

    Aeq = np.vstack([A_dyn, A_roll_eq])
    beq = np.hstack([(-h + tau_spring_nv).reshape(-1), rhs_roll.reshape(-1)])

    if wheel_ddq_mode.lower() == "hard":
        ivs_used, ddq_vals = wheel_ddq_meas_to_rows(model, wheel_ddq_meas, wheel_ivs)
        if len(ivs_used) > 0:
            A_w = np.zeros((len(ivs_used), n_x), dtype=float)
            for i, iv in enumerate(ivs_used): A_w[i, idx_ddq0 + iv] = 1.0
            Aeq, beq = np.vstack([Aeq, A_w]), np.hstack([beq, ddq_vals])

    tau_max_vec = build_tau_limits_per_actuated_joint(model, actuated_ivs, default_tau=float(tau_limit), tau_override=tau_override)
    A_tau = np.zeros((na, n_x), dtype=float)
    for i in range(na): A_tau[i, idx_tau0 + i] = 1.0

    A_fric = np.zeros((5 * nc, n_x), dtype=float)
    row = 0
    for i in range(nc):
        base = idx_f0 + 3*i
        A_fric[row:row+2, base+0], A_fric[row:row+2, base+2] = np.array([1.0, -1.0]), -mu
        row += 2
        A_fric[row:row+2, base+1], A_fric[row:row+2, base+2] = np.array([1.0, -1.0]), -mu
        row += 2
        A_fric[row, base+2] = -1.0
        row += 1

    iv_list = list(actuated_ivs) + list(wheel_ivs)
    n_kin = len(iv_list)
    A_kin = np.zeros((n_kin, n_x), dtype=float)
    ddq_lb, ddq_ub = np.zeros(n_kin), np.zeros(n_kin)
    kp_limit = 20.0
    iv_to_joint = {model.joints[j].idx_v: model.joints[j] for j in range(1, model.njoints) if model.joints[j].nv == 1}

    for i, iv in enumerate(iv_list):
        joint = iv_to_joint[iv]
        iq = joint.idx_q
        qi, vi = float(q[iq]), float(u[iv])
        q_min, q_max = float(model.lowerPositionLimit[iq]), float(model.upperPositionLimit[iq])
        v_max_urdf = float(model.velocityLimit[iv]) if iv < len(model.velocityLimit) else 0.0
        if v_max_urdf <= 1e-4: v_max_urdf = 100.0
        v_min_urdf = -v_max_urdf
        is_continuous = ((joint.nq == 2) or (abs(q_max - q_min) < 1e-4) or (q_max > 1e4) or (q_min < -1e4))
        if is_continuous:
            v_allowed_max, v_allowed_min = v_max_urdf, v_min_urdf
        else:
            v_allowed_max = min(v_max_urdf, kp_limit * (q_max - qi))
            v_allowed_min = max(v_min_urdf, kp_limit * (q_min - qi))
        A_kin[i, idx_ddq0 + iv] = 1.0
        ddq_ub[i] = float(np.clip((v_allowed_max - vi) / dt, -1000.0, 1000.0))
        ddq_lb[i] = float(np.clip((v_allowed_min - vi) / dt, -1000.0, 1000.0))
        if ddq_lb[i] > ddq_ub[i]:
            ddq_lb[i] = ddq_ub[i] = 0.5 * (ddq_lb[i] + ddq_ub[i])

    Aineq = np.vstack([A_tau, A_fric, A_kin])
    l_ineq = np.hstack([-tau_max_vec, -np.inf * np.ones(5 * nc), ddq_lb])
    u_ineq = np.hstack([+tau_max_vec, np.zeros(5 * nc), ddq_ub])

    H, g_obj = np.zeros((n_x, n_x), dtype=float), np.zeros(n_x)
    H[idx_ddq0:idx_ddq0+nv, idx_ddq0:idx_ddq0+nv] += w_ddq * np.eye(nv)

    w_tau_vec = build_tau_regularization_weights(
        actuated_names, w_tau_base=w_tau,
        knee_scale=knee_tau_scale, hip_scale=hip_tau_scale, waist_scale=waist_tau_scale
    )
    H[idx_tau0:idx_tau0+na, idx_tau0:idx_tau0+na] += np.diag(w_tau_vec)
    H[idx_f0:idx_f0+n_f, idx_f0:idx_f0+n_f] += w_f * np.eye(n_f)
    H[idx_sroll0:idx_sroll0+n_roll, idx_sroll0:idx_sroll0+n_roll] += w_roll_slack * np.eye(n_roll)
    if use_dyn_slack: H[idx_sdyn0:idx_sdyn0+nv, idx_sdyn0:idx_sdyn0+nv] += w_dyn_slack * np.eye(nv)

    pin.forwardKinematics(model, data, q, u, np.zeros_like(u))
    pin.updateFramePlacements(model, data)
    heading_I = compute_heading_from_base(model, data, base_frame, normalize(n_world))
    R_IN, pG_I = build_control_frame_N(contacts[0]["p_IC"], contacts[1]["p_IC"], heading_I, normalize(n_world))
    base_fid = require_frame_id(model, base_frame)
    R_NB = R_IN.T @ data.oMf[base_fid].rotation
    current_roll, current_pitch, _ = rpy_from_R_zyx(R_NB)

    # ── Height Task ──────────────────────────────────────────────────────────
    adjusted_z_ref = compute_adjusted_height_reference(target_base_height, current_pitch, current_roll)
    A_h, b_h = build_base_height_task_N(model, data, q, u, base_frame, R_IN, pG_I, adjusted_z_ref, kp_z, kd_z)
    A_full = np.zeros((A_h.shape[0], n_x), dtype=float)
    A_full[:, idx_ddq0:idx_ddq0+nv] = A_h
    H, g_obj = add_least_squares_cost(H, g_obj, A_full, b_h, w_height)

    # ── Roll Task (Gravity-based) ─────────────────────────────────────────────
    A_r, b_r, roll_cur, rolldot_cur = build_base_lateral_tilt_task(
        model, data, base_frame, target_roll, kp_roll, kd_roll
    )
    A_full = np.zeros((A_r.shape[0], n_x), dtype=float)
    A_full[:, idx_ddq0:idx_ddq0+nv] = A_r
    H, g_obj = add_least_squares_cost(H, g_obj, A_full, b_r, w_roll)

    # ── Posture Task ─────────────────────────────────────────────────────────
    if w_posture > 0.0:
        A_post, b_post = build_posture_task(
            model, q, u, q_ref_actuated, actuated_ivs, actuated_names,
            kp_hip=kp_hip, kd_hip=kd_hip,
            kp_knee=kp_knee, kd_knee=kd_knee,
            qdd_clip=posture_qdd_clip
        )
        A_full = np.zeros((A_post.shape[0], n_x), dtype=float)
        A_full[:, idx_ddq0:idx_ddq0+nv] = A_post
        H, g_obj = add_least_squares_cost(H, g_obj, A_full, b_post, w_posture)

    # ── Wheel ddq soft constraint ─────────────────────────────────────────────
    if wheel_ddq_mode.lower() == "soft":
        ivs_used, ddq_vals = wheel_ddq_meas_to_rows(model, wheel_ddq_meas, wheel_ivs)
        if len(ivs_used) > 0 and w_wheel_ddq > 0.0:
            A_wsoft = np.zeros((len(ivs_used), n_x), dtype=float)
            for i, iv in enumerate(ivs_used): A_wsoft[i, idx_ddq0 + iv] = 1.0
            H, g_obj = add_least_squares_cost(H, g_obj, A_wsoft, ddq_vals, w_wheel_ddq)

    prob = osqp.OSQP()
    A_osqp = sparse.csc_matrix(np.vstack([Aeq, Aineq]))
    P = sparse.csc_matrix((H + H.T) * 0.5)

    prob.setup(
        P=P, q=g_obj, A=A_osqp,
        l=np.concatenate([beq, l_ineq]),
        u=np.concatenate([beq, u_ineq]),
        verbose=False, polish=False,
        eps_abs=5e-3, eps_rel=5e-3, eps_prim_inf=1e-4, eps_dual_inf=1e-4,
        adaptive_rho=True, max_iter=2000, warm_start=True
    )

    res = prob.solve()
    if res.info.status_val not in (1, 2):
        raise RuntimeError(f"OSQP failed: {res.info.status}")

    x = res.x
    ddq     = x[idx_ddq0:idx_ddq0+nv]
    tau_leg = x[idx_tau0:idx_tau0+na]
    fC      = x[idx_f0:idx_f0+n_f].reshape(nc, 3)
    s_roll  = x[idx_sroll0:idx_sroll0+n_roll]
    s_dyn   = x[idx_sdyn0:idx_sdyn0+nv] if use_dyn_slack else np.zeros((0,), dtype=float)

    return dict(
        ddq=ddq, tau=tau_leg,
        actuated_names=actuated_names, wheel_joint_names=wheel_joint_names,
        fC=fC, roll_slack=s_roll, dyn_slack=s_dyn,
        dyn_res=A_dyn @ x + h - tau_spring_nv,
        roll_res=Aroll @ ddq - rhs_roll,
        adjusted_z_ref=adjusted_z_ref,
        target_roll=target_roll,
        tau_max_vec=tau_max_vec,
        roll=roll_cur, roll_rate=rolldot_cur,
        pitch=current_pitch,
        tau_spring_nv=tau_spring_nv,
        w_tau_vec=w_tau_vec,
    )


# =========================================================
# Execution (test)
# =========================================================
if __name__ == "__main__":
    urdf_path = "/home/xero/Ambula/robot_ws/src/ambula_simulation/ambula_description/urdf/ambula_bot_v3/ambula.urdf"

    try:
        model, data = load_model(urdf_path, use_free_flyer=True)
        q = pin.neutral(model)
        u = np.zeros(model.nv)

        set_freeflyer_pose(model, q, position_xyz=(0.0, 0.0, 0.0), quat_xyzw=(0.0, 0.0, 0.0, 1.0))

        hip_init_angle  = 1.57
        knee_init_angle = 2.44
        for jn, val in [("left_hip_joint",  hip_init_angle),
                        ("right_hip_joint", hip_init_angle),
                        ("left_knee_joint",  knee_init_angle),
                        ("right_knee_joint", knee_init_angle)]:
            try:
                set_joint_position_by_name(model, q, jn, val)
            except Exception:
                pass

        _, na, actuated_ivs, actuated_names = build_actuation_selection(model, exclude_keywords=("wheel",))
        q_ref_actuated = np.zeros(na, dtype=float)
        iv_to_iq = {model.joints[j].idx_v: model.joints[j].idx_q
                   for j in range(1, model.njoints) if model.joints[j].nv == 1}
        for i, iv in enumerate(actuated_ivs):
            q_ref_actuated[i] = q[iv_to_iq[iv]]

        tau_override = {
            "left_hip_joint": 40.0, "right_hip_joint": 40.0,
            "left_knee_joint": 20.0, "right_knee_joint": 20.0,
        }
        wheel_ddq_meas = {"left_wheel_joint": 0.0, "right_wheel_joint": 0.0}

        out = solve_wbc_idqp_full_dynamic_leg_only(
            model, data, q, u,
            wheel_frames={"left": "left_wheel_link", "right": "right_wheel_link"},
            target_base_height=0.30,
            target_roll=0.0,
            rho=0.07, mu=0.6,
            tau_limit=40.0,
            tau_override=tau_override,
            alpha_bg=50.0,
            q_ref_actuated=q_ref_actuated,
            kp_hip=300.0, kd_hip=30.0,
            kp_knee=50.0, kd_knee=14.0,
            w_height=30.0,
            w_roll=0.0,
            w_posture=20.0,
            w_roll_slack=1e4,
            use_dyn_slack=True,
            w_dyn_slack=1e4,
            wheel_ddq_mode="soft",
            wheel_ddq_meas=wheel_ddq_meas,
            w_wheel_ddq=1e3,
            knee_spring_k=0.5,
            knee_spring_b=0.3,
            knee_spring_q0_left=knee_init_angle,
            knee_spring_q0_right=knee_init_angle,
            knee_tau_scale=5.0,
            hip_tau_scale=1.0,
            posture_qdd_clip=200.0,
        )

        np.set_printoptions(suppress=True, formatter={'float_kind': '{:.4f}'.format})
        print("\n=== ID-QP (leg-only, no pitch task) ===")
        print("Joints    :", out["actuated_names"])
        print("tau (Nm)  :", out["tau"])
        print("w_tau_vec :", out["w_tau_vec"])
        print("pitch     :", out["pitch"])
        print("dyn_res   :", float(np.linalg.norm(out["dyn_res"])))
        print("roll_res  :", float(np.linalg.norm(out["roll_res"])))

    except Exception as e:
        print(f"Error: {e}")