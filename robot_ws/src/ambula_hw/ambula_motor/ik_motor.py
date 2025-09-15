#!/usr/bin/env python3
import odrive
from odrive.enums import AxisState
import numpy as np
import time

# ===== Serial Number ของ ODrive =====
SER0 = "315533683334"   # odrv0
SER1 = "315733593334"   # odrv1

# ===== ค่าคาลิเบรท (วัดจากการเคลื่อนจริง) =====
POS_ZERO_0 = -0.2
POS_ZERO_1 = 0.4
K_DEG_PER_TURN_0 = -18.019
K_DEG_PER_TURN_1 = -38.087

# ===== ความยาวลิ้งก์ (m) =====
L1, L2 = 0.20, 0.25

# ===== ออฟเซ็ตกรอบมุม =====
OFFSET_Q1 = 90.0
OFFSET_Q2 = 152.0

# ===== FK/IK =====
def fk_pos_rr(q1, q2, l1, l2, degrees=False):
    if degrees:
        q1 = np.deg2rad(q1); q2 = np.deg2rad(q2)
    th = q1 + q2
    x = l1*np.cos(q1) + l2*np.cos(th)
    y = l1*np.sin(q1) + l2*np.sin(th)
    return float(x), float(y)

def ik_rr_all(x, y, l1, l2, degrees=False):
    r2 = x*x + y*y
    if r2 > (l1+l2)**2 + 1e-9 or r2 < (l1-l2)**2 - 1e-9:
        return None
    c2 = (r2 - l1*l1 - l2*l2)/(2*l1*l2)
    c2 = np.clip(c2, -1.0, 1.0)
    s2a = np.sqrt(max(0.0, 1.0 - c2*c2))
    sols = {}
    for name, s2 in (("elbow_up", +s2a), ("elbow_down", -s2a)):
        q2 = np.arctan2(s2, c2)
        q1 = np.arctan2(y, x) - np.arctan2(l2*s2, l1 + l2*c2)
        sols[name] = (np.rad2deg(q1), np.rad2deg(q2)) if degrees else (q1, q2)
    return sols

def choose_ik_solution(solutions, q1_now, q2_now):
    best = None; best_err = None
    for _, (q1, q2) in solutions.items():
        err = (q1-q1_now)**2 + (q2-q2_now)**2
        if best is None or err < best_err:
            best = (q1, q2); best_err = err
    return best

# ===== mapping =====
def joint_deg_from_pos(pos, pos_zero, k_deg_per_turn):
    return (pos - pos_zero) * k_deg_per_turn

def pos_from_joint_deg_axis0(q_deg):
    return -( q_deg / K_DEG_PER_TURN_0 + POS_ZERO_0 )

def pos_from_joint_deg_axis1(q_deg):
    return ( q_deg / K_DEG_PER_TURN_1 + POS_ZERO_1 )

def setpoints_from_xy(x_t, y_t, q1_now_disp, q2_now_disp, prefer="auto"):
    q1_now_math = q1_now_disp - OFFSET_Q1
    q2_now_math = q2_now_disp - OFFSET_Q2
    sols = ik_rr_all(x_t, y_t, L1, L2, degrees=True)
    if sols is None:
        raise ValueError("Target unreachable")
    if prefer in sols:
        q1_math_deg, q2_math_deg = sols[prefer]
    else:
        q1_math_deg, q2_math_deg = choose_ik_solution(sols, q1_now_math, q2_now_math)
    q1_disp = q1_math_deg + OFFSET_Q1
    if q1_disp > 0:
        q1_disp -= 360
    q2_disp = q2_math_deg + OFFSET_Q2
    pos_cmd0 = pos_from_joint_deg_axis0(q1_disp)
    pos_cmd1 = pos_from_joint_deg_axis1(q2_disp)
    return pos_cmd0, pos_cmd1, (q1_disp, q2_disp)

# ===== main =====
def main():
    odrv0, odrv1 = odrive.find_sync(serial_number=[SER0, SER1])
    axis0 = odrv0.axis0; axis1 = odrv1.axis0
    axis0.requested_state = AxisState.IDLE
    axis1.requested_state = AxisState.IDLE
    # axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    # axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL

    x_t, y_t = -0.05, 0.35  # จุดเป้าหมาย

    while True:
        # อ่าน encoder
        pos1 = axis0.encoder.pos_estimate
        pos2 = axis1.encoder.pos_estimate
        q1_now_disp = joint_deg_from_pos(-pos1, POS_ZERO_0, K_DEG_PER_TURN_0)
        q2_now_disp = joint_deg_from_pos( pos2, POS_ZERO_1, K_DEG_PER_TURN_1)

        # FK ของมุมปัจจุบัน → pos (x_now,y_now)
        x_now, y_now = fk_pos_rr(q1_now_disp - OFFSET_Q1, q2_now_disp - OFFSET_Q2,
                                 L1, L2, degrees=True)

        # IK ของเป้า → pos_cmd
        pos_cmd0, pos_cmd1, (q1_cmd, q2_cmd) = setpoints_from_xy(
            x_t, y_t, q1_now_disp, q2_now_disp, prefer="elbow_down"
        )


        print(f"Now: q=({q1_now_disp:+7.2f},{q2_now_disp:+7.2f}) deg "
              f"EE=({x_now:+.3f},{y_now:+.3f}) m || "
              f"Target=({x_t:+.3f},{y_t:+.3f}) m "
              f"q_cmd=({q1_cmd:+7.2f},{q2_cmd:+7.2f}) deg "
              f"pos_cmd=({pos_cmd0:+.4f},{pos_cmd1:+.4f}) turns")

        # axis0.controller.input_pos = pos_cmd0
        # axis1.controller.input_pos = pos_cmd1
        time.sleep(0.02)

if __name__ == "__main__":
    main()
