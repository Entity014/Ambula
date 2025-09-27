#!/usr/bin/env python3
import time
import numpy as np
import odrive
from odrive.enums import AxisState, ControlMode, InputMode
import matplotlib.pyplot as plt

# ====== S-curve 7-phase (jerk-limited) ======
def s_curve(x0, x1, v_max, a_max, j_max, dt=0.001):
    """
    7-phase jerk-limited S-curve (third-order)
    Return: t, x, v, a, j
    """
    D_signed = x1 - x0
    s = 1 if D_signed >= 0 else -1
    D = abs(D_signed)
    if D < 1e-15:
        t = np.array([0.0])
        return t, np.array([x0]), np.array([0.0]), np.array([0.0]), np.array([0.0])

    # --- เลือก Tj เบื้องต้นจากเพดาน a และ v (สูตรมาตรฐาน S-curve) ---
    Tj = min(a_max / j_max, v_max / a_max)
    # --- เลือกชนิดช่วงเร่งอย่างถูกต้อง (third-order S-curve) ---
    if v_max >= (a_max * a_max) / j_max:
        # ถึง a_max ได้ (ทราเพโซอิดฝั่งเร่ง)
        Tj = a_max / j_max
        Tc = (v_max / a_max) - Tj       # >= 0
    else:
        # ไม่ถึง a_max (สามเหลี่ยมฝั่งเร่ง)
        Tj = np.sqrt(v_max / j_max)
        Tc = 0.0

    # --- คำนวณใหม่ทั้งหมดด้วย Tj/Tc ที่ “สรุปแล้ว” ---
    a_reached = min(a_max, j_max * Tj)        # ถ้าเป็นสามเหลี่ยม a_reached < a_max
    x1a = (1/6) * j_max * Tj**3
    v1a = 0.5 * j_max * Tj**2
    x2a = v1a * Tc + 0.5 * a_reached * Tc**2
    v2a = v1a + a_reached * Tc
    x3a = v2a * Tj + 0.5 * a_reached * Tj**2 - (1/6) * j_max * Tj**3
    v_peak = v2a + a_reached * Tj - 0.5 * j_max * Tj**2
    D_no_cruise = 2.0 * (x1a + x2a + x3a)

    # --- ตัดสินว่ามี cruise ไหม จากค่าที่เพิ่งคำนวณใหม่ ---
    if D_no_cruise <= D + 1e-12:
        Tv = (D - D_no_cruise) / max(v_peak, 1e-12)   # มี cruise หรือพอดี
    else:
        # ระยะสั้นเกิน → ไม่มี cruise: แก้โปรไฟล์ให้สั้นลง
        # 5-phase (ถึง a_reached) หรือ 3-phase (ไม่ถึง a_reached) ตามกรณี
        # วิธีง่าย: แก้ v_peak ใหม่ด้วยการแก้สมการระยะ (เวอร์ชันย่อ)
        # ---- ทางเลือกสั้น: ใช้ตัวแก้ของคุณเดิม แต่ต้อง "คำนวณ x1a.. v_peak.. D_no_cruise ใหม่" หลังได้ Tj/Tc ใหม่ทุกครั้ง ----
        j = j_max
        Tj_try = a_reached / j  # = Tj ถ้าเป็นสามเหลี่ยม; = a_max/j ถ้าเป็นทราเพโซฯ
        A = 1.0
        B = 3.0 * Tj_try
        C = 2.0 * Tj_try**2 - (D / (j * Tj_try))
        disc = B*B - 4*A*C
        Tc = max(0.0, (-B + np.sqrt(max(0.0, disc))) / (2*A))
        Tj = Tj_try
        # คำนวณใหม่ทั้งหมดอีกครั้งหลังได้ Tj/Tc สุดท้าย
        a_reached = min(a_max, j_max * Tj)
        x1a = (1/6) * j_max * Tj**3
        v1a = 0.5 * j_max * Tj**2
        x2a = v1a * Tc + 0.5 * a_reached * Tc**2
        v2a = v1a + a_reached * Tc
        x3a = v2a * Tj + 0.5 * a_reached * Tj**2 - (1/6) * j_max * Tj**3
        v_peak = v2a + a_reached * Tj - 0.5 * j_max * Tj**2
        Tv = 0.0


    # --- ความเร่งที่ "ถึงจริง" (สำคัญมากสำหรับเคส Tc=0) ---
    a_reached = min(a_max, j_max*Tj)

    # --- ป้องกันเลขลบเล็กน้อย + คำนวณหมุดเวลาใหม่ ---
    Tv = max(0.0, Tv)
    t1 = Tj
    t2 = Tj + Tc
    t3 = 2*Tj + Tc             # Ta
    t4 = t3 + Tv
    t5 = t4 + Tj
    t6 = t5 + Tc
    t7 = t6 + Tj
    T_total = t7

    # --- ค่า ณ เวลาใดๆ (ใช้ a_reached เสมอเมื่อไม่มีช่วงถึง a_max จริง) ---
    def seg_values_at(t):
        if t < t1:  # 1) +jerk
            tau = t
            j = j_max
            a = j_max*tau
            v = 0.5*j_max*tau**2
            x = (1/6)*j_max*tau**3

        elif t < t2:  # 2) +a const
            tau = t - t1
            j = 0.0
            a = a_reached
            v = v1a + a_reached*tau
            x = x1a + v1a*tau + 0.5*a_reached*tau**2

        elif t < t3:  # 3) -jerk
            tau = t - t2
            j = -j_max
            a = a_reached - j_max*tau
            v = v2a + a_reached*tau - 0.5*j_max*tau**2
            x = x1a + x2a + v2a*tau + 0.5*a_reached*tau**2 - (1/6)*j_max*tau**3

        elif t < t4:  # 4) const vel
            tau = t - t3
            j = 0.0
            a = 0.0
            v = v_peak
            x = (x1a + x2a + x3a) + v*tau

        elif t < t5:  # 5) -jerk (เริ่มเบรก)
            tau = t - t4
            j = -j_max
            a = -j_max*tau
            v = v_peak - 0.5*j_max*tau**2
            x = (x1a + x2a + x3a) + v_peak*Tv + v_peak*tau - (1/6)*j_max*tau**3

        elif t < t6:  # 6) -a const
            tau = t - t5
            j = 0.0
            a = -a_reached
            v = (v_peak - 0.5*j_max*Tj**2) + (-a_reached)*tau
            x = (x1a + x2a + x3a) + v_peak*Tv + (v_peak*Tj - (1/6)*j_max*Tj**3) \
                + (v_peak - 0.5*j_max*Tj**2)*tau - 0.5*a_reached*tau**2

        else:        # 7) +jerk (ปล่อยเบรก)
            tau = t - t6
            j = j_max
            a = -a_reached + j_max*tau
            v = (v_peak - 0.5*j_max*Tj**2) - a_reached*Tc - a_reached*tau + 0.5*j_max*tau**2
            x_t6 = (x1a + x2a + x3a) + v_peak*Tv + (v_peak*Tj - (1/6)*j_max*Tj**3) \
                 + (v_peak - 0.5*j_max*Tj**2)*Tc - 0.5*a_reached*Tc**2
            x = x_t6 + ((v_peak - 0.5*j_max*Tj**2) - a_reached*Tc)*tau \
                 - 0.5*a_reached*tau**2 + (1/6)*j_max*tau**3

        return x, v, a, j

    # --- สุ่มตามเวลา: ให้ลงท้ายที่ T_total เป๊ะ และ snap ปลายทาง ---
    N = max(1, int(np.ceil(T_total/dt)))
    t = np.linspace(0.0, T_total, N+1)
    X = np.zeros_like(t); V = np.zeros_like(t); A = np.zeros_like(t); J = np.zeros_like(t)
    for i, ti in enumerate(t):
        x, v, a, j = seg_values_at(ti)
        X[i] = x; V[i] = v; A[i] = a; J[i] = j

    return t, x0 + s*X, s*V, s*A, s*J

def prepare_axis_for_position(axis, vel_limit_turn_s=50.0, current_lim=20.0):
    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    axis.controller.config.control_mode = ControlMode.POSITION_CONTROL
    axis.controller.config.input_mode   = InputMode.PASSTHROUGH
    axis.controller.config.vel_limit = float(vel_limit_turn_s)
    axis.motor.config.current_lim    = float(current_lim)

def execute_scurve_position_and_log(axis, x_traj_turn, v_traj_turn_s, t_traj):
    """
    ป้อนตำแหน่งแบบ S-curve และเก็บ log:
      - time, x_cmd, v_cmd
      - x_meas (encoder.pos_estimate), v_meas (encoder.vel_estimate)
      - Iq_measured (motor.current_control.Iq_measured)
    """
    log_t  = []
    log_xc = []
    log_vc = []
    log_xm = []
    log_vm = []
    log_iq = []

    t0 = time.perf_counter()
    for xi, vi, ti in zip(x_traj_turn, v_traj_turn_s, t_traj):
        while (time.perf_counter() - t0) < ti:
            pass
        # สั่งตำแหน่ง
        axis.controller.input_pos = float(xi)

        # อ่านค่าจริง
        now = time.perf_counter() - t0
        x_meas = float(axis.encoder.pos_estimate)     # หน่วย: turns
        v_meas = float(axis.encoder.vel_estimate)     # หน่วย: turn/s
        iq     = float(axis.motor.current_control.Iq_measured)  # A

        # เก็บ log
        log_t.append(now)
        log_xc.append(float(xi))
        log_vc.append(float(vi))
        log_xm.append(x_meas)
        log_vm.append(v_meas)
        log_iq.append(iq)

        print(f"t={now:.3f}s  x_cmd={xi:.3f}  x_meas={x_meas:.3f}  v_meas={v_meas:.2f}  Iq={iq:.2f}A")

    return (np.array(log_t), np.array(log_xc), np.array(log_vc),
            np.array(log_xm), np.array(log_vm), np.array(log_iq))

def main():
    print("🔌 กำลังค้นหา ODrive ...")
    odrv0 = odrive.find_any(timeout=15)
    if odrv0 is None:
        print("❌ ไม่พบ ODrive")
        return

    axis_leg = odrv0.axis0

    # เตรียมแกน
    prepare_axis_for_position(axis_leg, vel_limit_turn_s=25.0, current_lim=15.0)

    # ====== กำหนด trajectory ======
    x0_turn = -1.0
    x1_turn = 3.0
    v_max   = 15.0     # [turn/s]
    a_max   = 50.0     # [turn/s^2]
    j_max   = 500.0    # [turn/s^3]
    dt      = 0.01    # 100 Hz

    t, x_cmd, v_cmd, a_cmd, j_cmd = s_curve(x0_turn, x1_turn, v_max, a_max, j_max, dt=dt)

    # ====== รัน + เก็บ log ======
    print("▶️ เริ่มสั่ง S-curve ไปยัง ODrive ...")
    log = execute_scurve_position_and_log(axis_leg, x_traj_turn=x_cmd, v_traj_turn_s=v_cmd, t_traj=t)
    t_log, x_cmd_log, v_cmd_log, x_meas_log, v_meas_log, iq_log = log
    print("✅ เสร็จสิ้น")

    # ====== Plot ======
    plt.figure(figsize=(10,7))

    # 1) Position
    plt.subplot(3,1,1)
    plt.plot(t,     x_cmd,      label="x_cmd (turns)")
    plt.plot(t_log, x_meas_log, label="x_meas (turns)", linestyle="--")
    plt.ylabel("Position (turns)")
    plt.legend(); plt.grid(True)

    # 2) Velocity
    plt.subplot(3,1,2)
    plt.plot(t,     v_cmd,      label="v_cmd (turn/s)")
    plt.plot(t_log, v_meas_log, label="v_meas (turn/s)", linestyle="--")
    plt.ylabel("Velocity (turn/s)")
    plt.legend(); plt.grid(True)

    # 3) Motor current (Iq)
    plt.subplot(3,1,3)
    plt.plot(t_log, iq_log, label="Iq_measured (A)")
    plt.xlabel("Time (s)")
    plt.ylabel("Iq (A)")
    plt.legend(); plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
