#!/usr/bin/env python3
import odrive
from odrive.enums import AxisState, ControlMode
import time
import matplotlib.pyplot as plt

def main():
    print("🔍 กำลังค้นหา ODrive ...")
    odrv0 = odrive.find_any(timeout=10)
    if odrv0 is None:
        print("❌ ไม่พบ ODrive")
        return

    axis = odrv0.axis0

    # ===== Set Closed-loop Velocity Control =====
    print("⚙️ กำลังเซ็ต closed-loop velocity control...")
    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL

    # ===== สั่งให้มอเตอร์วิ่ง =====
    target_vel = 10.0   # turn/s (1 รอบ = 360°)
    print(f"🚀 วิ่งที่ {target_vel} turn/s")
    axis.controller.input_vel = target_vel

    # ===== เก็บข้อมูลความเร็ว =====
    times = []
    velocities = []
    t0 = time.time()
    duration = 5.0   # วินาที
    dt = 0.01

    while (time.time() - t0) < duration:
        vel = axis.encoder.vel_estimate
        t = time.time() - t0
        times.append(t)
        velocities.append(vel)
        time.sleep(dt)

    # ===== หยุดมอเตอร์ =====
    print("🛑 หยุดมอเตอร์")
    axis.controller.input_vel = 0.0

    # ===== Plot =====
    plt.figure()
    plt.plot(times, velocities, label="Measured Velocity [turn/s]")
    plt.axhline(target_vel, color="red", linestyle="--", label="Target Velocity")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [turn/s]")
    plt.title("ODrive Axis0 Velocity Tracking")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
