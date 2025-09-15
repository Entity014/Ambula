#!/usr/bin/env python3
import odrive
import time
import matplotlib.pyplot as plt
from collections import deque
from odrive.enums import AxisState

def main():
    # 1. ค้นหาและเชื่อมต่อ ODrive
    odrv0 = odrive.find_any(timeout=10)
    if odrv0 is None:
        print("❌ ไม่พบ ODrive")
        return
    
    # buffer เก็บข้อมูล 200 จุดล่าสุด
    max_len = 200
    t = deque([0.0]*max_len, maxlen=max_len)
    pos_data = deque([0.0]*max_len, maxlen=max_len)
    vel_data = deque([0.0]*max_len, maxlen=max_len)

    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8,6), sharex=True)

    line_pos, = ax1.plot(t, pos_data, label="Position [turns]")
    ax1.set_ylabel("Position (turns)")
    ax1.legend()

    line_vel, = ax2.plot(t, vel_data, label="Velocity [turns/s]", color="orange")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Velocity (turns/s)")
    ax2.legend()

    start_time = time.time()
    
    odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.input_pos = 0.0

    while True:
        now = time.time() - start_time
        pos = odrv0.axis0.encoder.pos_estimate
        vel = odrv0.axis0.encoder.vel_estimate

        t.append(now)
        pos_data.append(pos)
        vel_data.append(vel)

        line_pos.set_xdata(t)
        line_pos.set_ydata(pos_data)

        line_vel.set_xdata(t)
        line_vel.set_ydata(vel_data)

        ax1.set_xlim(max(0, now-4), now+0.1)  # แสดงย้อนหลัง 4 วิ
        ax2.set_xlim(max(0, now-4), now+0.1)

        ax1.relim(); ax1.autoscale_view(scalex=False, scaley=True)
        ax2.relim(); ax2.autoscale_view(scalex=False, scaley=True)

        plt.pause(0.01)

if __name__ == "__main__":
    main()
