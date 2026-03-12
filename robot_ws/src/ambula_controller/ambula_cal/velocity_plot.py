import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline # นำเข้าเครื่องมือสำหรับทำเส้นโค้ง

# 1. Load data from CSV files
# เปลี่ยน path ให้ตรงกับไฟล์ของคุณ
df_forward = pd.read_csv('/home/xero/Ambula/robot_ws/src/ambula_controller/ambula_cal/Data/Data - 3m_forward_rude.csv')
df_backward = pd.read_csv('/home/xero/Ambula/robot_ws/src/ambula_controller/ambula_cal/Data/Data - 3m_backward_rude.csv')

# Clean data: Remove rows where Velocity is NaN
df_forward = df_forward.dropna(subset=['ความเร็ว'])
df_backward = df_backward.dropna(subset=['ความเร็ว'])

# 2. Extract values and Add (0,0) point
trials_f = np.insert(df_forward['ครั้งที่'].values, 0, 0)
vel_forward = np.insert(df_forward['ความเร็ว'].values, 0, 0)

trials_b = np.insert(df_backward['ครั้งที่'].values, 0, 0)
vel_backward = np.insert(df_backward['ความเร็ว'].values, 0, 0)

# --- ฟังก์ชันสำหรับสร้างเส้นโค้ง ---
def make_smooth(x, y):
    # สร้างจุด X ที่ละเอียดขึ้น (จาก 0-10 เป็น 300 จุด)
    x_new = np.linspace(x.min(), x.max(), 300) 
    # ใช้ Cubic Spline (k=3) เพื่อคำนวณค่า Y ให้โค้งมน
    spl = make_interp_spline(x, y, k=3)
    y_smooth = spl(x_new)
    return x_new, y_smooth

# สร้างข้อมูลที่โค้งมนแล้ว
x_f_smooth, y_f_smooth = make_smooth(trials_f, vel_forward)
x_b_smooth, y_b_smooth = make_smooth(trials_b, vel_backward)

# Target Velocity
target_velocity = 0.75 

# 3. Plotting
plt.figure(figsize=(10, 6))

# วาดเส้นโค้ง (ใช้ข้อมูล smooth ที่สร้างขึ้น)
plt.plot(x_f_smooth, y_f_smooth, color='blue', label='Measured Velocity (Forward)', linewidth=2)
plt.plot(x_b_smooth, y_b_smooth, color='red', label='Measured Velocity (Backward)', linewidth=2)

# (ทางเลือก) วาดจุดข้อมูลจริงทับลงไปเพื่อให้เห็นตำแหน่งที่วัดได้จริง
plt.scatter(trials_f, vel_forward, color='blue', marker='o', s=30, alpha=0.5)
plt.scatter(trials_b, vel_backward, color='red', marker='s', s=30, alpha=0.5)

# Plot Target Velocity (Setpoint)
plt.axhline(y=target_velocity, color='black', linestyle='--', label='Setpoint Velocity (0.75 m/s)', linewidth=2)

# 4. Aesthetics & Labels
plt.xlabel('Iterations', fontsize=12)
plt.ylabel('Velocity (m/s)', fontsize=12)

plt.xlim(0, 10.5) 
plt.ylim(0, 1.0) 

plt.grid(True, linestyle=':', color='gray', alpha=0.7)
plt.xticks(np.arange(0, 11, 1))
plt.yticks(np.arange(0, 1.1, 0.1))

plt.legend(loc='upper right', frameon=True, edgecolor='black', fontsize=10)
plt.title('Velocity Performance Test (3m)', fontsize=14)
plt.tight_layout()
plt.show()