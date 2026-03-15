import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline

# 1. Load data from CSV files
# เปลี่ยน path ให้ตรงกับตำแหน่งไฟล์ของคุณ
df_up = pd.read_csv('/home/xero/Ambula/robot_ws/src/ambula_controller/ambula_cal/Data/Data - 13deg_up.csv')
df_down = pd.read_csv('/home/xero/Ambula/robot_ws/src/ambula_controller/ambula_cal/Data/Data - 13deg_down.csv')

# Clean data: ลบแถวที่ไม่มีข้อมูลความเร็ว
df_up = df_up.dropna(subset=['ความเร็ว'])
df_down = df_down.dropna(subset=['ความเร็ว'])

# 2. Extract values and Add (0,0) point
# แกน X: ครั้งที่ (Iterations), แกน Y: ความเร็ว (Velocity)
trials_up = np.insert(df_up['ครั้งที่'].values, 0, 0)
vel_up = np.insert(df_up['ความเร็ว'].values, 0, 0)

trials_down = np.insert(df_down['ครั้งที่'].values, 0, 0)
vel_down = np.insert(df_down['ความเร็ว'].values, 0, 0)

# Target Velocity (จากข้อมูลดูเหมือนจะตั้งไว้ที่ 0.5 m/s)
target_velocity = 0.75

# --- ฟังก์ชันสำหรับสร้างเส้นโค้ง ---
def make_smooth(x, y):
    x_new = np.linspace(x.min(), x.max(), 300) 
    spl = make_interp_spline(x, y, k=3)
    y_smooth = spl(x_new)
    return x_new, y_smooth

# สร้างข้อมูลที่โค้งมน
x_up_smooth, y_up_smooth = make_smooth(trials_up, vel_up)
x_down_smooth, y_down_smooth = make_smooth(trials_down, vel_down)

# 3. Plotting
plt.figure(figsize=(10, 6))

plt.plot(x_up_smooth, y_up_smooth, color='blue', label='Measured Velocity (Ascending)', linewidth=2)
plt.scatter(trials_up, vel_up, color='blue', marker='o', s=30, alpha=0.5)

plt.plot(x_down_smooth, y_down_smooth, color='red', label='Measured Velocity (Descending)', linewidth=2)
plt.scatter(trials_down, vel_down, color='red', marker='s', s=30, alpha=0.5)

# Plot Target Velocity (Setpoint) - เส้นประสีดำ
plt.axhline(y=target_velocity, color='black', linestyle='--', label='Setpoint Velocity (0.75 m/s)', linewidth=2)

# 4. Aesthetics & English Labels
plt.xlabel('Iterations', fontsize=12)
plt.ylabel('Velocity (m/s)', fontsize=12)

# ตั้งค่าแกนให้เริ่มที่ 0
plt.xlim(0, 10.5) 
plt.ylim(0, 1.0) 

# Dotted Grid
plt.grid(True, linestyle=':', color='gray', alpha=0.7)

# ปรับ Ticks
plt.xticks(np.arange(0, 11, 1))
plt.yticks(np.arange(0, 1.1, 0.1))

plt.legend(loc='upper right', frameon=True, edgecolor='black', fontsize=10)
plt.title('Velocity Performance Test (13° Slope)', fontsize=14)

plt.tight_layout()
plt.show()