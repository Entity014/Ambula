import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline

height = 50
# 1. Load data from CSV file
# เปลี่ยน path ให้ตรงกับตำแหน่งไฟล์ของคุณ
file_path = f'/home/xero/Ambula/robot_ws/src/ambula_controller/ambula_cal/Data/Data - {height}cm.csv'
df = pd.read_csv(file_path)

# Clean data: ลบแถวที่เป็นค่าว่างถ้ามี
df = df.dropna(subset=['ระยะยืดจริงที่วัดได้ (เซนติเมตร)'])

# 2. Extract values and Add (0,0) point
# แกน X: ครั้งที่ (Iterations)
# แกน Y: ระยะยืดจริง (Measured Height)
trials = np.insert(df['ครั้งที่'].values, 0, 0)
measured_height = np.insert(df['ระยะยืดจริงที่วัดได้ (เซนติเมตร)'].values, 0, 0)

target_height = df['ระยะยืดเป้าหมาย (เซนติเมตร)'].iloc[0]

# --- ฟังก์ชันสำหรับสร้างเส้นโค้ง ---
def make_smooth(x, y):
    x_new = np.linspace(x.min(), x.max(), 300) 
    spl = make_interp_spline(x, y, k=3)
    y_smooth = spl(x_new)
    return x_new, y_smooth

# สร้างข้อมูลที่โค้งมน
x_smooth, y_smooth = make_smooth(trials, measured_height)

# 3. Plotting
plt.figure(figsize=(10, 6))

# วาดเส้นโค้ง Measured Height - สีน้ำเงิน
plt.plot(x_smooth, y_smooth, color='red', label='Measured Height', linewidth=2)

# วาดจุดข้อมูลจริง (Scatter)
plt.scatter(trials, measured_height, color='red', marker='o', s=30, alpha=0.5)

# วาดเส้น Target Height (Setpoint) - เส้นประสีดำ
plt.axhline(y=target_height, color='black', linestyle='--', label=f'Setpoint Height ({height} cm)', linewidth=2)

# 4. Aesthetics & English Labels
plt.xlabel('Iterations', fontsize=12)
plt.ylabel('Height (cm)', fontsize=12)

# ตั้งค่าแกนให้เริ่มที่ 0
plt.xlim(0, 10.5) 
plt.ylim(0, 60) # ปรับให้สูงกว่าเป้าหมายเล็กน้อยเพื่อให้เห็นภาพชัดเจน

# Dotted Grid
plt.grid(True, linestyle=':', color='gray', alpha=0.7)

# ปรับ Ticks
plt.xticks(np.arange(0, 11, 1))
plt.yticks(np.arange(0, 61, 5))

plt.legend(loc='upper right', frameon=True, edgecolor='black', fontsize=10)
plt.title(f'Robot Height Adjustment Performance Test', fontsize=14)

plt.tight_layout()
plt.show()