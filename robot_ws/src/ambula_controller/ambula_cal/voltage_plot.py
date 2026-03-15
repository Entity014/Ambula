import numpy as np
import matplotlib.pyplot as plt

# ฟังก์ชันสำหรับสร้างเส้นโค้ง Exponential Decay
def get_expo_curve(v_start, v_end, t_total, points=500):
    v_asymp = 13.5 
    k = -np.log((v_end - v_asymp) / (v_start - v_asymp)) / t_total
    time = np.linspace(0, t_total, points)
    voltage = (v_start - v_asymp) * np.exp(-k * time) + v_asymp
    return time, voltage

# ฟังก์ชันสำหรับดึงค่า Voltage เฉพาะจุดที่เวลาหาร 10 ลงตัว
def get_markers_every_10min(t_total, v_start, v_end):
    v_asymp = 13.5
    k = -np.log((v_end - v_asymp) / (v_start - v_asymp)) / t_total
    # สร้าง array ของเวลาทุกๆ 10 นาที (0, 10, 20, ...) ที่ไม่เกิน t_total
    t_markers = np.arange(0, t_total + 1, 10)
    v_markers = (v_start - v_asymp) * np.exp(-k * t_markers) + v_asymp
    return t_markers, v_markers

# 1. ข้อมูลพื้นฐาน
v_start = 16.4
v_end = 14.0

# 2. คำนวณเส้นหลัก (Smooth curves)
t_idle, v_idle = get_expo_curve(v_start, v_end, 240 * 1.7)       
t_balance, v_balance = get_expo_curve(v_start, v_end, 154.28 * 1.7)  
t_height, v_height = get_expo_curve(v_start, v_end, 14.4 * 1.7)  

# 3. คำนวณจุด Marker ทุก 10 นาที
tm_idle, vm_idle = get_markers_every_10min(240 * 1.7, v_start, v_end)
tm_balance, vm_balance = get_markers_every_10min(154.28 * 1.7, v_start, v_end)
tm_height, vm_height = get_markers_every_10min(14.4 * 1.7, v_start, v_end)

# 4. การสร้างกราฟ
plt.figure(figsize=(12, 7))

# พล็อตเส้นหลัก
plt.plot(t_idle, v_idle, color='#2ecc71', label='IDLE Mode', linewidth=2, alpha=0.8)
plt.plot(t_balance, v_balance, color='#3498db', label='BALANCE Mode', linewidth=2, alpha=0.8)
plt.plot(t_height, v_height, color='#e74c3c', label='Height 50 cm', linewidth=2, alpha=0.8)

# พล็อตจุด Marker (ทุก 10 นาที)
plt.scatter(tm_idle, vm_idle, color='#2ecc71', s=30, edgecolors='black', zorder=5)
plt.scatter(tm_balance, vm_balance, color='#3498db', s=30, edgecolors='black', zorder=5)
plt.scatter(tm_height, vm_height, color='#e74c3c', s=30, edgecolors='black', zorder=5)

# วาดเส้นประแสดงระดับแรงดัน Full และ Cut-off
plt.axhline(y=16.4, color='black', linestyle='--', alpha=0.3, label='Full Voltage (16.4V)')
plt.axhline(y=14.0, color='red', linestyle='--', alpha=0.3, label='Cut-off Voltage (14.0V)')

# 5. ตกแต่งกราฟ
plt.title('Battery Discharge', fontsize=14, fontweight='bold', pad=15)
plt.xlabel('Time (Minutes)', fontsize=12)
plt.ylabel('Voltage (V)', fontsize=12)

plt.xlim(0, 410)
plt.ylim(13.8, 16.8)

plt.grid(True, linestyle=':', alpha=0.6)
plt.xticks(np.arange(0, 411, 10), rotation=45) # เอียงตัวเลขแกน X เพื่อให้อ่านง่าย
plt.yticks(np.arange(14.0, 16.6, 0.2))

plt.legend(loc='upper right', frameon=True, shadow=True)
plt.tight_layout()

plt.show()