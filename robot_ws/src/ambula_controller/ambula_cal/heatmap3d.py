import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# 1. โหลดข้อมูล (ใช้ Path ตามที่คุณระบุ)
file_path = '/home/xero/Ambula/robot_ws/src/ambula_controller/ambula_cal/Data/AMBULA - Rough Area Map.csv'
df = pd.read_csv(file_path, header=None)

# 2. เตรียมข้อมูล Grid 2
# ค่า Z (ข้อมูลหลัก)
grid2_values = df.iloc[3:, 13:22].astype(float).values
# ค่าพิกัด X (Width) และ Y (Length)
x_labels = df.iloc[2, 13:22].astype(float).values
y_labels = df.iloc[3:, 12].astype(float).values

# สร้าง Meshgrid
X, Y = np.meshgrid(x_labels, y_labels)
Z = grid2_values

# 3. สร้างรูป 3D Surface
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# --- จุดที่แก้ไข: เพิ่ม vmin=-20 และ vmax=0 เพื่อล็อคสี ---
surf = ax.plot_surface(X, Y, Z, 
                       cmap='RdYlBu', 
                       edgecolor='grey', 
                       alpha=0.8, 
                       linewidth=0.1,
                       vmin=-21.7,   # กำหนดให้สีแดงเข้มที่สุดคือ -20
                       vmax=0)     # กำหนดให้สีน้ำเงินเข้มที่สุดคือ 0

# 4. เพิ่ม Colorbar (มันจะอ้างอิงค่า -20 ถึง 0 ตามที่ตั้งไว้ใน surf)
fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, label='Depth Value (Scale -20 to 0)')

# 5. ตั้งค่าแกนและชื่อ
ax.set_title('3D Heatmap of Surface', fontsize=15, pad=20)
ax.set_xlabel('Width (X)')
ax.set_ylabel('Length (Y)')
ax.set_zlabel('Depth (Z)')

# ปรับระดับความลึกของแกน Z ให้เห็นสเกลถึง -20 ชัดๆ
ax.set_zlim(-20, 0)

# ปรับมุมมอง
ax.view_init(elev=30, azim=45) 

plt.tight_layout()
plt.show()