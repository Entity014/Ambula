import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 1. โหลดข้อมูลจากไฟล์ CSV
df = pd.read_csv('/home/xero/Ambula/robot_ws/src/ambula_controller/ambula_cal/Data/AMBULA - Rough Area Map.csv', header=None)

# 2. เลือกข้อมูลเฉพาะ Grid 2 (คอลัมน์ดัชนี 13-21 ตั้งแต่แถวที่ 3 เป็นต้นไป)
# ข้อมูลค่าในตาราง
grid2_values = df.iloc[3:, 13:22].astype(float).values
# แกน X (แถวที่ 2 คอลัมน์ 13-21)
x_labels = df.iloc[2, 13:22].astype(float).astype(int).values
# แกน Y (แถวที่ 3 เป็นต้นไป คอลัมน์ที่ 12)
y_labels = df.iloc[3:, 12].astype(float).astype(int).values

# 3. สร้าง Heatmap ด้วย Matplotlib (เพียวๆ ไม่ต้องพึ่ง seaborn)
fig, ax = plt.subplots(figsize=(6, 8))
# เลือกสี (cmap) เช่น 'RdYlBu', 'viridis', 'coolwarm'
im = ax.imshow(grid2_values, cmap='RdYlBu', origin='lower')

# เพิ่มแถบสีด้านข้าง (Colorbar)
cbar = ax.figure.colorbar(im, ax=ax)
cbar.ax.set_ylabel("Value", rotation=-90, va="bottom")

# ตั้งค่าจุดบนแกน X และ Y
ax.set_xticks(np.arange(len(x_labels)))
ax.set_yticks(np.arange(len(y_labels)))
# ใส่ชื่อกำกับแกน
ax.set_xticklabels(x_labels)
ax.set_yticklabels(y_labels)

# เขียนตัวเลขลงในช่องสี่เหลี่ยมแต่ละช่อง
for i in range(len(y_labels)):
    for j in range(len(x_labels)):
        text = ax.text(j, i, f"{grid2_values[i, j]:.1f}",
                       ha="center", va="center", color="black", fontsize=9)

ax.set_title("Heatmap of Surface", fontsize=14, pad=20)
ax.set_xlabel("Width (X-Axis)", fontsize=12)
ax.set_ylabel("Length (Y-Axis)", fontsize=12)

plt.tight_layout()
plt.show()
# หรือใช้ plt.savefig('grid2_heatmap.png') เพื่อบันทึกไฟล์