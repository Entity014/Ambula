import numpy as np

def fk_pos_rr(q1, q2, l1, l2, degrees=False):
    """
    คืนค่าเฉพาะตำแหน่งปลายเอฟเฟกเตอร์ (x, y) สำหรับแขน 2R บนระนาบ
    q1, q2 : มุมข้อ (เรเดียน; ตั้ง degrees=True ถ้าใส่เป็นองศา)
    l1, l2 : ความยาวลิงก์
    """
    if degrees:
        q1 = np.deg2rad(q1)
        q2 = np.deg2rad(q2)
    th = q1 + q2
    x = l1 * np.cos(q1) + l2 * np.cos(th)
    y = l1 * np.sin(q1) + l2 * np.sin(th)
    return float(x), float(y)

# ตัวอย่างใช้งาน
if __name__ == "__main__":
    l1, l2 = 0.20, 0.25
    q1, q2 = 90, -152  # องศา
    x, y = fk_pos_rr(q1, q2, l1, l2, degrees=True)
    print(f"x={x:.3f} m, y={y:.3f} m")
