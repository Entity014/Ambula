from dataclasses import dataclass
import math
from .utils import clip, wrap_pi, sgn
from .frames import side_sign_y

@dataclass
class Link3:
    L1: float; L2: float; L3: float  # [m]

@dataclass
class JointAngles:
    q1: float; q2: float; q3: float  # [rad]

@dataclass
class FootPose:
    x: float; y: float; z: float     # [m]

def fk_leg(q: JointAngles, link: Link3, side: str) -> FootPose:
    """FK ขา 3-DOF (waist, hip, knee), mirror แกน Y ตามข้างให้สอดคล้อง REP-103"""
    s1, c1 = math.sin(q.q1), math.cos(q.q1)
    s2, c2 = math.sin(q.q2), math.cos(q.q2)
    s23, c23 = math.sin(q.q2 + q.q3), math.cos(q.q2 + q.q3)

    x = link.L2 * s2 + link.L3 * s23
    y = -link.L1 * c1 + link.L2 * s1 * c2 + link.L3 * s1 * c23
    z = -link.L1 * s1 - link.L2 * c1 * c2 - link.L3 * c1 * c23

    # ทำให้ Y เป็นบวกฝั่งซ้าย/ลบฝั่งขวา (mirror ที่ผลลัพธ์)
    y_eff = side_sign_y(side) * y
    return FootPose(x, y_eff, z)

def ik_leg(p: FootPose, link: Link3, side: str) -> JointAngles | None:
    """IK ขา 3-DOF: สูตรเดียวกับสคริปต์ SymPy
       q1 = π + (atan2(Z, Y) - atan2(d, L1))  โดย Y ถูก canonicalize ก่อน
       q3_expr ใช้ (π + acos) ส่วน elbow ใช้ q3_temp = (π - acos) ตามสคริปต์
       q2 = (atan2(Z, X) + π/2) + atan2(L3*sin(q3_temp), L2 + L3*cos(q3_temp))
    """
    # -------- 1) canonicalize ให้เหมือนฝั่งซ้าย แล้วค่อย map กลับตอนท้าย --------
    y_canon = side_sign_y(side) * p.y

    # -------- 2) q1 ตาม SymPy: q1 = π + (atan2(Z, Y) - atan2(d, L1)) --------
    r_yz = math.hypot(y_canon, p.z)
    if r_yz < link.L1 - 1e-9:
        return None  # เชิงเรขาคณิตเอื้อมไม่ถึง (ไม่มีวิธีหมุน q1 ให้ L1 ไปแตะระนาบ YZ ได้)
    d = math.sqrt(max(0.0, r_yz * r_yz - link.L1 * link.L1))
    q1_raw = math.pi + (math.atan2(p.z, y_canon) - math.atan2(d, link.L1))
    q1 = side_sign_y(side) * q1_raw  # map กลับตามข้าง

    # -------- 3) q3: แยก q3_expr (ผลลัพธ์สุดท้าย) และ q3_temp (สำหรับ elbow term) --------
    r_xyz = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
    c3_arg = (link.L1**2 + link.L2**2 + link.L3**2 - r_xyz**2) / (2.0 * link.L2 * link.L3)
    c3_arg = clip(c3_arg, -1.0, 1.0)  # หนีบโดเมนให้ปลอดภัยเหมือน acos_clip

    # ตามสคริปต์:
    # q3_expr = -sign(X) * (π + acos(...))
    # q3_temp = -sign(X) * (π - acos(...))  # ใช้กับ elbow term เท่านั้น
    sx = sgn(p.x)  # ควรคืนค่า 1, 0, หรือ -1 (ถ้า x=0 → 0 เหมือน sp.sign)
    q3_temp = -sx * (math.pi - math.acos(c3_arg))
    q3 = -sx * (math.pi + math.acos(c3_arg))

    # -------- 4) elbow term ใช้ q3_temp (อิงกิ่งเดียวกับ SymPy เสมอ) --------
    elbow = math.atan2(link.L3 * math.sin(q3_temp), link.L2 + link.L3 * math.cos(q3_temp))

    # -------- 5) q2: (atan2(Z, X) + π/2) + elbow  เหมือน SymPy --------
    q2 = (math.atan2(p.z, p.x) + 0.5 * math.pi) + elbow

    # -------- 6) wrap ให้อยู่ในช่วง [-π, π) --------
    return JointAngles(wrap_pi(q1), wrap_pi(q2), wrap_pi(q3))


