#!/usr/bin/env python3
# fk_leg_plot.py  — visualize 3-DoF leg FK with sliders (Matplotlib)
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# ========== FK core (DH เช่นเดียวกับที่ใช้ใน fk_leg_3d_symbolic.py) ==========
def DH(a, alpha, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ ct, -st*ca,  st*sa, a*ct],
        [ st,  ct*ca, -ct*sa, a*st],
        [  0,     sa,     ca,    d],
        [  0,      0,      0,    1],
    ], dtype=float)

def fk_chain(q1, q2, q3, L1, L2, L3):
    """
    q1: waist ab/ad (หมุนรอบแกน x)
    q2: hip pitch (หมุนรอบแกน y)
    q3: knee pitch (หมุนรอบแกน y)
    L1: ความยาวต้นขา
    L2: ความยาวหน้าแข้ง
    L3: ความยาวเท้า
    """
    # ใช้ rotation matrix ตรง ๆ
    Rx = lambda th: np.array([
        [1,0,0,0],
        [0,np.cos(th),-np.sin(th),0],
        [0,np.sin(th), np.cos(th),0],
        [0,0,0,1]
    ])
    Ry = lambda th: np.array([
        [ np.cos(th),0,np.sin(th),0],
        [ 0,1,0,0],
        [-np.sin(th),0,np.cos(th),0],
        [0,0,0,1]
    ])
    Tz = lambda d: np.array([
        [1,0,0,0],
        [0,1,0,0],
        [0,0,1,d],
        [0,0,0,1]
    ])

    # เริ่มจาก base ที่สะโพก
    T0 = np.eye(4)
    # waist: หมุนรอบ x
    T1 = Rx(q1)
    # hip: หมุนรอบ y แล้ว translate ลง -L1
    T2 = T1 @ Ry(q2) @ Tz(-L1)
    # knee: หมุนรอบ y แล้ว translate ลง -L2
    T3 = T2 @ Ry(q3) @ Tz(-L2)
    # foot: translate ลง -L3
    TF = T3 @ Tz(-L3)

    return [T0, T1, T2, T3, TF]


def pose_from_T(T):
    R = T[:3,:3]; p = T[:3,3]
    roll  = np.arctan2(R[2,1], R[2,2])
    pitch = np.arctan2(-R[2,0], np.sqrt(R[2,1]**2 + R[2,2]**2))
    yaw   = np.arctan2(R[1,0], R[0,0])
    return float(p[0]), float(p[1]), float(p[2]), float(roll), float(pitch), float(yaw)

# ========== defaults ==========
q1, q2, q3 = 0.0, 0.0, 0.0   # rad
L1, L2, L3 = 0.20, 0.20, 0.10    # m

# ========== figure layout ==========
plt.rcParams["figure.figsize"] = (10, 6)
fig = plt.figure("3-DoF Leg FK (Matplotlib)", constrained_layout=True)
gs  = fig.add_gridspec(2, 2, height_ratios=[3,1])

ax3d = fig.add_subplot(gs[0,0], projection='3d')
ax2d = fig.add_subplot(gs[0,1])
axsl = fig.add_subplot(gs[1,:]); axsl.axis("off")

# สไลเดอร์
ax_q1 = plt.axes([0.10, 0.12, 0.35, 0.03]); s_q1 = Slider(ax_q1, "q1 (rad)", -np.pi, np.pi, valinit=q1)
ax_q2 = plt.axes([0.10, 0.08, 0.35, 0.03]); s_q2 = Slider(ax_q2, "q2 (rad)", -np.pi, np.pi, valinit=q2)
ax_q3 = plt.axes([0.10, 0.04, 0.35, 0.03]); s_q3 = Slider(ax_q3, "q3 (rad)", -np.pi, np.pi, valinit=q3)

ax_L1 = plt.axes([0.55, 0.12, 0.35, 0.03]); s_L1 = Slider(ax_L1, "L1 (m)", 0.05, 0.50, valinit=L1)
ax_L2 = plt.axes([0.55, 0.08, 0.35, 0.03]); s_L2 = Slider(ax_L2, "L2 (m)", 0.05, 0.50, valinit=L2)
ax_L3 = plt.axes([0.55, 0.04, 0.35, 0.03]); s_L3 = Slider(ax_L3, "L3 (m)", 0.05, 0.50, valinit=L3)

# primitive handles (จะอัปเดตค่าทุกครั้งที่ขยับสไลเดอร์)
line3d, = ax3d.plot([], [], [], marker='o')
line2d, = ax2d.plot([], [], marker='o')
txt_pose = axsl.text(0.01, 0.6, "", fontsize=11, transform=axsl.transAxes)

def update(_=None):
    q1,q2,q3 = s_q1.val, s_q2.val, s_q3.val
    L1,L2,L3 = s_L1.val, s_L2.val, s_L3.val

    Ts = fk_chain(q1,q2,q3,L1,L2,L3)
    P  = np.stack([T[:3,3] for T in Ts])  # (5,3): base, j1, j2, j3, foot
    x,y,z = P[:,0], P[:,1], P[:,2]

    # 3D
    ax3d.cla()
    ax3d.plot(x, y, z, marker='o')
    ax3d.set_xlabel("X (m)"); ax3d.set_ylabel("Y (m)"); ax3d.set_zlabel("Z (m)")
    ax3d.set_title("3-DoF Leg (3D view)")
    # กำหนดสเกลให้สมมาตร
    mins = P.min(axis=0); maxs = P.max(axis=0); ctr = (mins+maxs)/2.0
    rng  = (maxs-mins).max(); rng = max(rng, 0.2)
    ax3d.set_xlim(ctr[0]-rng/2, ctr[0]+rng/2)
    ax3d.set_ylim(ctr[1]-rng/2, ctr[1]+rng/2)
    ax3d.set_zlim(ctr[2]-rng/2, ctr[2]+rng/2)

    # 2D x–z
    ax2d.cla()
    ax2d.plot(x, z, marker='o')
    for label, (px,pz) in zip(['base','j1','j2','j3','foot'], P[:,[0,2]]):
        ax2d.text(px, pz, label)
    ax2d.set_aspect('equal', adjustable='box')
    ax2d.set_xlabel("X (m)"); ax2d.set_ylabel("Z (m)")
    ax2d.set_title("x–z projection")

    # pose ของปลายเท้า
    xF,yF,zF, roll,pitch,yaw = pose_from_T(Ts[-1])
    txt = (f"Foot pose:\n"
           f"x={xF:+.3f} m, y={yF:+.3f} m, z={zF:+.3f} m\n"
           f"roll={roll:+.3f} rad, pitch={pitch:+.3f} rad, yaw={yaw:+.3f} rad")
    txt_pose.set_text(txt)

    fig.canvas.draw_idle()

# ติด callback ให้สไลเดอร์
for s in [s_q1,s_q2,s_q3,s_L1,s_L2,s_L3]:
    s.on_changed(update)

update()
plt.show()
