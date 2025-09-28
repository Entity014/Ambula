#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

# ---------------------------
# 1) Closed-form from formula
# ---------------------------
def p_closed_form(q1, q2, q3, L1, L2, L3):
    s2  = np.sin(q2);    c2  = np.cos(q2)
    s23 = np.sin(q2+q3); c23 = np.cos(q2+q3)
    s1  = np.sin(q1);    c1  = np.cos(q1)

    x = - L1*s2 - L2*s23 - L3*s23
    R =  (L1*c2 + L2*c23 + L3*c23)        # ระยะฉายลงระนาบ y–z หลังหมุนเอว
    y =  R * s1
    z = -R * c1
    return np.array([x, y, z])


# --------------------------------------
# 2) FK via transform chain (ground truth)
#    Frames: 0-(WaistX)-(HipY)-(KneeY)-Foot
#    Link offsets: Tz(-L1), Tz(-L2), Tz(-L3)
# --------------------------------------
def Rx(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]], dtype=float)

def Ry(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]], dtype=float)

def Tz(d):
    return np.array([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]], dtype=float)

def fk_transform(q1, q2, q3, L1, L2, L3):
    T01 = Rx(q1)
    T12 = Ry(q2) @ Tz(-L1)
    T23 = Ry(q3) @ Tz(-L2)
    T3F = Tz(-L3)
    T0F = (((T01 @ T12) @ T23) @ T3F)
    p = T0F[:3, 3]
    return p, T0F


# ---------------------------
# 3) Randomized unit testing
# ---------------------------
def run_random_tests(n=1000, tol=1e-9):
    rng = np.random.default_rng(0)
    # ตั้งค่าความยาวลิงก์ตามจริงได้เลย
    L1, L2, L3 = 0.08, 0.18, 0.18

    max_err = 0.0
    worst = None

    for _ in range(n):
        q1 = rng.uniform(-np.pi, np.pi)        # Waist (X)
        q2 = rng.uniform(-np.pi/2, np.pi/2)    # Hip (Y)
        q3 = rng.uniform(-np.pi/2, np.pi/2)    # Knee (Y)

        p_cf = p_closed_form(q1, q2, q3, L1, L2, L3)
        p_tf, _ = fk_transform(q1, q2, q3, L1, L2, L3)

        err = np.linalg.norm(p_cf - p_tf)
        if err > max_err:
            max_err = err
            worst = (q1, q2, q3, p_cf, p_tf)

    print(f"✅ Completed {n} tests. Max |error| = {max_err:.3e}")
    if max_err > tol:
        print("⚠️  Error exceeds tolerance! Show worst case:")
        q1, q2, q3, p_cf, p_tf = worst
        print(f"q1,q2,q3 = {q1:.3f}, {q2:.3f}, {q3:.3f}")
        print("p_closed_form =", p_cf)
        print("p_transform   =", p_tf)
    return max_err, worst


# ---------------------------
# 4) Quick visualization
# ---------------------------
def demo_plot():
    L1, L2, L3 = 0.08, 0.18, 0.18
    q1, q2, q3 = np.deg2rad([0.0, 0.0, 0.0])

    # จุดจากสองวิธี
    p_cf = p_closed_form(q1, q2, q3, L1, L2, L3)
    p_tf, T0F = fk_transform(q1, q2, q3, L1, L2, L3)

    # วาดแกนและตำแหน่งปลายเท้าแบบง่ายๆ มุมมอง 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter([0], [0], [0], s=40, label='Base')
    ax.scatter([p_cf[0]], [p_cf[1]], [p_cf[2]], s=40, marker='x', label='Closed-form')
    ax.scatter([p_tf[0]], [p_tf[1]], [p_tf[2]], s=40, marker='o', facecolors='none', edgecolors='k', label='Transform')

    # สเกลแกน
    r = L1 + L2 + L3
    for a in [ax.set_xlim, ax.set_ylim, ax.set_zlim]:
        a(-r, r)

    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_zlabel('z [m]')
    ax.set_title('FK test: closed-form vs transform')
    ax.legend()
    plt.show()


if __name__ == "__main__":
    run_random_tests(n=1000)
    demo_plot()
