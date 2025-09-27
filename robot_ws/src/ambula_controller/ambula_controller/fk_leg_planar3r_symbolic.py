#!/usr/bin/env python3
# fk_leg_3d_symbolic.py
# -----------------------------------------------
# 3-DoF leg FK (3D): Waist-X, Hip-Y, Knee-Y
# → [x,y,z,roll,pitch,yaw] + Jacobian
# -----------------------------------------------

import sympy as sp
from sympy.utilities.lambdify import lambdify

# ---------- symbols ----------
q1, q2, q3 = sp.symbols('q1 q2 q3', real=True)                    # joint angles [rad]
L1, L2, L3 = sp.symbols('L1 L2 L3', positive=True, real=True)     # link lengths [m]

# ---------- rotation & translation ----------
def Rx(th):
    return sp.Matrix([
        [1,0,0,0],
        [0,sp.cos(th),-sp.sin(th),0],
        [0,sp.sin(th), sp.cos(th),0],
        [0,0,0,1]
    ])
def Ry(th):
    return sp.Matrix([
        [ sp.cos(th),0,sp.sin(th),0],
        [ 0,1,0,0],
        [-sp.sin(th),0,sp.cos(th),0],
        [0,0,0,1]
    ])
def Tz(d):
    return sp.Matrix([
        [1,0,0,0],
        [0,1,0,0],
        [0,0,1,d],
        [0,0,0,1]
    ])

# ---------- kinematic chain ----------
# base @ hip
T01 = Rx(q1)                 # waist ab/ad
T12 = Ry(q2) * Tz(-L1)       # hip pitch + translate thigh
T23 = Ry(q3) * Tz(-L2)       # knee pitch + translate shank
T3F = Tz(-L3)                # foot length

T0F = sp.simplify(T01 * T12 * T23 * T3F)

# ---------- position ----------
x, y, z = T0F[0,3], T0F[1,3], T0F[2,3]

# ---------- orientation (ZYX Euler) ----------
R = T0F[:3, :3]
roll  = sp.atan2(R[2,1], R[2,2])
pitch = sp.atan2(-R[2,0], sp.sqrt(R[2,1]**2 + R[2,2]**2))
yaw   = sp.atan2(R[1,0], R[0,0])

# ---------- pack & Jacobian ----------
pose_expr = sp.Matrix([x, y, z, roll, pitch, yaw])
J_expr    = pose_expr.jacobian(sp.Matrix([q1, q2, q3]))

# ---------- lambdify ----------
fk_pose = lambdify((q1, q2, q3, L1, L2, L3), pose_expr, modules="numpy")
fk_T0F  = lambdify((q1, q2, q3, L1, L2, L3), T0F.tolist(), modules="numpy")
fk_J    = lambdify((q1, q2, q3, L1, L2, L3), J_expr, modules="numpy")

# ---------- defaults ----------
DEFAULT_LINKS = (0.20, 0.20, 0.10)  # [m]

# ---------- helper functions ----------
def compute_fk(q1_val, q2_val, q3_val, links=DEFAULT_LINKS):
    """Forward Kinematics -> (x,y,z,roll,pitch,yaw)"""
    L1, L2, L3 = links
    result = fk_pose(q1_val, q2_val, q3_val, L1, L2, L3)
    return tuple(float(v) for v in result)

def compute_T0F(q1_val, q2_val, q3_val, links=DEFAULT_LINKS):
    """Homogeneous transform (4x4 list of lists)"""
    L1, L2, L3 = links
    mat = fk_T0F(q1_val, q2_val, q3_val, L1, L2, L3)
    return [[float(x) for x in row] for row in mat]

def compute_J(q1_val, q2_val, q3_val, links=DEFAULT_LINKS):
    """Jacobian (6x3 list of lists)"""
    L1, L2, L3 = links
    mat = fk_J(q1_val, q2_val, q3_val, L1, L2, L3)
    return [[float(x) for x in row] for row in mat]

# ---------- exports ----------
__all__ = [
    'q1','q2','q3','L1','L2','L3',
    'T0F','pose_expr','J_expr',
    'fk_pose','fk_T0F','fk_J',
    'DEFAULT_LINKS',
    'compute_fk','compute_T0F','compute_J'
]

# ---------- demo ----------
if __name__ == "__main__":
    q_sample = (0.1, -0.3, 0.2)  # rad
    pose = compute_fk(*q_sample)
    print("Pose [x,y,z,roll,pitch,yaw] =", pose)

    T = compute_T0F(*q_sample)
    print("T0F =")
    for row in T:
        print(row)

    J = compute_J(*q_sample)
    print("Jacobian =")
    for row in J:
        print(row)
