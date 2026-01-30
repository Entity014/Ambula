#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster


def matmul3(A, B):
    """3x3 matrix multiply (tuple of tuples)."""
    return (
        (A[0][0]*B[0][0] + A[0][1]*B[1][0] + A[0][2]*B[2][0],
         A[0][0]*B[0][1] + A[0][1]*B[1][1] + A[0][2]*B[2][1],
         A[0][0]*B[0][2] + A[0][1]*B[1][2] + A[0][2]*B[2][2]),
        (A[1][0]*B[0][0] + A[1][1]*B[1][0] + A[1][2]*B[2][0],
         A[1][0]*B[0][1] + A[1][1]*B[1][1] + A[1][2]*B[2][1],
         A[1][0]*B[0][2] + A[1][1]*B[1][2] + A[1][2]*B[2][2]),
        (A[2][0]*B[0][0] + A[2][1]*B[1][0] + A[2][2]*B[2][0],
         A[2][0]*B[0][1] + A[2][1]*B[1][1] + A[2][2]*B[2][1],
         A[2][0]*B[0][2] + A[2][1]*B[1][2] + A[2][2]*B[2][2]),
    )


def matvec3(R, v):
    """3x3 matrix times 3x1 vector."""
    return (
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    )


def Rz(th):
    c, s = math.cos(th), math.sin(th)
    return ((c, -s, 0.0),
            (s,  c, 0.0),
            (0.0, 0.0, 1.0))


def Ry(th):
    c, s = math.cos(th), math.sin(th)
    return (( c, 0.0,  s),
            (0.0, 1.0, 0.0),
            (-s, 0.0,  c))


def quat_from_R(R):
    """
    Convert 3x3 rotation matrix to quaternion (x,y,z,w).
    """
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]

    tr = m00 + m11 + m22
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S

    return (qx, qy, qz, qw)


class RRLegForwardKinematics(Node):
    """
    Matches Sympy chain:

      TBF = TBH * Ry(align_y) * Rz(q1) * Ry(q2) * Tx(L1) * Ry(q3) * Tx(L2+L3)

    where:
      q1 = waist (yaw about Z)
      q2 = hip   (pitch about Y)
      q3 = knee  (pitch about Y)

    Adds joint offsets in degrees:
      q_used = q_measured + radians(offset_deg)
    """

    def __init__(self):
        super().__init__("rr_leg_forward_kinematics")

        # ---- topics / frames ----
        self.declare_parameter("input_topic", "/joint_states/hardware")
        self.declare_parameter("output_topic", "/rr_foot_point")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("rr_foot_frame", "rr_foot")
        self.declare_parameter("publish_tf", True)

        # ---- joint names ----
        self.declare_parameter("waist_name", "waist")
        self.declare_parameter("hip_name", "hip")
        self.declare_parameter("knee_name", "knee")

        # ✅ joint offsets (degrees)
        self.declare_parameter("waist_offset_deg", 0.0)
        self.declare_parameter("hip_offset_deg", 0.0)
        self.declare_parameter("knee_offset_deg", 0.0)

        # ---- hip offset (m) base_link -> hip joint ----
        self.declare_parameter("hip_x", 0.0)
        self.declare_parameter("hip_y", 0.2)
        self.declare_parameter("hip_z", 0.0)

        # ---- link lengths (m) ----
        self.declare_parameter("L1", 0.28)
        self.declare_parameter("L2", 0.30)
        self.declare_parameter("L3", 0.075)

        # ---- align rotation (rad): T_align = Ry(align_y) ----
        self.declare_parameter("align_y", math.pi / 2.0)

        # ---- read params ----
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.rr_foot_frame = self.get_parameter("rr_foot_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.j_waist = self.get_parameter("waist_name").value
        self.j_hip = self.get_parameter("hip_name").value
        self.j_knee = self.get_parameter("knee_name").value

        self.hip_x = float(self.get_parameter("hip_x").value)
        self.hip_y = float(self.get_parameter("hip_y").value)
        self.hip_z = float(self.get_parameter("hip_z").value)

        self.L1 = float(self.get_parameter("L1").value)
        self.L2 = float(self.get_parameter("L2").value)
        self.L3 = float(self.get_parameter("L3").value)

        self.align_y = float(self.get_parameter("align_y").value)

        # offsets deg -> rad
        self.waist_off = math.radians(float(self.get_parameter("waist_offset_deg").value))
        self.hip_off = math.radians(float(self.get_parameter("hip_offset_deg").value))
        self.knee_off = math.radians(float(self.get_parameter("knee_offset_deg").value))

        # ---- ROS IO ----
        self.sub = self.create_subscription(JointState, self.input_topic, self.cb, 10)
        self.pub = self.create_publisher(PointStamped, self.output_topic, 10)
        self.tf_br = TransformBroadcaster(self)

        self.get_logger().info(f"FK RR leg listening: {self.input_topic}")
        self.get_logger().info(f"Publishing point: {self.output_topic}")
        self.get_logger().info(f"align_y(rad)={self.align_y}")
        self.get_logger().info(
            "offset_deg waist=%.3f hip=%.3f knee=%.3f"
            % (math.degrees(self.waist_off), math.degrees(self.hip_off), math.degrees(self.knee_off))
        )

    def cb(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        def get_pos(jn: str) -> float:
            i = name_to_idx.get(jn, None)
            if i is None:
                return 0.0
            return msg.position[i] if i < len(msg.position) else 0.0

        # measured
        q1 = get_pos(self.j_waist)
        q2 = get_pos(self.j_hip)
        q3 = get_pos(self.j_knee)

        # apply offsets (deg->rad already)
        q1 += self.waist_off
        q2 += self.hip_off
        q3 += self.knee_off

        # rotation: R = Ry(align) * Rz(q1) * Ry(q2) * Ry(q3)
        R_align = Ry(self.align_y)
        R = matmul3(R_align, Rz(q1))
        R = matmul3(R, Ry(q2))
        R = matmul3(R, Ry(q3))

        # translation:
        # v_plane = Ry(q2)*[L1,0,0] + Ry(q2)*Ry(q3)*[(L2+L3),0,0]
        v1 = matvec3(Ry(q2), (self.L1, 0.0, 0.0))
        v2 = matvec3(matmul3(Ry(q2), Ry(q3)), (self.L2 + self.L3, 0.0, 0.0))
        v_plane = (v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2])

        R_pre = matmul3(R_align, Rz(q1))
        v_base = matvec3(R_pre, v_plane)

        x = self.hip_x + v_base[0]
        y = self.hip_y + v_base[1]
        z = self.hip_z + v_base[2]

        # publish point
        pt = PointStamped()
        pt.header = msg.header
        pt.header.frame_id = self.base_frame
        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = float(z)
        self.pub.publish(pt)

        # publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header = pt.header
            t.child_frame_id = self.rr_foot_frame

            t.transform.translation.x = pt.point.x
            t.transform.translation.y = pt.point.y
            t.transform.translation.z = pt.point.z

            qx, qy, qz, qw = quat_from_R(R)
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.tf_br.sendTransform(t)


def main():
    rclpy.init()
    node = RRLegForwardKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
