#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


class RRLegInverseKinematics(Node):
    """
    IK matches chain used by your FK node:

      p = TBH * Ry(align_y) * Rz(q1_used) * Ry(q2_used)Tx(L1) * Ry(q3_used)Tx(L2+L3)

    FK node does:
      q_used = q_measured + offset

    Therefore IK should output:
      q_cmd (measured-domain) = q_used - offset
    """

    def __init__(self):
        super().__init__("rr_leg_inverse_kinematics")

        # ---- topics ----
        self.declare_parameter("target_topic", "/rr_foot_target")
        self.declare_parameter("cmd_topic", "/rr_leg_joint_cmd")

        # ---- joint names ----
        self.declare_parameter("waist_name", "waist")
        self.declare_parameter("hip_name", "hip")
        self.declare_parameter("knee_name", "knee")

        # ✅ joint offsets (deg) — same style as FK
        self.declare_parameter("waist_offset_deg", 0.0)
        self.declare_parameter("hip_offset_deg", 0.0)
        self.declare_parameter("knee_offset_deg", 0.0)

        # ---- hip offset ----
        self.declare_parameter("hip_x", 0.0)
        self.declare_parameter("hip_y", 0.2)
        self.declare_parameter("hip_z", 0.0)

        # ---- link lengths ----
        self.declare_parameter("L1", 0.28)
        self.declare_parameter("L2", 0.30)
        self.declare_parameter("L3", 0.075)

        # ---- elbow mode ----
        self.declare_parameter("elbow", "down")  # "down" or "up"

        # ---- debug ----
        self.declare_parameter("print_deg", True)

        # read params
        self.target_topic = self.get_parameter("target_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value

        self.j_waist = self.get_parameter("waist_name").value
        self.j_hip = self.get_parameter("hip_name").value
        self.j_knee = self.get_parameter("knee_name").value

        self.hx = float(self.get_parameter("hip_x").value)
        self.hy = float(self.get_parameter("hip_y").value)
        self.hz = float(self.get_parameter("hip_z").value)

        self.L1 = float(self.get_parameter("L1").value)
        self.L2 = float(self.get_parameter("L2").value)
        self.L3 = float(self.get_parameter("L3").value)
        self.L23 = self.L2 + self.L3

        self.elbow = str(self.get_parameter("elbow").value).lower()
        self.print_deg = bool(self.get_parameter("print_deg").value)

        # ✅ offsets deg -> rad
        self.waist_off = math.radians(float(self.get_parameter("waist_offset_deg").value))
        self.hip_off = math.radians(float(self.get_parameter("hip_offset_deg").value))
        self.knee_off = math.radians(float(self.get_parameter("knee_offset_deg").value))

        self.sub = self.create_subscription(PointStamped, self.target_topic, self.cb, 10)
        self.pub = self.create_publisher(JointState, self.cmd_topic, 10)

        self.get_logger().info(f"IK target: {self.target_topic} -> cmd: {self.cmd_topic}")
        self.get_logger().info(f"elbow={self.elbow}  L1={self.L1}  L23={self.L23}")
        self.get_logger().info(
            "offset_deg waist=%.3f hip=%.3f knee=%.3f"
            % (math.degrees(self.waist_off), math.degrees(self.hip_off), math.degrees(self.knee_off))
        )

    def solve_ik_used(self, xd, yd, zd):
        """Return q_used (the angles inside the kinematic model)."""
        # --- yaw from YZ ---
        yp = yd - self.hy
        zp = zd - self.hz

        q1 = math.atan2(yp, -zp)
        S = math.sqrt(yp**2 + zp**2)

        # --- planar target ---
        xp = self.hx - xd  # IMPORTANT: hx - xd (matches your FK sign)

        r2 = xp**2 + S**2
        r = math.sqrt(r2)

        # reachability
        if r > (self.L1 + self.L23) or r < abs(self.L1 - self.L23):
            raise ValueError(
                f"out of reach: r={r:.4f} range=[{abs(self.L1-self.L23):.4f},{(self.L1+self.L23):.4f}]"
            )

        c3 = (r2 - self.L1**2 - self.L23**2) / (2.0 * self.L1 * self.L23)
        # c3 = clamp(c3)

        s3_mag = math.sqrt(1.0 - c3 ** 2)
        # s3_mag = math.sqrt(max(0.0, 1.0 - c3 ** 2))
        if self.elbow == "down":
            s3 = -s3_mag
        elif self.elbow == "up":
            s3 = +s3_mag
        else:
            raise ValueError("elbow must be 'down' or 'up'")

        q3 = math.atan2(s3, c3)
        q2 = math.atan2(xp, S) - math.atan2(self.L23 * s3, self.L1 + self.L23 * c3)

        return q1, q2, q3

    def cb(self, msg: PointStamped):
        xd = msg.point.x
        yd = msg.point.y
        zd = msg.point.z

        try:
            # model-domain angles (same convention as FK internal)
            q1_used, q2_used, q3_used = self.solve_ik_used(xd, yd, zd)
        except Exception as e:
            self.get_logger().warn(f"IK failed: {e}")
            return

        # ✅ convert to "measured-domain" command so that FK will do +offset and match q_used
        q1_cmd = q1_used - self.waist_off
        q2_cmd = q2_used - self.hip_off
        q3_cmd = q3_used - self.knee_off

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = [self.j_waist, self.j_hip, self.j_knee]
        out.position = [float(q1_cmd), float(q2_cmd), float(q3_cmd)]
        self.pub.publish(out)

        if self.print_deg:
            self.get_logger().info(
                "IK used(deg): waist=%.2f hip=%.2f knee=%.2f | cmd(deg): waist=%.2f hip=%.2f knee=%.2f"
                % (
                    math.degrees(q1_used), math.degrees(q2_used), math.degrees(q3_used),
                    math.degrees(q1_cmd),  math.degrees(q2_cmd),  math.degrees(q3_cmd),
                )
            )


def main():
    rclpy.init()
    node = RRLegInverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
