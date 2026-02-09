#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

def wrap_pi(a: float) -> float:
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a

def yaw_from_quat(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class GoToGoalController(Node):
    def __init__(self):
        super().__init__('go_to_goal_controller')

        # ===== gains / limits =====
        self.declare_parameter('k_rho',   0.8)
        self.declare_parameter('k_alpha', 2.0)
        self.declare_parameter('k_beta', -0.5)

        self.declare_parameter('v_max', 0.6)
        self.declare_parameter('w_max', 2.0)
        self.declare_parameter('goal_tol_xy', 0.03)
        self.declare_parameter('goal_tol_yaw', 0.05)
        self.declare_parameter('control_rate_hz', 50.0)

        # ===== default goal =====
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)

        # ===== topics / frames (from config) =====
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        # ===== state =====
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_yaw = float(self.get_parameter('goal_yaw').value)

        # ===== QoS =====
        qos_odom = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ===== ROS interfaces =====
        self.sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self.odom_cb, qos_odom
        )

        self.sub_goal = self.create_subscription(
            PoseStamped, self.goal_topic, self.goal_cb, 10
        )

        self.pub_cmd = self.create_publisher(
            Twist, self.cmd_vel_topic, 10
        )

        rate = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            f"GoToGoalController started.\n"
            f"  odom_topic: {self.odom_topic}\n"
            f"  goal_topic: {self.goal_topic}\n"
            f"  cmd_vel_topic: {self.cmd_vel_topic}\n"
            f"  frames: {self.odom_frame} -> {self.base_frame}"
        )

    def odom_cb(self, msg: Odometry):
        # (optional) check frame if you want strictness
        # if msg.header.frame_id and msg.header.frame_id != self.odom_frame:
        #     return

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = float(p.x)
        self.y = float(p.y)
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_odom = True

    def goal_cb(self, msg: PoseStamped):
        self.goal_x = float(msg.pose.position.x)
        self.goal_y = float(msg.pose.position.y)
        q = msg.pose.orientation
        self.goal_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        self.get_logger().info(
            f"New goal: x={self.goal_x:.3f}, y={self.goal_y:.3f}, yaw={self.goal_yaw:.3f}"
        )

    def control_loop(self):
        if not self.have_odom:
            return

        k_rho   = float(self.get_parameter('k_rho').value)
        k_alpha = float(self.get_parameter('k_alpha').value)
        k_beta  = float(self.get_parameter('k_beta').value)

        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)
        tol_xy = float(self.get_parameter('goal_tol_xy').value)
        tol_yaw = float(self.get_parameter('goal_tol_yaw').value)

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        rho = math.hypot(dx, dy)

        yaw_err = wrap_pi(self.goal_yaw - self.yaw)
        # if rho < tol_xy and abs(yaw_err) < tol_yaw:
        #     self.publish_cmd(0.0, 0.0)
        #     return

        goal_heading = math.atan2(dy, dx)
        alpha = wrap_pi(goal_heading - self.yaw)
        beta = wrap_pi(self.goal_yaw - goal_heading)

        v = k_rho * rho
        w = k_alpha * alpha + k_beta * beta

        if abs(alpha) > (math.pi / 2.0):
            v = -v

        v = max(-v_max, min(v_max, v))
        w = max(-w_max, min(w_max, w))

        self.publish_cmd(v, w)

    def publish_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

def main():
    rclpy.init()
    node = GoToGoalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
