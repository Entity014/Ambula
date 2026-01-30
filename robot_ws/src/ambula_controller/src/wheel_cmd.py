#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class CmdVelToWheel(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_wheel')

        # ---------------- Parameters ----------------
        self.declare_parameter('wheel_radius', 0.075)        # m
        self.declare_parameter('wheel_separation', 0.40)    # m
        self.declare_parameter('output_unit', 'rad_s')      # 'rad_s' or 'm_s'

        # topic parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('left_wheel_topic',
                               '/ambula/left_wheel_velocity_cmd')
        self.declare_parameter('right_wheel_topic',
                               '/ambula/right_wheel_velocity_cmd')

        # get parameters
        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value
        self.unit = self.get_parameter('output_unit').value

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.left_topic = self.get_parameter('left_wheel_topic').value
        self.right_topic = self.get_parameter('right_wheel_topic').value

        # ---------------- Publishers ----------------
        self.pub_left = self.create_publisher(
            Float64, self.left_topic, 10
        )
        self.pub_right = self.create_publisher(
            Float64, self.right_topic, 10
        )

        # ---------------- Subscriber ----------------
        self.sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10
        )

        self.get_logger().info(
            f'CmdVelToWheel started\n'
            f'  cmd_vel_topic   : {self.cmd_vel_topic}\n'
            f'  left_wheel_topic: {self.left_topic}\n'
            f'  right_wheel_topic: {self.right_topic}'
        )

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive kinematics
        v_left = v - (w * self.L / 2.0)
        v_right = v + (w * self.L / 2.0)

        # Convert to wheel angular velocity if needed
        if self.unit == 'rad_s':
            v_left /= self.r
            v_right /= self.r

        self.pub_left.publish(Float64(data=float(v_left)))
        self.pub_right.publish(Float64(data=float(v_right)))


def main():
    rclpy.init()
    node = CmdVelToWheel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
