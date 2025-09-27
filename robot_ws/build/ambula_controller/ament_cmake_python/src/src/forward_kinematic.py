#!/usr/bin/env python3
# ROS 2 node: /joint_states -> /fk/foot_pose (PoseStamped) + /fk/foot_rpy (Vector3Stamped)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from tf_transformations import quaternion_from_euler  # <-- ใช้ไลบรารี

# อยู่แพ็กเกจเดียวกัน ใช้ relative import
from ambula_controller import fk_leg_planar3r_symbolic as sym

class FKLegNode(Node):
    def __init__(self):
        super().__init__('fk_leg_node')

        # ---- parameters ----
        self.declare_parameter('joint_names', ['waist_yaw', 'hip_pitch', 'knee_pitch'])
        self.declare_parameter('links', list(sym.DEFAULT_LINKS))  # [L1, L2, L3] (m)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('foot_frame', 'foot_link')

        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        links_param = self.get_parameter('links').get_parameter_value().double_array_value
        self.links = tuple(links_param) if links_param else sym.DEFAULT_LINKS
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.foot_frame = self.get_parameter('foot_frame').get_parameter_value().string_value

        # ---- I/O ----
        self.sub = self.create_subscription(JointState, '/joint_states', self.cb_joint, 50)
        self.pub_pose = self.create_publisher(PoseStamped, '/fk/foot_pose', 50)
        self.pub_rpy  = self.create_publisher(Vector3Stamped, '/fk/foot_rpy', 50)

        self.get_logger().info(f'[{self.get_name()}] joint_names={self.joint_names}, links={self.links}')

    def cb_joint(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            q1 = float(name_to_pos[self.joint_names[0]])
            q2 = float(name_to_pos[self.joint_names[1]])
            q3 = float(name_to_pos[self.joint_names[2]])
        except Exception as e:
            self.get_logger().throttle_logger_error(2000, f'JointState missing fields: {e}')
            return

        L1, L2, L3 = self.links
        x, y, z, roll, pitch, yaw = sym.compute_fk(q1, q2, q3, sym.DEFAULT_LINKS)

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp if (msg.header.stamp.sec or msg.header.stamp.nanosec) else self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        # ใช้ tf_transformations
        qx, qy, qz, qw = quaternion_from_euler(float(roll), float(pitch), float(yaw))  # RPY(rad) -> quat
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pub_pose.publish(pose)

        rpy = Vector3Stamped()
        rpy.header = pose.header
        rpy.vector.x = float(roll)
        rpy.vector.y = float(pitch)
        rpy.vector.z = float(yaw)
        self.pub_rpy.publish(rpy)

def main():
    rclpy.init()
    node = FKLegNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
