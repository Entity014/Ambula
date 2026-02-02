#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointVisualizer(Node):
    """
    Subscribe:  /joint_states/converted  (sensor_msgs/JointState)
    Publish:    /joint_states            (sensor_msgs/JointState)

    - Reorder joints to a fixed order
    - Rename joints according to a mapping
    - Fill missing joints with 0.0
    - Keep header.stamp fresh (or pass-through if you prefer)
    """

    def __init__(self):
        super().__init__("joint_visualizer")

        # --------- parameters (ปรับง่าย) ----------
        self.declare_parameter("in_topic", "/joint_states/converted")
        self.declare_parameter("out_topic", "/joint_states")

        # ชื่อเป้าหมายที่ต้องการ (ตามที่คุณให้มา)
        self.target_names = [
            "left_waist", "left_hip", "left_knee", "left_wheel",
            "right_waist", "right_hip", "right_knee", "right_wheel"
        ]

        # mapping: ชื่อที่เข้ามา -> ชื่อที่อยากให้เป็น (ถ้าชื่อเข้ามาตรงอยู่แล้วก็ไม่ต้องใส่)
        # ปรับตรงนี้ให้เข้ากับชื่อ joint ใน /joint_states/converted ของคุณ
        self.name_map = {
            # ตัวอย่าง (สมมติ input เป็น *_joint):
            "left_waist_joint": "left_waist",
            "left_hip_joint": "left_hip",
            "left_knee_joint": "left_knee",
            "left_wheel_joint": "left_wheel",
            "right_waist_joint": "right_waist",
            "right_hip_joint": "right_hip",
            "right_knee_joint": "right_knee",
            "right_wheel_joint": "right_wheel",
        }

        in_topic = self.get_parameter("in_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("out_topic").get_parameter_value().string_value

        self.pub = self.create_publisher(JointState, out_topic, 10)
        self.sub = self.create_subscription(JointState, in_topic, self.cb, 10)

        self.get_logger().info(f"Listening: {in_topic}  -> Publishing: {out_topic}")
        self.get_logger().info(f"Target joints: {self.target_names}")

    def cb(self, msg_in: JointState):
        # 1) สร้าง dict: mapped_name -> index ของ input
        idx_by_name = {}
        for i, n in enumerate(msg_in.name):
            mapped = self.name_map.get(n, n)  # ถ้าไม่มี mapping ก็ใช้ชื่อเดิม
            idx_by_name[mapped] = i

        # helpers: ดึงค่าจาก array แบบกันกรณี array ว่าง/สั้น
        def get_arr(arr, i, default=0.0):
            try:
                return float(arr[i])
            except Exception:
                return float(default)

        # 2) สร้าง message output เรียงตาม target_names
        msg_out = JointState()
        # ใช้ stamp ปัจจุบัน (ปลอดภัยกับ visualization)
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = msg_in.header.frame_id  # จะให้ว่างก็ได้

        msg_out.name = list(self.target_names)

        positions = []
        velocities = []
        efforts = []

        for tn in self.target_names:
            if tn in idx_by_name:
                i = idx_by_name[tn]
                positions.append(get_arr(msg_in.position, i, 0.0))
                velocities.append(get_arr(msg_in.velocity, i, 0.0))
                efforts.append(get_arr(msg_in.effort, i, 0.0))
            else:
                # joint หาย → ใส่ 0
                positions.append(0.0)
                velocities.append(0.0)
                efforts.append(0.0)

        # ตาม spec JointState: position/velocity/effort สามารถใส่หรือปล่อยว่างได้
        # แต่เพื่อให้เข้ากับ tooling ส่วนใหญ่ (RViz/foxglove) ใส่ให้ครบง่ายสุด
        msg_out.position = positions
        msg_out.velocity = velocities
        msg_out.effort = efforts

        self.pub.publish(msg_out)


def main():
    rclpy.init()
    node = JointVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
