#!/usr/bin/env python3
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped

from ambula_controller.kinematics import fk_leg, Link3, JointAngles

class LegFKNode(Node):
    def __init__(self):
        super().__init__('leg_fk_node')

        # ---- parameters ----
        self.declare_parameter(
            'legs', [
                'left: left_waist,left_hip,left_knee',
                'right: right_waist,right_hip,right_knee',
            ]
        )
        self.declare_parameter('L1', 0.075)
        self.declare_parameter('L2', 0.200)
        self.declare_parameter('L3', 0.240)
        self.declare_parameter('input_topic', '/joint_states/hw')
        self.declare_parameter('frame_id', 'base_link')

        self._update_params_from_server()
        self.add_on_set_parameters_callback(self._on_param_change)

        qos = rclpy.qos.qos_profile_sensor_data
        self.sub = self.create_subscription(JointState, self.input_topic, self.joint_cb, qos)

        self.pubs: Dict[str, any] = {
            leg_name: self.create_publisher(PointStamped, f"/{leg_name}_foot_pose", 10)
            for leg_name in self.legs.keys()
        }

        # กันสแปมข้อความเตือน
        self._last_missing: Dict[str, Tuple[str, ...]] = {}

        # ลิงก์ไว้ใช้กับ fk_leg()
        self.link = Link3(self.L1, self.L2, self.L3)

        self.get_logger().info(f"Legs: {list(self.legs.keys())}")

    def _update_params_from_server(self):
        legs_param: List[str] = self.get_parameter('legs').get_parameter_value().string_array_value
        self.legs: Dict[str, List[str]] = {}
        for entry in legs_param:
            leg_name, joints_str = entry.split(':')
            joints = [j.strip() for j in joints_str.split(',')]
            self.legs[leg_name.strip()] = joints

        self.L1 = float(self.get_parameter('L1').value)
        self.L2 = float(self.get_parameter('L2').value)
        self.L3 = float(self.get_parameter('L3').value)
        self.input_topic = self.get_parameter('input_topic').value
        self.frame_id = self.get_parameter('frame_id').value

    def _on_param_change(self, params):
        changed = False
        for p in params:
            if p.name in ('L1','L2','L3') and p.type_ == p.TYPE_DOUBLE:
                setattr(self, p.name, p.value)
                changed = True
        if changed:
            self.link = Link3(self.L1, self.L2, self.L3)
        return SetParametersResult(successful=True)

    # ----- helpers -----
    @staticmethod
    def _aliases(name: str) -> List[str]:
        # ทำ alias สองทาง: left_* <-> leg_*
        al = {name}
        if name.startswith('left_'):
            al.add(name.replace('left_', 'leg_', 1))
        if name.startswith('leg_'):
            al.add(name.replace('leg_', 'left_', 1))
        return list(al)

    def _get_joint_positions(self, leg_name: str, joints: List[str], name_to_pos: Dict[str, float]) -> Optional[Tuple[float,float,float]]:
        found = []
        missing = []
        for j in joints:
            val = None
            # 1) ตรงตัว
            if j in name_to_pos:
                val = name_to_pos[j]
            else:
                # 2) alias
                for a in self._aliases(j):
                    if a in name_to_pos:
                        val = name_to_pos[a]
                        break
                # 3) suffix (_waist/_hip/_knee)
                if val is None and '_' in j:
                    suffix = j.split('_', 1)[1]
                    candidates = [k for k in name_to_pos if k.endswith(suffix)]
                    if candidates:
                        val = name_to_pos[candidates[0]]
            if val is None:
                missing.append(j)
            else:
                found.append(float(val))

        if missing:
            tup = tuple(missing)
            if self._last_missing.get(leg_name) != tup:
                self.get_logger().warning(f"[{leg_name}] Waiting Joint: {missing}")
                self._last_missing[leg_name] = tup
            return None

        if leg_name in self._last_missing:
            del self._last_missing[leg_name]
        return tuple(found)  # q1,q2,q3

    # ----- callback -----
    def joint_cb(self, msg: JointState):
        name_to_pos: Dict[str, float] = {n: p for n, p in zip(msg.name, msg.position)}

        for leg_name, joints in self.legs.items():
            q = self._get_joint_positions(leg_name, joints, name_to_pos)
            if q is None:
                continue
            q1, q2, q3 = q

            # เรียกใช้ fk_leg() จากไลบรารี
            side = "left" if "left" in leg_name else "right"
            p = fk_leg(JointAngles(q1, q2, q3), self.link, side)

            out = PointStamped()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.frame_id
            out.point.x, out.point.y, out.point.z = p.x, p.y, p.z
            self.pubs[leg_name].publish(out)

def main():
    rclpy.init()
    node = LegFKNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
