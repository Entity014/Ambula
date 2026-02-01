#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from logging import root
import rclpy
from rclpy.node import Node

import py_trees
import py_trees_ros

from ambula_state_machine.feedforward import FeedForward
from ambula_state_machine.move import DriveTwistTime
from ambula_state_machine.balance import PDBalance, LQRBalance


class AutonomyBehavior(Node):
    def __init__(self):
        super().__init__("autonomy_node")

        self.declare_parameter("tick_period", 0.1)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("euler_axes", "sxyz")
        self.declare_parameter("balance_mode", "pd")

        self.declare_parameter("feedforward.control_hz", 100.0)
        self.declare_parameter("feedforward.v_peak", 0.20)
        self.declare_parameter("feedforward.forward_time", 0.35)
        self.declare_parameter("feedforward.backward_time", 0.35)
        self.declare_parameter("feedforward.ramp_time", 0.10)
        self.declare_parameter("feedforward.yaw_rate", 0.0)
        self.declare_parameter("feedforward.use_pitch_trigger", True)
        self.declare_parameter("feedforward.pitch_trigger_deg", 12.0)
        self.declare_parameter("feedforward.pitch_trigger_mode", "either")  # either|positive|negative
        self.declare_parameter("feedforward.pitch_hold_s", 0.10)

        self.tick_period = float(self.get_parameter("tick_period").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.euler_axes = str(self.get_parameter("euler_axes").value)
        self.balance_mode = str(self.get_parameter("balance_mode").value)

        self.ff = {
            "control_hz": float(self.get_parameter("feedforward.control_hz").value),
            "v_peak": float(self.get_parameter("feedforward.v_peak").value),
            "forward_time": float(self.get_parameter("feedforward.forward_time").value),
            "backward_time": float(self.get_parameter("feedforward.backward_time").value),
            "ramp_time": float(self.get_parameter("feedforward.ramp_time").value),
            "yaw_rate": float(self.get_parameter("feedforward.yaw_rate").value),
            "use_pitch_trigger": bool(self.get_parameter("feedforward.use_pitch_trigger").value),
            "pitch_trigger_deg": float(self.get_parameter("feedforward.pitch_trigger_deg").value),
            "pitch_trigger_mode": str(self.get_parameter("feedforward.pitch_trigger_mode").value),
            "pitch_hold_s": float(self.get_parameter("feedforward.pitch_hold_s").value),
        }

        self.tree = self.create_behavior_tree()

    def create_behavior_tree(self):
        startup = py_trees.composites.Sequence(name="startup", memory=True)
        startup.add_children(
            [
                FeedForward(
                    name="FeedForward",
                    node=self,
                    cmd_vel_topic=self.cmd_vel_topic,
                    imu_topic=self.imu_topic,
                    euler_axes=self.euler_axes,
                    control_hz=self.ff["control_hz"],
                    v_peak=self.ff["v_peak"],
                    forward_time=self.ff["forward_time"],
                    backward_time=self.ff["backward_time"],
                    ramp_time=self.ff["ramp_time"],
                    yaw_rate=self.ff["yaw_rate"],
                    use_pitch_trigger=self.ff["use_pitch_trigger"],
                    pitch_trigger_deg=self.ff["pitch_trigger_deg"],
                    pitch_trigger_mode=self.ff["pitch_trigger_mode"],
                    pitch_hold_s=self.ff["pitch_hold_s"],
                ),
                DriveTwistTime(
                    name="Forward",
                    node=self,
                    cmd_vel_topic=self.cmd_vel_topic,
                    linear_x=1.0,
                    angular_z=0.0,
                    duration=1.0
                )
                
            ]
        )


        if self.balance_mode == "pd":
            self.declare_parameter("pdbalance.kp_pitch", 5.0)
            self.declare_parameter("pdbalance.kd_pitch", 0.1)
            self.declare_parameter("pdbalance.v_limit", 0.5)
            self.declare_parameter("pdbalance.wz_cmd", 0.0)
            self.declare_parameter("pdbalance.pitch_soft_deg", 5.0)

            balance_leaf = PDBalance(
                name="Balance(PD)",
                node=self,
                cmd_vel_topic=self.cmd_vel_topic,
                imu_topic=self.imu_topic,
                euler_axes=self.euler_axes,
                kp_pitch=float(self.get_parameter("pdbalance.kp_pitch").value),
                kd_pitch=float(self.get_parameter("pdbalance.kd_pitch").value),
                v_limit=float(self.get_parameter("pdbalance.v_limit").value),
                wz_cmd=float(self.get_parameter("pdbalance.wz_cmd").value),
                pitch_soft_deg=float(self.get_parameter("pdbalance.pitch_soft_deg").value),
            )
        else:
            self.declare_parameter("lqrbalance.a", 12.0)
            self.declare_parameter("lqrbalance.b", -2.0)
            self.declare_parameter("lqrbalance.c", 20.0)
            self.declare_parameter("lqrbalance.q_pitch", 25.0)
            self.declare_parameter("lqrbalance.q_pitch_rate", 2.0)
            self.declare_parameter("lqrbalance.r_vx", 1.0)
            self.declare_parameter("lqrbalance.v_limit", 0.5)
            self.declare_parameter("lqrbalance.wz_cmd", 0.0)
            self.declare_parameter("lqrbalance.pitch_soft_deg", 5.0)
            self.declare_parameter("lqrbalance.pitch_sign", 1.0)
            self.declare_parameter("lqrbalance.pitch_rate_sign", 1.0)
            self.declare_parameter("lqrbalance.vx_sign", 1.0)

            balance_leaf = LQRBalance(
                name="Balance(LQR)",
                node=self,
                cmd_vel_topic=self.cmd_vel_topic,
                imu_topic=self.imu_topic,
                euler_axes=self.euler_axes,
                a=float(self.get_parameter("lqrbalance.a").value),
                b=float(self.get_parameter("lqrbalance.b").value),
                c=float(self.get_parameter("lqrbalance.c").value),
                q_pitch=float(self.get_parameter("lqrbalance.q_pitch").value),
                q_pitch_rate=float(self.get_parameter("lqrbalance.q_pitch_rate").value),
                r_vx=float(self.get_parameter("lqrbalance.r_vx").value),
                v_limit=float(self.get_parameter("lqrbalance.v_limit").value),
                wz_cmd=float(self.get_parameter("lqrbalance.wz_cmd").value),
                pitch_soft_deg=float(self.get_parameter("lqrbalance.pitch_soft_deg").value),
                pitch_sign=float(self.get_parameter("lqrbalance.pitch_sign").value),
                pitch_rate_sign=float(self.get_parameter("lqrbalance.pitch_rate_sign").value),
                vx_sign=float(self.get_parameter("lqrbalance.vx_sign").value),
            )

        run = py_trees.composites.Parallel(name="loop balance", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
        run.add_children([balance_leaf])

        seq = py_trees.composites.Sequence(name="stand feedforward", memory=True)
        seq.add_children([startup])

        root = py_trees.decorators.OneShot(
            name="root",
            child=seq,
            policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
        )

        tree = py_trees_ros.trees.BehaviourTree(root, unicode_tree_debug=False)
        tree.setup(timeout=15.0, node=self)

        return tree

    def execute(self):
        self.tree.tick_tock(period_ms=self.tick_period * 1000.0)
        rclpy.spin(self)
        rclpy.shutdown()


def main():
    rclpy.init()
    node = AutonomyBehavior()
    node.execute()


if __name__ == "__main__":
    main()
