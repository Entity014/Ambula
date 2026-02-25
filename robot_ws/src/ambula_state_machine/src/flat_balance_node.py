#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from logging import root
import rclpy
from rclpy.node import Node

import py_trees
import py_trees_ros



class AutonomyBehavior(Node):
    def __init__(self):
        super().__init__("autonomy_node")

        self.declare_parameter("tick_period", 0.1)

        self.tick_period = float(self.get_parameter("tick_period").value)

        self.tree = self.create_behavior_tree()

    def create_behavior_tree(self):
        startup = py_trees.composites.Sequence(name="startup", memory=True)
        startup.add_children(
            [   
            ]
        )

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
