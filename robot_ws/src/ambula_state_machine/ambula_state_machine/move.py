import py_trees
from py_trees.common import Status
from geometry_msgs.msg import Twist


class DriveTwistTime(py_trees.behaviour.Behaviour):
    """
    BT Leaf: Publish constant Twist (vx, wz) for duration, then SUCCESS.
    Logging style aligned with FeedForward.
    """

    def __init__(self, name, node,
                 cmd_vel_topic="/cmd_vel",
                 linear_x=0.0,     # m/s
                 angular_z=0.0,    # rad/s
                 duration=1.0):    # seconds
        super().__init__(name)
        self.node = node

        self.cmd_vel_topic = cmd_vel_topic
        self.vx = float(linear_x)
        self.wz = float(angular_z)
        self.duration = float(duration)

        self.pub = self.node.create_publisher(Twist, cmd_vel_topic, 10)
        self._start_time = None

    # -----------------------------
    # py_trees lifecycle
    # -----------------------------
    def initialise(self):
        self._start_time = self.node.get_clock().now()

        self.node.get_logger().info(
            f"[{self.name}] START DriveTwistForTime\n"
            f"  cmd_vel_topic : {self.cmd_vel_topic}\n"
            f"  linear_x      : {self.vx:.3f} m/s\n"
            f"  angular_z     : {self.wz:.3f} rad/s\n"
            f"  duration      : {self.duration:.3f} s"
        )

        # publish immediately on start
        self._publish(self.vx, self.wz)

    def update(self):
        now = self.node.get_clock().now()
        elapsed = (now - self._start_time).nanoseconds * 1e-9

        if elapsed < self.duration:
            self._publish(self.vx, self.wz)
            return Status.RUNNING

        # finish
        self.node.get_logger().info(
            f"[{self.name}] DONE ({elapsed:.2f}s ≥ {self.duration:.2f}s) → SUCCESS"
        )
        self._publish(0.0, 0.0)
        return Status.SUCCESS

    def terminate(self, new_status):
        self.node.get_logger().info(
            f"[{self.name}] TERMINATE → {new_status}"
        )
        self._publish(0.0, 0.0)

    # -----------------------------
    # helper
    # -----------------------------
    def _publish(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub.publish(msg)
