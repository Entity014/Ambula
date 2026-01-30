#!/usr/bin/env python3
import math
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# -------------------------
# Scalar profiles u(t) in [0,1]
# Return arrays: t, u, udot, uddot, ujerk
# -------------------------

def profile_rectangle(T: float, dt: float) -> Tuple[List[float], List[float], List[float], List[float], List[float]]:
    """
    Rectangle profile on u: u increases linearly from 0->1 with constant udot.
    NOTE: ideal discontinuity in acceleration at ends (a impulse).
    """
    T = max(T, 1e-6)
    dt = max(dt, 1e-4)
    n = max(2, int(math.ceil(T / dt)) + 1)
    t = [i * (T / (n - 1)) for i in range(n)]
    u = [ti / T for ti in t]
    ud = [1.0 / T for _ in t]
    udd = [0.0 for _ in t]
    uj = [0.0 for _ in t]
    u[-1] = 1.0
    ud[-1] = 0.0
    return t, u, ud, udd, uj


def profile_trapezoid(vmax: float, amax: float, dt: float) -> Tuple[List[float], List[float], List[float], List[float], List[float]]:
    """
    Trapezoid on u (distance = 1.0):
      - accelerate with +amax up to vmax
      - cruise at vmax
      - decelerate with -amax to 0
    If distance too short -> triangular (no cruise).

    This is scalar kinematics for u:
      u_ddot in {+amax, 0, -amax}
    """
    vmax = max(vmax, 1e-9)
    amax = max(amax, 1e-9)
    dt = max(dt, 1e-4)

    # Can we reach vmax?
    # distance needed accel+decel: D_need = vmax^2 / amax  (because 2*(0.5*vmax^2/amax))
    D_need = (vmax * vmax) / amax

    if 1.0 >= D_need:
        Ta = vmax / amax
        Da = 0.5 * amax * Ta * Ta
        Dc = 1.0 - 2.0 * Da
        Tv = Dc / vmax
    else:
        # triangular
        vmax = math.sqrt(amax * 1.0)
        Ta = vmax / amax
        Tv = 0.0
        Da = 0.5 * amax * Ta * Ta

    T = 2.0 * Ta + Tv
    n = max(2, int(math.ceil(T / dt)) + 1)
    t = [i * (T / (n - 1)) for i in range(n)]

    u = [0.0] * n
    ud = [0.0] * n
    udd = [0.0] * n

    for i, ti in enumerate(t):
        if ti < Ta:
            udd[i] = amax
            ud[i] = amax * ti
            u[i] = 0.5 * amax * ti * ti
        elif ti < Ta + Tv:
            udd[i] = 0.0
            ud[i] = vmax
            u[i] = Da + vmax * (ti - Ta)
        else:
            td = ti - (Ta + Tv)
            udd[i] = -amax
            ud[i] = vmax - amax * td
            u[i] = Da + (1.0 - 2.0 * Da) + vmax * td - 0.5 * amax * td * td  # Da + Dc + ...

    # numeric jerk (for logging/plot)
    uj = [0.0] * n
    for i in range(1, n):
        uj[i] = (udd[i] - udd[i - 1]) / (t[i] - t[i - 1])

    # snap end
    u[-1] = 1.0
    ud[-1] = 0.0
    udd[-1] = 0.0
    uj[-1] = 0.0
    return t, u, ud, udd, uj


def profile_scurve(vmax: float, amax: float, jmax: float, dt: float):
    """
    7-phase jerk-limited S-curve on u (distance=1.0).
    Scalar constraints:
      udot <= vmax
      |uddot| <= amax
      |ujerk| <= jmax
    Return: t, u, ud, udd, uj
    """
    import math

    vmax = max(vmax, 1e-9)
    amax = max(amax, 1e-9)
    jmax = max(jmax, 1e-9)
    dt = max(dt, 1e-4)

    # Decide if we reach amax
    if vmax >= (amax * amax) / jmax:
        Tj = amax / jmax
        Tc = (vmax / amax) - Tj
    else:
        Tj = math.sqrt(vmax / jmax)
        Tc = 0.0

    a_reached = min(amax, jmax * Tj)

    # Precompute accel-half values (end of phases 1,2,3)
    # Phase1 end:
    u1 = (1.0 / 6.0) * jmax * Tj**3
    ud1 = 0.5 * jmax * Tj**2
    # Phase2 end:
    u2 = u1 + ud1 * Tc + 0.5 * a_reached * Tc**2
    ud2 = ud1 + a_reached * Tc
    # Phase3 end (this is t3):
    u3 = u2 + ud2 * Tj + 0.5 * a_reached * Tj**2 - (1.0 / 6.0) * jmax * Tj**3
    ud_peak = ud2 + a_reached * Tj - 0.5 * jmax * Tj**2  # this is speed during cruise

    u_no_cruise = 2.0 * u3  # total distance without cruise

    if u_no_cruise <= 1.0 + 1e-12:
        Tv = (1.0 - u_no_cruise) / max(ud_peak, 1e-9)
    else:
        # very short move: reduce Tc (keep Tj) by binary search
        Tv = 0.0
        lo, hi = 0.0, Tc
        for _ in range(60):
            mid = 0.5 * (lo + hi)

            u2m = u1 + ud1 * mid + 0.5 * a_reached * mid**2
            ud2m = ud1 + a_reached * mid
            u3m = u2m + ud2m * Tj + 0.5 * a_reached * Tj**2 - (1.0 / 6.0) * jmax * Tj**3
            u_nc = 2.0 * u3m

            if u_nc > 1.0:
                hi = mid
            else:
                lo = mid

        Tc = lo

        # recompute with new Tc
        u2 = u1 + ud1 * Tc + 0.5 * a_reached * Tc**2
        ud2 = ud1 + a_reached * Tc
        u3 = u2 + ud2 * Tj + 0.5 * a_reached * Tj**2 - (1.0 / 6.0) * jmax * Tj**3
        ud_peak = ud2 + a_reached * Tj - 0.5 * jmax * Tj**2
        Tv = 0.0

    Tv = max(0.0, Tv)

    # phase boundaries
    t1 = Tj
    t2 = Tj + Tc
    t3 = 2.0 * Tj + Tc
    t4 = t3 + Tv
    t5 = t4 + Tj
    t6 = t5 + Tc
    t7 = t6 + Tj
    T = t7

    # Precompute boundary states to avoid recursion:
    # At t3: (u3, ud_peak, 0, 0) because accel-half ends with a=0, j=0
    u_t3 = u3
    ud_t3 = ud_peak

    # At t4: end of cruise
    u_t4 = u_t3 + ud_peak * Tv
    ud_t4 = ud_peak

    # At t5: after phase5 (-jerk for Tj)
    # During phase5: a = -j t, ud = ud_peak - 0.5*j t^2, u = u_t4 + ud_peak*t - (1/6)*j t^3
    u_t5 = u_t4 + ud_peak * Tj - (1.0 / 6.0) * jmax * Tj**3
    ud_t5 = ud_peak - 0.5 * jmax * Tj**2  # velocity at start of phase6

    # At t6: after phase6 (-a const for Tc)
    # phase6: a=-a_reached, ud = ud_t5 - a_reached*t, u = u_t5 + ud_t5*t - 0.5*a_reached*t^2
    u_t6 = u_t5 + ud_t5 * Tc - 0.5 * a_reached * Tc**2
    ud_t6 = ud_t5 - a_reached * Tc

    def seg(t: float):
        # returns (u, ud, udd, uj) with NO recursive calls
        if t < t1:  # 1) +jerk
            tau = t
            uj = jmax
            udd = uj * tau
            ud = 0.5 * uj * tau**2
            u = (1.0 / 6.0) * uj * tau**3
            return u, ud, udd, uj

        if t < t2:  # 2) +a const
            tau = t - t1
            uj = 0.0
            udd = a_reached
            ud = ud1 + udd * tau
            u = u1 + ud1 * tau + 0.5 * udd * tau**2
            return u, ud, udd, uj

        if t < t3:  # 3) -jerk
            tau = t - t2
            uj = -jmax
            udd = a_reached + uj * tau
            ud = ud2 + a_reached * tau + 0.5 * uj * tau**2
            u = u2 + ud2 * tau + 0.5 * a_reached * tau**2 + (1.0 / 6.0) * uj * tau**3
            return u, ud, udd, uj

        if t < t4:  # 4) cruise
            tau = t - t3
            uj = 0.0
            udd = 0.0
            ud = ud_t3
            u = u_t3 + ud * tau
            return u, ud, udd, uj

        if t < t5:  # 5) -jerk (start decel)
            tau = t - t4
            uj = -jmax
            udd = uj * tau
            ud = ud_t4 + 0.5 * uj * tau**2
            u = u_t4 + ud_t4 * tau + (1.0 / 6.0) * uj * tau**3
            return u, ud, udd, uj

        if t < t6:  # 6) -a const
            tau = t - t5
            uj = 0.0
            udd = -a_reached
            ud = ud_t5 + udd * tau
            u = u_t5 + ud_t5 * tau + 0.5 * udd * tau**2
            return u, ud, udd, uj

        # 7) +jerk (release decel)
        tau = t - t6
        uj = jmax
        udd = -a_reached + uj * tau
        ud = ud_t6 + (-a_reached) * tau + 0.5 * uj * tau**2
        u = u_t6 + ud_t6 * tau + 0.5 * (-a_reached) * tau**2 + (1.0 / 6.0) * uj * tau**3
        return u, ud, udd, uj

    n = max(2, int(math.ceil(T / dt)) + 1)
    t = [i * (T / (n - 1)) for i in range(n)]
    u, ud, udd, uj = [], [], [], []
    for ti in t:
        ui, udi, uddi, uji = seg(ti)
        u.append(ui); ud.append(udi); udd.append(uddi); uj.append(uji)

    # snap end
    u[-1] = 1.0
    ud[-1] = 0.0
    udd[-1] = 0.0
    uj[-1] = 0.0
    return t, u, ud, udd, uj


# -------------------------
# ROS2 Node: Joint-space smoothing using selectable profile
# -------------------------

class JointSpaceSmoother(Node):
    """
    Subscribe:
      - state_topic  (JointState): current joints (for q0)
      - target_topic (JointState): new target joint positions (q*)
    Publish:
      - out_topic    (JointState): smoothed command (q(t))

    Profiles: rectangle | trapezoid | s_curve
    We compute conservative scalar limits so all joints satisfy their per-joint limits:
      u_dot_max = min_i (v_max_i / |dq_i|)
      u_ddot_max = min_i (a_max_i / |dq_i|)
      u_jerk_max = min_i (j_max_i / |dq_i|)
    Then build u(t), command q(t)=q0 + u(t)*dq (sync multi-axis).
    """

    def __init__(self):
        super().__init__("joint_space_smoother")

        self.declare_parameter("state_topic", "/joint_states/hardware")
        self.declare_parameter("target_topic", "/ambula/left_leg_joint_cmd")
        self.declare_parameter("out_topic", "/ambula/left_leg_joint_cmd_smoothed")
        self.declare_parameter("joint_names", ["left_hip", "left_knee"])

        self.declare_parameter("profile", "s_curve")  # rectangle|trapezoid|s_curve

        # per-joint limits in rad/s, rad/s^2, rad/s^3
        self.declare_parameter("v_max", [2.0, 2.0])
        self.declare_parameter("a_max", [10.0, 10.0])
        self.declare_parameter("j_max", [100.0, 100.0])  # only used for s_curve

        self.declare_parameter("publish_rate", 200.0)
        self.declare_parameter("dt_profile", 0.002)
        self.declare_parameter("frame_id", "base_link")

        self.state_topic = str(self.get_parameter("state_topic").value)
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.out_topic = str(self.get_parameter("out_topic").value)
        self.joint_names = list(self.get_parameter("joint_names").value)

        self.profile = str(self.get_parameter("profile").value).strip().lower()

        self.v_max = list(self.get_parameter("v_max").value)
        self.a_max = list(self.get_parameter("a_max").value)
        self.j_max = list(self.get_parameter("j_max").value)

        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.dt_profile = float(self.get_parameter("dt_profile").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        # normalize limit list lengths
        if len(self.v_max) != len(self.joint_names):
            self.v_max = [float(self.v_max[0])] * len(self.joint_names)
        if len(self.a_max) != len(self.joint_names):
            self.a_max = [float(self.a_max[0])] * len(self.joint_names)
        if len(self.j_max) != len(self.joint_names):
            self.j_max = [float(self.j_max[0])] * len(self.joint_names)

        # state / trajectory
        self.q_current: Dict[str, float] = {}
        self.have_state = False

        self.active = False
        self.q0: Dict[str, float] = {}
        self.dq: Dict[str, float] = {}
        self.t_arr: List[float] = []
        self.u_arr: List[float] = []
        self.start_time = None
        self.idx = 0

        # ROS I/O
        self.sub_state = self.create_subscription(JointState, self.state_topic, self.cb_state, 30)
        self.sub_target = self.create_subscription(JointState, self.target_topic, self.cb_target, 30)
        self.pub_out = self.create_publisher(JointState, self.out_topic, 30)

        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(f"state_topic={self.state_topic}")
        self.get_logger().info(f"target_topic={self.target_topic}")
        self.get_logger().info(f"out_topic={self.out_topic}")
        self.get_logger().info(f"joints={self.joint_names}")
        self.get_logger().info(f"profile={self.profile}")

    def cb_state(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        ok = True
        for jn in self.joint_names:
            i = name_to_idx.get(jn, None)
            if i is None or i >= len(msg.position):
                ok = False
                continue
            self.q_current[jn] = float(msg.position[i])
        if ok:
            self.have_state = True

    def cb_target(self, msg: JointState):
        if not self.have_state and not self.q_current:
            self.get_logger().warn("No joint state yet; wait for state_topic.")
            return

        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        q_star: Dict[str, float] = {}
        for jn in self.joint_names:
            i = name_to_idx.get(jn, None)
            if i is None or i >= len(msg.position):
                self.get_logger().warn(f"Target missing '{jn}'. Have={list(msg.name)}")
                return
            q_star[jn] = float(msg.position[i])

        # pick q0 from measured (best), else last command
        q0 = {jn: float(self.q_current.get(jn, q_star[jn])) for jn in self.joint_names}
        dq = {jn: (q_star[jn] - q0[jn]) for jn in self.joint_names}

        if all(abs(dq[jn]) < 1e-9 for jn in self.joint_names):
            return

        # conservative scalar limits
        u_vmax = float("inf")
        u_amax = float("inf")
        u_jmax = float("inf")
        for k, jn in enumerate(self.joint_names):
            d = abs(dq[jn])
            if d < 1e-9:
                continue
            u_vmax = min(u_vmax, self.v_max[k] / d)
            u_amax = min(u_amax, self.a_max[k] / d)
            u_jmax = min(u_jmax, self.j_max[k] / d)

        if not math.isfinite(u_vmax) or not math.isfinite(u_amax):
            return

        # build u(t)
        if self.profile == "rectangle":
            # choose T from vmax (time to move u=1 at speed u_vmax)
            T = 1.0 / max(u_vmax, 1e-9)
            t_arr, u_arr, *_ = profile_rectangle(T=T, dt=self.dt_profile)

        elif self.profile == "trapezoid":
            t_arr, u_arr, *_ = profile_trapezoid(vmax=u_vmax, amax=u_amax, dt=self.dt_profile)

        elif self.profile in ("s_curve", "scurve", "s-curve"):
            if not math.isfinite(u_jmax):
                u_jmax = 1e9
            t_arr, u_arr, *_ = profile_scurve(vmax=u_vmax, amax=u_amax, jmax=u_jmax, dt=self.dt_profile)

        else:
            self.get_logger().warn(f"Unknown profile='{self.profile}', fallback to s_curve")
            t_arr, u_arr, *_ = profile_scurve(vmax=u_vmax, amax=u_amax, jmax=u_jmax, dt=self.dt_profile)

        # activate
        self.q0 = q0
        self.dq = dq
        self.t_arr = t_arr
        self.u_arr = u_arr
        self.idx = 0
        self.active = True
        self.start_time = self.get_clock().now()

        self.get_logger().info(
            f"New target -> {self.profile}. "
            f"T={t_arr[-1]:.4f}s, u_vmax={u_vmax:.4g}, u_amax={u_amax:.4g}, u_jmax={u_jmax:.4g}"
        )

    def on_timer(self):
        if not self.active:
            return

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        while self.idx < len(self.t_arr) - 1 and self.t_arr[self.idx + 1] <= elapsed:
            self.idx += 1

        u = float(self.u_arr[self.idx])

        out = JointState()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self.frame_id
        out.name = list(self.joint_names)
        out.position = [self.q0[jn] + u * self.dq[jn] for jn in self.joint_names]
        self.pub_out.publish(out)

        # keep last commanded as fallback state
        for jn, q in zip(out.name, out.position):
            self.q_current[jn] = float(q)

        if self.idx >= len(self.t_arr) - 1:
            self.active = False


def main():
    rclpy.init()
    node = JointSpaceSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
