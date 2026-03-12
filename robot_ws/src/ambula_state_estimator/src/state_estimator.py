#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import PoseStamped
import pinocchio as pin
import numpy as np

class AmbulaStateEstimator(Node):
    def __init__(self):
        super().__init__('ambula_state_estimator')
        
        # ==========================================
        # Declare Parameters
        # ==========================================
        self.declare_parameter('loop_rate_hz',            100.0)
        self.declare_parameter('urdf_path',               '/home/robot_ws/src/ambula_simulation/ambula_description/urdf/ambula_bot_v3/ambula.urdf')
        self.declare_parameter('topic_imu',               '/imu/data_raw')
        self.declare_parameter('topic_joint_state',       '/joint_states/hardware')
        self.declare_parameter('topic_pose_pub',          '/estimated_base_pose')
        self.declare_parameter('frame_base_link',         'base_link')
        self.declare_parameter('frame_left_wheel',        'left_wheel_link')
        self.declare_parameter('frame_right_wheel',       'right_wheel_link')
        self.declare_parameter('frame_left_hip',          'left_hip_link')
        self.declare_parameter('frame_right_hip',         'right_hip_link')
        self.declare_parameter('joint_names',             ['left_hip_joint', 'left_knee_joint', 'right_hip_joint', 'right_knee_joint'])
        self.declare_parameter('joint_stale_timeout_sec', 0.1)

        # ==========================================
        # Get Parameters
        # ==========================================
        self.loop_rate          = self.get_parameter('loop_rate_hz').value
        urdf_path               = self.get_parameter('urdf_path').value
        topic_imu               = self.get_parameter('topic_imu').value
        topic_joint_state       = self.get_parameter('topic_joint_state').value
        topic_pose_pub          = self.get_parameter('topic_pose_pub').value
        frame_base_link         = self.get_parameter('frame_base_link').value
        frame_left_wheel        = self.get_parameter('frame_left_wheel').value
        frame_right_wheel       = self.get_parameter('frame_right_wheel').value
        frame_left_hip          = self.get_parameter('frame_left_hip').value
        frame_right_hip         = self.get_parameter('frame_right_hip').value
        joint_names_to_track    = self.get_parameter('joint_names').value
        self.stale_timeout      = self.get_parameter('joint_stale_timeout_sec').value

        timer_period = 1.0 / self.loop_rate

        # ==========================================
        # Load Pinocchio Model
        # ==========================================
        try:
            self.model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
            self.data  = self.model.createData()
            self.get_logger().info(f"Model loaded! nq={self.model.nq}, nv={self.model.nv}")
        except Exception as e:
            self.get_logger().error(f"Failed to load URDF: {e}")
            return

        # Frame IDs
        self.base_link_id   = self.model.getFrameId(frame_base_link)
        self.left_wheel_id  = self.model.getFrameId(frame_left_wheel)
        self.right_wheel_id = self.model.getFrameId(frame_right_wheel)
        self.left_hip_id    = self.model.getFrameId(frame_left_hip)
        self.right_hip_id   = self.model.getFrameId(frame_right_hip)

        # Joint idx_q map
        self.joint_q_idx = {}
        for name in joint_names_to_track:
            if self.model.existJointName(name):
                jid = self.model.getJointId(name)
                self.joint_q_idx[name] = self.model.joints[jid].idx_q
            else:
                self.get_logger().warn(f"Joint '{name}' not found in URDF!")

        self.q = pin.neutral(self.model)

        # ==========================================
        # ROS 2 Communication
        # ==========================================
        self.imu_sub   = self.create_subscription(Imu,        topic_imu,         self.imu_callback,   10)
        self.joint_sub = self.create_subscription(JointState, topic_joint_state,  self.joint_callback, 10)
        self.pose_pub  = self.create_publisher(PoseStamped,   topic_pose_pub,     10)

        self.imu_quat       = [0.0, 0.0, 0.0, 1.0]
        self.joint_pos      = {name: 0.0 for name in joint_names_to_track}
        self.last_joint_time = None

        self.timer = self.create_timer(timer_period, self.control_loop)
        self.get_logger().info(f"Ambula State Estimator running at {self.loop_rate} Hz")

    # ------------------------------------------------------------------
    def imu_callback(self, msg):
        self.imu_quat[0] = msg.orientation.x
        self.imu_quat[1] = msg.orientation.y
        self.imu_quat[2] = msg.orientation.z
        self.imu_quat[3] = msg.orientation.w

    def joint_callback(self, msg):
        self.last_joint_time = self.get_clock().now()
        for i, name in enumerate(msg.name):
            if name in self.joint_pos:
                self.joint_pos[name] = msg.position[i]

    # ------------------------------------------------------------------
    def control_loop(self):
        # Safety: เช็ค joint stale
        if self.last_joint_time is not None:
            age = (self.get_clock().now() - self.last_joint_time).nanoseconds / 1e9
            if age > self.stale_timeout:
                self.get_logger().warn(
                    f"Joint data stale: {age:.3f}s > {self.stale_timeout}s",
                    throttle_duration_sec=1.0
                )

        # ------------------------------------------
        # Step 1: อัปเดต q
        # ------------------------------------------
        self.q[0:3] = [0.0, 0.0, 0.0]
        self.q[3:7] = self.imu_quat

        for name, pos in self.joint_pos.items():
            if name in self.joint_q_idx:
                self.q[self.joint_q_idx[name]] = pos

        # ------------------------------------------
        # Step 2: Forward Kinematics
        # ------------------------------------------
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        pos_base        = self.data.oMf[self.base_link_id].translation
        pos_wheel_left  = self.data.oMf[self.left_wheel_id].translation
        pos_wheel_right = self.data.oMf[self.right_wheel_id].translation
        pos_hip_left    = self.data.oMf[self.left_hip_id].translation
        pos_hip_right   = self.data.oMf[self.right_hip_id].translation

        # ------------------------------------------
        # Step 3: คำนวณระยะ
        # ------------------------------------------
        vec_left             = pos_wheel_left  - pos_hip_left
        vec_right            = pos_wheel_right - pos_hip_right
        dist_hip2wheel_left  = np.linalg.norm(vec_left)
        dist_hip2wheel_right = np.linalg.norm(vec_right)

        dist_left  = np.linalg.norm(pos_base - pos_wheel_left)
        dist_right = np.linalg.norm(pos_base - pos_wheel_right)

        if pos_wheel_left[2] <= pos_wheel_right[2]:
            contact_leg = "LEFT"
            current_Z   = dist_left
        else:
            contact_leg = "RIGHT"
            current_Z   = dist_right

        # ------------------------------------------
        # Step 4: Logging
        # ------------------------------------------
        #self.get_logger().info(
        #    f"\n--- Ambula State Estimation ---\n"
        #    f"Rate              : {self.loop_rate} Hz\n"
        #    f"IMU [x,y,z,w]     : [{self.q[3]:.3f}, {self.q[4]:.3f}, {self.q[5]:.3f}, {self.q[6]:.3f}]\n"
        #    f"pos_base          : [{pos_base[0]:.4f}, {pos_base[1]:.4f}, {pos_base[2]:.4f}] m\n"
        #    f"\n[LEFT LEG]\n"
        #    f"  hip_link        : [{pos_hip_left[0]:.4f}, {pos_hip_left[1]:.4f}, {pos_hip_left[2]:.4f}] m\n"
        #    f"  wheel_link      : [{pos_wheel_left[0]:.4f}, {pos_wheel_left[1]:.4f}, {pos_wheel_left[2]:.4f}] m\n"
        #    f"  vec hip→wheel   : [{vec_left[0]:.4f}, {vec_left[1]:.4f}, {vec_left[2]:.4f}]\n"
        #    f"  dist hip→wheel  : {dist_hip2wheel_left:.4f} m  |  dist base→wheel: {dist_left:.4f} m\n"
        #    f"\n[RIGHT LEG]\n"
        #    f"  hip_link        : [{pos_hip_right[0]:.4f}, {pos_hip_right[1]:.4f}, {pos_hip_right[2]:.4f}] m\n"
        #    f"  wheel_link      : [{pos_wheel_right[0]:.4f}, {pos_wheel_right[1]:.4f}, {pos_wheel_right[2]:.4f}] m\n"
        #    f"  vec hip→wheel   : [{vec_right[0]:.4f}, {vec_right[1]:.4f}, {vec_right[2]:.4f}]\n"
        #    f"  dist hip→wheel  : {dist_hip2wheel_right:.4f} m  |  dist base→wheel: {dist_right:.4f} m\n"
        #    f"\nHip  L={self.joint_pos['left_hip_joint']:.3f}  R={self.joint_pos['right_hip_joint']:.3f}\n"
        #    f"Knee L={self.joint_pos['left_knee_joint']:.3f}  R={self.joint_pos['right_knee_joint']:.3f}\n"
        #    f"Contact leg       : {contact_leg}\n"
        #    f">>> ESTIMATED BASE Z : {current_Z:.4f} m <<<\n",
        #    throttle_duration_sec=0.5
        #)

        # ------------------------------------------
        # Step 5: Publish
        # ------------------------------------------
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "odom"
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = current_Z
        pose_msg.pose.orientation.x = float(self.imu_quat[0])
        pose_msg.pose.orientation.y = float(self.imu_quat[1])
        pose_msg.pose.orientation.z = float(self.imu_quat[2])
        pose_msg.pose.orientation.w = float(self.imu_quat[3])
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AmbulaStateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()