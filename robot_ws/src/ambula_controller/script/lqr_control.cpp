#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <Eigen/Dense>

#include <mutex>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

static inline double clamp(double x, double lo, double hi)
{
    return std::max(lo, std::min(hi, x));
}

static inline double radps_to_revps(double rad_s)
{
    return rad_s / (2.0 * M_PI);
}

/**
 * Compute gravity direction (unit vector) in BODY frame from IMU quaternion.
 *
 * Assumption:
 * - msg->orientation represents rotation R_bw (body -> world)
 * - world gravity g_w = [0, 0, -1]
 * - g_b = R_bw^T * g_w
 *
 * Returned (gx,gy,gz) is unit-length.
 */
static inline void gravity_body_from_quat(
    double qx, double qy, double qz, double qw,
    double &gx, double &gy, double &gz)
{
    const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (n < 1e-12)
    {
        gx = 0.0;
        gy = 0.0;
        gz = -1.0;
        return;
    }
    qx /= n;
    qy /= n;
    qz /= n;
    qw /= n;

    // Rotation matrix R_bw (body->world) from quaternion (x,y,z,w)
    const double xx = qx * qx, yy = qy * qy, zz = qz * qz;
    const double xz = qx * qz, yz = qy * qz;
    const double wy = qw * qy, wx = qw * qx;

    // only need column 2: [R02, R12, R22]
    const double R02 = 2.0 * (xz + wy);
    const double R12 = 2.0 * (yz - wx);
    const double R22 = 1.0 - 2.0 * (xx + yy);

    // g_w=[0,0,-1] => g_b = R^T g_w = -col2(R)
    gx = -R02;
    gy = -R12;
    gz = -R22;

    const double gn = std::sqrt(gx * gx + gy * gy + gz * gz);
    if (gn > 1e-12)
    {
        gx /= gn;
        gy /= gn;
        gz /= gn;
    }
}

/**
 * Pitch from projected gravity.
 * Convention: x forward, z up, pitch about +Y:
 * theta = atan2(gx, gz)
 *
 * If sign is wrong:
 * try atan2(-gx,gz) or atan2(gx,-gz)
 */
static inline double pitch_from_projected_gravity(double gx, double gy, double gz)
{
    (void)gy;
    return std::atan2(gx, gz);
}

class LqrBalanceNode : public rclcpp::Node
{
public:
    LqrBalanceNode() : Node("lqr_balance_node")
    {
        // Topics
        imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu/data");
        joint_topic_ = this->declare_parameter<std::string>("joint_states_topic", "/wheel_joint_states");
        cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        pub_topic_ = this->declare_parameter<std::string>("wheel_cmd_topic", "/cmd_wheel_twist");

        left_wheel_joint_ = this->declare_parameter<std::string>("left_wheel_joint", "left_wheel_joint");
        right_wheel_joint_ = this->declare_parameter<std::string>("right_wheel_joint", "right_wheel_joint");

        // Geometry
        wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.08);         // [m]
        wheel_separation_ = this->declare_parameter<double>("wheel_separation", 0.30); // [m]
        pitch_offset_rad_ = this->declare_parameter<double>("pitch_offset_rad", 0.0);

        // K 2x4 row-major: u=[v_cmd, r_cmd] = -K x
        // x=[pitch, pitch_rate, v_err, yaw_rate_err]
        K_flat_ = this->declare_parameter<std::vector<double>>(
            "K_flat",
            std::vector<double>{
                // Default: your Kd
                -9.941991, -2.49687, -1.719665, 0.0,
                0.0, 0.0, 0.0, 0.402065});

        // Reference from cmd_vel
        v_ref_ = this->declare_parameter<double>("v_ref", 0.0);               // m/s
        yaw_rate_ref_ = this->declare_parameter<double>("yaw_rate_ref", 0.0); // rad/s

        // Limits (command space)
        max_v_cmd_ = this->declare_parameter<double>("max_v_cmd", 2.0); // m/s
        max_r_cmd_ = this->declare_parameter<double>("max_r_cmd", 4.0); // rad/s

        // Limits (wheel space, internal rad/s)
        max_wheel_rad_s_ = this->declare_parameter<double>("max_wheel_rad_s", 25.0); // rad/s

        // Slew-rate limit (wheel accel) [rad/s^2]
        enable_slew_ = this->declare_parameter<bool>("enable_slew", true);
        max_wheel_acc_rad_s2_ = this->declare_parameter<double>("max_wheel_acc_rad_s2", 300.0);

        // Require cmd_vel freshness
        require_cmd_vel_ = this->declare_parameter<bool>("require_cmd_vel", false);
        cmd_vel_timeout_s_ = this->declare_parameter<double>("cmd_vel_timeout_s", 0.5);

        // Control loop rate
        rate_hz_ = this->declare_parameter<double>("rate_hz", 200.0);

        // Subs
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LqrBalanceNode::on_imu, this, std::placeholders::_1));

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_topic_, rclcpp::QoS(10),
            std::bind(&LqrBalanceNode::on_joint, this, std::placeholders::_1));

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, rclcpp::QoS(10),
            std::bind(&LqrBalanceNode::on_cmd, this, std::placeholders::_1));

        // Pub: Twist.angular.x/y = left/right wheel (rev/s)
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(pub_topic_, rclcpp::QoS(10));

        // Timer loop
        auto period = std::chrono::duration<double>(1.0 / std::max(10.0, rate_hz_));
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&LqrBalanceNode::on_timer, this));

        RCLCPP_INFO(this->get_logger(),
                    "LQR balance node started.\n imu=%s\n joint=%s\n cmd_vel=%s\n pub=%s\n rate=%.1f Hz\n require_cmd_vel=%s timeout=%.2fs",
                    imu_topic_.c_str(),
                    joint_topic_.c_str(),
                    cmd_vel_topic_.c_str(),
                    pub_topic_.c_str(),
                    rate_hz_,
                    require_cmd_vel_ ? "true" : "false",
                    cmd_vel_timeout_s_);
    }

private:
    void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);

        const auto &q = msg->orientation;
        gravity_body_from_quat(q.x, q.y, q.z, q.w, gxb_, gyb_, gzb_);
        pitch_rad_ = pitch_from_projected_gravity(gxb_, gyb_, gzb_) - pitch_offset_rad_;

        // NOTE: adjust axes based on IMU mounting
        pitch_rate_rad_s_ = msg->angular_velocity.y;
        yaw_rate_rad_s_ = msg->angular_velocity.z;

        imu_ok_ = true;
        last_imu_time_ = this->now();
    }

    void on_joint(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);

        int li = -1, ri = -1;
        for (size_t i = 0; i < msg->name.size(); i++)
        {
            if (msg->name[i] == left_wheel_joint_)
                li = static_cast<int>(i);
            if (msg->name[i] == right_wheel_joint_)
                ri = static_cast<int>(i);
        }

        if (li >= 0 && ri >= 0 &&
            msg->velocity.size() > static_cast<size_t>(std::max(li, ri)))
        {
            wheel_l_rad_s_meas_ = msg->velocity[li];
            wheel_r_rad_s_meas_ = msg->velocity[ri];

            const double v_l = wheel_l_rad_s_meas_ * wheel_radius_;
            const double v_r = wheel_r_rad_s_meas_ * wheel_radius_;
            v_m_s_ = 0.5 * (v_l + v_r);

            joint_ok_ = true;
            last_joint_time_ = this->now();
        }
    }

    void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        v_ref_ = msg->linear.x;
        yaw_rate_ref_ = msg->angular.z;
        cmd_ok_ = true;
        last_cmd_time_ = this->now();
    }

    void on_timer()
    {
        // Snapshot
        double pitch, pitch_rate, v, yaw_rate;
        double v_ref, yaw_ref;
        bool ok_imu, ok_joint, ok_cmd;
        rclcpp::Time t_cmd, t_now;

        {
            std::lock_guard<std::mutex> lk(mtx_);
            pitch = pitch_rad_;
            pitch_rate = pitch_rate_rad_s_;
            v = v_m_s_;
            yaw_rate = yaw_rate_rad_s_;
            v_ref = v_ref_;
            yaw_ref = yaw_rate_ref_;

            ok_imu = imu_ok_;
            ok_joint = joint_ok_;
            ok_cmd = cmd_ok_;
            t_cmd = last_cmd_time_;
            t_now = this->now();
        }

        if (!(ok_imu && ok_joint))
            return;

        if (require_cmd_vel_)
        {
            if (!ok_cmd)
                return;
            const double age = (t_now - t_cmd).seconds();
            if (age > cmd_vel_timeout_s_)
                return;
        }

        if (K_flat_.size() != 8)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "K_flat must have 8 elements (2x4). Current size=%zu", K_flat_.size());
            return;
        }

        // error-state x=[pitch, pitch_rate, v_err, yaw_err]
        const double v_err = (v - v_ref);
        const double yaw_err = (yaw_rate - yaw_ref);

        Eigen::Vector4d x;
        x << pitch, pitch_rate, v_err, yaw_err;

        Eigen::Matrix<double, 2, 4> K;
        K << K_flat_[0], K_flat_[1], K_flat_[2], K_flat_[3],
            K_flat_[4], K_flat_[5], K_flat_[6], K_flat_[7];

        // u=[v_cmd, r_cmd] = -Kx
        Eigen::Vector2d u = -(K * x);

        double v_cmd = clamp(u(0), -max_v_cmd_, max_v_cmd_); // m/s
        double r_cmd = clamp(u(1), -max_r_cmd_, max_r_cmd_); // rad/s

        // diff-drive -> wheel rad/s
        const double b = wheel_separation_;
        const double R = wheel_radius_;

        const double vL = v_cmd - 0.5 * b * r_cmd;
        const double vR = v_cmd + 0.5 * b * r_cmd;

        double wl_rad_s = vL / R; // rad/s
        double wr_rad_s = vR / R; // rad/s

        wl_rad_s = clamp(wl_rad_s, -max_wheel_rad_s_, max_wheel_rad_s_);
        wr_rad_s = clamp(wr_rad_s, -max_wheel_rad_s_, max_wheel_rad_s_);

        // slew limit (rad/s)
        if (enable_slew_)
        {
            double dt = 1.0 / std::max(10.0, rate_hz_);
            if (!have_last_time_)
            {
                last_time_ = t_now;
                have_last_time_ = true;
                last_wl_cmd_rad_s_ = wl_rad_s;
                last_wr_cmd_rad_s_ = wr_rad_s;
            }
            else
            {
                dt = (t_now - last_time_).seconds();
                last_time_ = t_now;
                if (dt <= 1e-6)
                    dt = 1.0 / std::max(10.0, rate_hz_);
            }

            const double max_step = max_wheel_acc_rad_s2_ * dt;
            wl_rad_s = clamp(wl_rad_s, last_wl_cmd_rad_s_ - max_step, last_wl_cmd_rad_s_ + max_step);
            wr_rad_s = clamp(wr_rad_s, last_wr_cmd_rad_s_ - max_step, last_wr_cmd_rad_s_ + max_step);

            last_wl_cmd_rad_s_ = wl_rad_s;
            last_wr_cmd_rad_s_ = wr_rad_s;
        }

        // Convert to rev/s at output ONLY
        const double wl_rev_s = radps_to_revps(wl_rad_s);
        const double wr_rev_s = radps_to_revps(wr_rad_s);

        geometry_msgs::msg::Twist out;
        out.linear.x = 0.0;
        out.linear.y = 0.0;
        out.linear.z = 0.0;

        out.angular.x = wl_rev_s; // LEFT  [rev/s]
        out.angular.y = wr_rev_s; // RIGHT [rev/s]
        out.angular.z = 0.0;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            200, // ms
            "[LQR] pitch=%.3f rad | pitch_rate=%.3f rad/s | v=%.2f m/s | "
            "v_cmd=%.2f m/s r_cmd=%.2f rad/s | "
            "wheel_cmd: L=%.3f rev/s R=%.3f rev/s",
            pitch,
            pitch_rate,
            v,
            v_cmd,
            r_cmd,
            wl_rev_s,
            wr_rev_s);

        cmd_pub_->publish(out);
    }

private:
    std::mutex mtx_;

    // Params
    std::string imu_topic_, joint_topic_, cmd_vel_topic_, pub_topic_;
    std::string left_wheel_joint_, right_wheel_joint_;

    double wheel_radius_{0.08};
    double wheel_separation_{0.30};
    double pitch_offset_rad_{0.0};

    std::vector<double> K_flat_;

    double v_ref_{0.0};
    double yaw_rate_ref_{0.0};

    double max_v_cmd_{2.0};
    double max_r_cmd_{4.0};
    double max_wheel_rad_s_{25.0};

    bool enable_slew_{true};
    double max_wheel_acc_rad_s2_{300.0};

    bool require_cmd_vel_{false};
    double cmd_vel_timeout_s_{0.5};
    double rate_hz_{200.0};

    // State cache
    double pitch_rad_{0.0};
    double pitch_rate_rad_s_{0.0};
    double yaw_rate_rad_s_{0.0};

    double wheel_l_rad_s_meas_{0.0};
    double wheel_r_rad_s_meas_{0.0};
    double v_m_s_{0.0};

    // projected gravity in body frame
    double gxb_{0.0}, gyb_{0.0}, gzb_{-1.0};

    bool imu_ok_{false};
    bool joint_ok_{false};
    bool cmd_ok_{false};

    rclcpp::Time last_imu_time_;
    rclcpp::Time last_joint_time_;
    rclcpp::Time last_cmd_time_;

    // Slew memory
    bool have_last_time_{false};
    rclcpp::Time last_time_;
    double last_wl_cmd_rad_s_{0.0};
    double last_wr_cmd_rad_s_{0.0};

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LqrBalanceNode>());
    rclcpp::shutdown();
    return 0;
}
