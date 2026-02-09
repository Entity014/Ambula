#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <array>
#include <cmath>
#include <algorithm>
#include <string>
#include <chrono>

static inline double clamp(double x, double lo, double hi)
{
    return std::max(lo, std::min(x, hi));
}

// g_B = R(q)^T * [0, 0, -1]
static inline std::array<double, 3>
gravity_body_from_quat(double qx, double qy, double qz, double qw)
{
    double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (n < 1e-12)
        return {0.0, 0.0, -1.0};
    qx /= n;
    qy /= n;
    qz /= n;
    qw /= n;

    const double xx = qx * qx, yy = qy * qy, zz = qz * qz;
    const double xy = qx * qy, xz = qx * qz, yz = qy * qz;
    const double wx = qw * qx, wy = qw * qy, wz = qw * qz;

    const double R00 = 1.0 - 2.0 * (yy + zz);
    const double R01 = 2.0 * (xy - wz);
    const double R02 = 2.0 * (xz + wy);

    const double R10 = 2.0 * (xy + wz);
    const double R11 = 1.0 - 2.0 * (xx + zz);
    const double R12 = 2.0 * (yz - wx);

    const double R20 = 2.0 * (xz - wy);
    const double R21 = 2.0 * (yz + wx);
    const double R22 = 1.0 - 2.0 * (xx + yy);

    const double gBx = R00 * 0.0 + R10 * 0.0 + R20 * (-1.0);
    const double gBy = R01 * 0.0 + R11 * 0.0 + R21 * (-1.0);
    const double gBz = R02 * 0.0 + R12 * 0.0 + R22 * (-1.0);

    return {gBx, gBy, gBz};
}

class PDBalanceWheelTwistNode : public rclcpp::Node
{
public:
    PDBalanceWheelTwistNode() : Node("pd_balance_wheel_twist_node")
    {
        imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
        cmd_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        out_topic_ = declare_parameter<std::string>("out_topic", "/wheel_cmd");

        kp_ = declare_parameter<double>("kp", 18.0);
        kd_ = declare_parameter<double>("kd", 0.6);

        kv_ = declare_parameter<double>("kv", 0.0);
        max_g_ref_ = declare_parameter<double>("max_g_ref", 0.30);

        wheel_radius_ = declare_parameter<double>("wheel_radius", 0.065);
        wheel_separation_ = declare_parameter<double>("wheel_separation", 0.27);

        max_u_ = declare_parameter<double>("max_u", 2.0);
        max_wheel_rad_s_ = declare_parameter<double>("max_wheel_rad_s", 40.0);
        max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 2.0);

        fall_gbz_ = declare_parameter<double>("fall_gbz", 0.45);

        imu_timeout_s_ = declare_parameter<double>("imu_timeout", 0.10);
        odom_timeout_s_ = declare_parameter<double>("odom_timeout", 0.25);

        gravity_axis_ = declare_parameter<int>("gravity_axis", 0); // 0=gBx,1=gBy
        gravity_sign_ = declare_parameter<double>("gravity_sign", 1.0);

        gyro_axis_ = declare_parameter<int>("gyro_axis", 1); // 0=x,1=y,2=z
        gyro_sign_ = declare_parameter<double>("gyro_sign", 1.0);

        // threshold สำหรับบอกทิศ pitch
        pitch_dir_thresh_ = declare_parameter<double>("pitch_dir_thresh", 0.02);

        if (out_topic_ == cmd_topic_)
        {
            RCLCPP_WARN(get_logger(),
                        "WARNING: out_topic == cmd_vel_topic (%s) -> feedback loop risk. Recommend /wheel_cmd",
                        out_topic_.c_str());
        }

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(),
            std::bind(&PDBalanceWheelTwistNode::onImu, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::QoS(10),
            std::bind(&PDBalanceWheelTwistNode::onOdom, this, std::placeholders::_1));

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            cmd_topic_, rclcpp::QoS(10),
            std::bind(&PDBalanceWheelTwistNode::onCmd, this, std::placeholders::_1));

        pub_ = create_publisher<geometry_msgs::msg::Twist>(out_topic_, rclcpp::QoS(10));

        double hz = declare_parameter<double>("control_hz", 200.0);
        const double clamped_hz = std::max(10.0, hz);
        auto period = std::chrono::duration<double>(1.0 / clamped_hz);

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&PDBalanceWheelTwistNode::onTimer, this));

        last_imu_steady_ = std::chrono::steady_clock::now();
        last_odom_steady_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(get_logger(),
                    "PD Balance → Wheel Twist (REV/S)\n"
                    " imu: %s\n odom: %s\n cmd: %s\n out: %s (angular.x=L_rev_s, angular.y=R_rev_s)\n"
                    " kp=%.3f kd=%.3f kv=%.3f",
                    imu_topic_.c_str(), odom_topic_.c_str(), cmd_topic_.c_str(), out_topic_.c_str(),
                    kp_, kd_, kv_);
    }

private:
    void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        last_imu_steady_ = std::chrono::steady_clock::now();
        imu_seen_ = true;

        gB_ = gravity_body_from_quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        const auto &w = msg->angular_velocity;
        double rate = 0.0;
        if (gyro_axis_ == 0)
            rate = w.x;
        else if (gyro_axis_ == 1)
            rate = w.y;
        else
            rate = w.z;

        pitch_rate_ = gyro_sign_ * rate;
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        last_odom_steady_ = std::chrono::steady_clock::now();
        odom_seen_ = true;
        v_ = msg->twist.twist.linear.x;
    }

    void onCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        v_ref_ = msg->linear.x;
        yaw_ref_ = msg->angular.z;
    }

    void onTimer()
    {
        if (!imu_seen_)
        {
            publishZero("IMU not received yet");
            return;
        }
        const double imu_dt = std::chrono::duration<double>(
                                  std::chrono::steady_clock::now() - last_imu_steady_)
                                  .count();
        if (imu_dt > imu_timeout_s_)
        {
            publishZero("IMU timeout");
            return;
        }

        if (!odom_seen_)
        {
            publishZero("Odom not received yet");
            return;
        }
        const double odom_dt = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - last_odom_steady_)
                                   .count();
        if (odom_dt > odom_timeout_s_)
        {
            publishZero("Odom timeout");
            return;
        }

        if (std::fabs(gB_[2]) < fall_gbz_)
        {
            publishZero("Tilt too large (|gBz| small)");
            return;
        }

        // pitch proxy จาก projected gravity
        const double g_meas = (gravity_axis_ == 0) ? gB_[0] : gB_[1];
        const double g_ref = clamp(kv_ * (v_ref_ - v_), -max_g_ref_, +max_g_ref_);
        const double err = gravity_sign_ * (g_ref - g_meas);

        // PD -> forward command (m/s equivalent)
        double u = kp_ * err + kd_ * (0.0 - pitch_rate_);
        u = clamp(u, -max_u_, +max_u_);

        // yaw clamp
        const double yaw = clamp(yaw_ref_, -max_yaw_rate_, +max_yaw_rate_);

        // wheel rad/s
        const double r = std::max(1e-6, wheel_radius_);
        const double L = wheel_separation_;

        double wL_rad_s = (u / r) - (L / (2.0 * r)) * yaw;
        double wR_rad_s = (u / r) + (L / (2.0 * r)) * yaw;

        wL_rad_s = clamp(wL_rad_s, -max_wheel_rad_s_, +max_wheel_rad_s_);
        wR_rad_s = clamp(wR_rad_s, -max_wheel_rad_s_, +max_wheel_rad_s_);

        // ===== convert to rev/s =====
        const double inv_2pi = 1.0 / (2.0 * M_PI);
        const double wL_rev_s = wL_rad_s * inv_2pi;
        const double wR_rev_s = wR_rad_s * inv_2pi;

        geometry_msgs::msg::Twist out;
        out.angular.x = wL_rev_s; // Left wheel (rev/s)
        out.angular.y = wR_rev_s; // Right wheel (rev/s)
        pub_->publish(out);

        // -------- logger: สั่งเท่าไหร่ + pitch ไปทางไหน --------
        const char *pitch_dir = "LEVEL";
        if (g_meas > pitch_dir_thresh_)
            pitch_dir = "FORWARD";
        else if (g_meas < -pitch_dir_thresh_)
            pitch_dir = "BACKWARD";

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 200,
            "[BAL] pitch=%s | g=%.3f (ref=%.3f) | err=%.3f | gyro=%.3f rad/s | "
            "u=%.3f | wL=%.3f rev/s | wR=%.3f rev/s",
            pitch_dir, g_meas, g_ref, err, pitch_rate_, u, wL_rev_s, wR_rev_s);
    }

    void publishZero(const char *reason)
    {
        geometry_msgs::msg::Twist out;
        out.angular.x = 0.0;
        out.angular.y = 0.0;
        pub_->publish(out);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "STOP: %s", reason);
    }

    // params
    std::string imu_topic_, odom_topic_, cmd_topic_, out_topic_;
    double kp_{18.0}, kd_{0.6};
    double kv_{0.0}, max_g_ref_{0.30};
    double wheel_radius_{0.065}, wheel_separation_{0.27};
    double max_u_{2.0}, max_wheel_rad_s_{40.0}, max_yaw_rate_{2.0};
    double fall_gbz_{0.45};
    double imu_timeout_s_{0.10}, odom_timeout_s_{0.25};
    int gravity_axis_{0};
    double gravity_sign_{1.0};
    int gyro_axis_{1};
    double gyro_sign_{1.0};
    double pitch_dir_thresh_{0.02};

    // state
    std::array<double, 3> gB_{0.0, 0.0, -1.0};
    double pitch_rate_{0.0};
    double v_{0.0}, v_ref_{0.0}, yaw_ref_{0.0};

    // steady clock timestamps
    std::chrono::steady_clock::time_point last_imu_steady_;
    std::chrono::steady_clock::time_point last_odom_steady_;
    bool imu_seen_{false};
    bool odom_seen_{false};

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PDBalanceWheelTwistNode>());
    rclcpp::shutdown();
    return 0;
}
