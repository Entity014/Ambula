// rr_leg_inverse_kinematics_point.cpp
#include <cmath>
#include <algorithm>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

static inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
static inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

class RRLegInverseKinematics : public rclcpp::Node
{
public:
    RRLegInverseKinematics()
        : Node("rr_leg_inverse_kinematics")
    {
        // ---- topics ----
        this->declare_parameter<std::string>("target_topic", "/rr_foot_target");
        this->declare_parameter<std::string>("cmd_topic", "/rr_leg_joint_cmd");

        // ---- joint offsets (deg) ----
        this->declare_parameter<double>("waist_offset_deg", 0.0);
        this->declare_parameter<double>("hip_offset_deg", 0.0);
        this->declare_parameter<double>("knee_offset_deg", 0.0);

        // ---- hip offset ----
        this->declare_parameter<double>("hip_x", 0.0);
        this->declare_parameter<double>("hip_y", 0.2);
        this->declare_parameter<double>("hip_z", 0.0);

        // ---- link lengths ----
        this->declare_parameter<double>("L1", 0.28);
        this->declare_parameter<double>("L2", 0.30);
        this->declare_parameter<double>("L3", 0.075);

        // ---- elbow mode ----
        this->declare_parameter<std::string>("elbow", "down");

        // ---- debug ----
        this->declare_parameter<bool>("print_deg", true);

        // read params
        target_topic_ = this->get_parameter("target_topic").as_string();
        cmd_topic_ = this->get_parameter("cmd_topic").as_string();

        hx_ = this->get_parameter("hip_x").as_double();
        hy_ = this->get_parameter("hip_y").as_double();
        hz_ = this->get_parameter("hip_z").as_double();

        L1_ = this->get_parameter("L1").as_double();
        L2_ = this->get_parameter("L2").as_double();
        L3_ = this->get_parameter("L3").as_double();
        L23_ = L2_ + L3_;

        elbow_ = this->get_parameter("elbow").as_string();
        std::transform(elbow_.begin(), elbow_.end(), elbow_.begin(), ::tolower);

        print_deg_ = this->get_parameter("print_deg").as_bool();

        waist_off_ = deg2rad(this->get_parameter("waist_offset_deg").as_double());
        hip_off_ = deg2rad(this->get_parameter("hip_offset_deg").as_double());
        knee_off_ = deg2rad(this->get_parameter("knee_offset_deg").as_double());

        sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            target_topic_, rclcpp::QoS(10),
            std::bind(&RRLegInverseKinematics::cb, this, std::placeholders::_1));

        pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            cmd_topic_, rclcpp::QoS(10));

        RCLCPP_INFO(this->get_logger(), "IK target: %s -> cmd: %s",
                    target_topic_.c_str(), cmd_topic_.c_str());
    }

private:
    struct QUsed
    {
        double q1;
        double q2;
        double q3;
    };

    QUsed solve_ik_used(double xd, double yd, double zd)
    {
        // --- p_des - h ---
        const double dx = xd - hx_;
        const double dy = yd - hy_;
        const double dz = zd - hz_;

        // --- after align inverse: pA = R_align^T (p_des - h),  R_align = Ry(pi/2)
        // xA = -dz, yA = dy, zA = dx
        const double xA = -dz;
        const double yA = dy;
        const double zA = dx;

        // --- yaw (updated): q1 = atan2(yA, -xA) = atan2(dy, dz)
        const double q1 = std::atan2(dy, dz);

        // --- planar quantities ---
        const double r_xy = std::sqrt(xA * xA + yA * yA); // r = sqrt(dy^2 + dz^2)
        const double r2 = r_xy * r_xy + zA * zA;          // dx^2 + dy^2 + dz^2
        const double r = std::sqrt(r2);                   // distance to target in the 2-link plane

        // reachability
        const double r_min = std::fabs(L1_ - L23_);
        const double r_max = L1_ + L23_;
        if (r > r_max || r < r_min)
            throw std::runtime_error("Target out of reach");

        // cosine law
        double c3 = (r2 - L1_ * L1_ - L23_ * L23_) / (2.0 * L1_ * L23_);
        c3 = std::clamp(c3, -1.0, 1.0);

        const double s3_mag = std::sqrt(std::max(0.0, 1.0 - c3 * c3));
        const double s3 = (elbow_ == "down") ? -s3_mag : s3_mag;

        const double q3 = std::atan2(s3, c3);

        // updated q2: q2 = -(atan2(zA, r_xy) - atan2(L23*s3, L1 + L23*c3))
        const double q2 = -std::atan2(zA, r_xy) + std::atan2(L23_ * s3, L1_ + L23_ * c3);

        return {q1, q2, q3};
    }

    void cb(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        try
        {
            QUsed q = solve_ik_used(msg->x, msg->y, msg->z);

            geometry_msgs::msg::Point out;
            out.x = q.q1 - waist_off_;
            out.y = q.q2 - hip_off_;
            out.z = q.q3 - knee_off_;

            pub_->publish(out);

            if (print_deg_)
            {
                RCLCPP_INFO(this->get_logger(),
                            "cmd(deg): waist=%.2f hip=%.2f knee=%.2f",
                            rad2deg(out.x),
                            rad2deg(out.y),
                            rad2deg(out.z));
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "IK failed: %s", e.what());
        }
    }

private:
    std::string target_topic_;
    std::string cmd_topic_;

    double hx_{0.0}, hy_{0.0}, hz_{0.0};
    double L1_{0.0}, L2_{0.0}, L3_{0.0}, L23_{0.0};

    std::string elbow_;
    bool print_deg_{true};

    double waist_off_{0.0}, hip_off_{0.0}, knee_off_{0.0};

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRLegInverseKinematics>());
    rclcpp::shutdown();
    return 0;
}
