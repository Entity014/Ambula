// rr_leg_inverse_kinematics_point.cpp
#include <cmath>
#include <algorithm>
#include <sstream>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

static inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
static inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

// wrap angle to (-pi, pi]
static inline double wrapToPi(double a)
{
    // same spirit as: Mod(a+pi, 2*pi)-pi
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0.0)
        a += 2.0 * M_PI;
    return a - M_PI;
}

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

        // ---- kept for compatibility (but we will force elbow=-1 policy below) ----
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

    // ---- IK exactly following your Sympy "one-solution: q1b + elbow=-1" ----
    QUsed solve_ik_used(double xd, double yd, double zd)
    {
        // Sympy symbols mapping:
        // s = y - hy
        // t = z - hz
        const double s = yd - hy_;
        const double t = zd - hz_;
        const double r = std::sqrt(s * s + t * t); // |U|

        // q1a = atan2(s, -t)
        // q1b = q1a + pi
        double q1 = std::atan2(s, -t) + M_PI;
        q1 = wrapToPi(q1);

        // dx = hx - x
        const double dx = hx_ - xd;

        // choose ONLY q1b branch: U = -r
        const double u = -r;
        const double v = dx;

        // D = (u^2+v^2 - L1^2 - L23^2) / (2 L1 L23)
        const double u2v2 = u * u + v * v;
        const double r_min = std::fabs(L1_ - L23_);
        const double r_max = L1_ + L23_;
        const double dist = std::sqrt(u2v2);

        if (dist > r_max + 1e-9 || dist < r_min - 1e-9)
        {
            std::ostringstream ss;
            ss << "Target out of reach: dist=" << dist
               << " (min=" << r_min << ", max=" << r_max << ")";
            throw std::runtime_error(ss.str());
        }

        double D = (u2v2 - L1_ * L1_ - L23_ * L23_) / (2.0 * L1_ * L23_);
        D = std::clamp(D, -1.0, 1.0);

        // elbow = -1 => s3 = -sqrt(1-D^2)
        const double s3 = -std::sqrt(std::max(0.0, 1.0 - D * D));

        // q3 = atan2(s3, D)
        double q3 = std::atan2(s3, D);

        // q2 = atan2(v,u) - atan2(L23*s3, L1 + L23*D)
        double q2 = std::atan2(v, u) - std::atan2(L23_ * s3, L1_ + L23_ * D);

        q2 = wrapToPi(q2);
        q3 = wrapToPi(q3);

        return {q1, q2, q3};
    }

    void cb(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        try
        {
            const QUsed q = solve_ik_used(msg->x, msg->y, msg->z);

            geometry_msgs::msg::Point out;
            out.x = wrapToPi(q.q1 - waist_off_);
            out.y = wrapToPi(q.q2 - hip_off_);
            out.z = wrapToPi(q.q3 - knee_off_);

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
