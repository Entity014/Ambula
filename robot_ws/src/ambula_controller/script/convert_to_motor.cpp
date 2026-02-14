#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <cmath>
#include <string>

static inline float dir_sign(int direction)
{
    if (direction < 0)
        return -1.0f;
    if (direction > 0)
        return +1.0f;
    return 1.0f;
}

struct JointCfg
{
    float pos_scale;
    float pos_offset_rad;
    bool output_deg;
    bool flip;
    int direction;
};

class JointToMotorRevPointNode : public rclcpp::Node
{
public:
    JointToMotorRevPointNode()
        : Node("joint_to_motor_rev_point_node")
    {
        // topics
        this->declare_parameter("joint_in_topic", "/rr_leg_joint_cmd");
        this->declare_parameter("motor_out_topic", "/motor_rev_cmd");
        this->declare_parameter("print_debug", false);

        // waist params
        this->declare_parameter("waist.pos_scale", 2.0 * M_PI / 20.0);
        this->declare_parameter("waist.offset_deg", 70.0);
        this->declare_parameter("waist.flip", true);
        this->declare_parameter("waist.direction", -1);

        // hip params
        this->declare_parameter("hip.pos_scale", 2.0 * M_PI / 20.0);
        this->declare_parameter("hip.offset_deg", 70.0);
        this->declare_parameter("hip.flip", true);
        this->declare_parameter("hip.direction", -1);

        // knee params
        this->declare_parameter("knee.pos_scale", 2.0 * M_PI / 10.0);
        this->declare_parameter("knee.offset_deg", 150.0);
        this->declare_parameter("knee.flip", false);
        this->declare_parameter("knee.direction", -1);

        load_parameters();

        sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            in_topic_, rclcpp::QoS(10),
            std::bind(&JointToMotorRevPointNode::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            out_topic_, rclcpp::QoS(10));

        RCLCPP_INFO(this->get_logger(), "JointToMotorRevPointNode ready");
    }

private:
    void load_parameters()
    {
        in_topic_ = this->get_parameter("joint_in_topic").as_string();
        out_topic_ = this->get_parameter("motor_out_topic").as_string();
        print_debug_ = this->get_parameter("print_debug").as_bool();

        waist_cfg_ = {
            (float)this->get_parameter("waist.pos_scale").as_double(),
            (float)(this->get_parameter("waist.offset_deg").as_double() * M_PI / 180.0),
            false,
            this->get_parameter("waist.flip").as_bool(),
            this->get_parameter("waist.direction").as_int()};

        hip_cfg_ = {
            (float)this->get_parameter("hip.pos_scale").as_double(),
            (float)(this->get_parameter("hip.offset_deg").as_double() * M_PI / 180.0),
            false,
            this->get_parameter("hip.flip").as_bool(),
            this->get_parameter("hip.direction").as_int()};

        knee_cfg_ = {
            (float)this->get_parameter("knee.pos_scale").as_double(),
            (float)(this->get_parameter("knee.offset_deg").as_double() * M_PI / 180.0),
            false,
            this->get_parameter("knee.flip").as_bool(),
            this->get_parameter("knee.direction").as_int()};
    }

    float reverse_joint(float joint_rad, const JointCfg &cfg)
    {
        float pos = joint_rad;

        float s = dir_sign(cfg.direction);
        pos /= s;

        pos -= cfg.pos_offset_rad;

        float motor_rev = pos / cfg.pos_scale;

        if (cfg.flip)
            motor_rev = -motor_rev;

        return motor_rev;
    }

    void callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        geometry_msgs::msg::Point out;

        out.x = reverse_joint(msg->x, waist_cfg_);
        out.y = reverse_joint(msg->y, hip_cfg_);
        out.z = reverse_joint(msg->z, knee_cfg_);

        pub_->publish(out);

        if (print_debug_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "joint(rad) [%.3f %.3f %.3f] -> motor(rev) [%.3f %.3f %.3f]",
                        msg->x, msg->y, msg->z,
                        out.x, out.y, out.z);
        }
    }

private:
    std::string in_topic_;
    std::string out_topic_;
    bool print_debug_;

    JointCfg waist_cfg_;
    JointCfg hip_cfg_;
    JointCfg knee_cfg_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointToMotorRevPointNode>());
    rclcpp::shutdown();
    return 0;
}
