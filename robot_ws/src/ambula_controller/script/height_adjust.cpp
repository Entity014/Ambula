#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>

class HeightAdjustNode : public rclcpp::Node
{
public:
    HeightAdjustNode() : Node("height_adjust_node")
    {
        // ---- Parameters: height & stance ----
        declare_parameter<double>("default_height", 0.35);
        declare_parameter<double>("min_height", 0.20);
        declare_parameter<double>("max_height", 0.60);

        declare_parameter<double>("foot_offset_x", 0.0);
        declare_parameter<double>("foot_offset_y", 0.10); // half width

        // ---- Parameters: topics (adjustable) ----
        declare_parameter<std::string>("topic_height_cmd", "/ambula/height");
        declare_parameter<std::string>("topic_left_foot_target", "/ambula/left_foot_target");
        declare_parameter<std::string>("topic_right_foot_target", "/ambula/right_foot_target");

        load_params();

        // create pub/sub using current params
        setup_interfaces();

        // publish initial pose
        publishTargets();

        // ---- Optional: allow changing topics at runtime ----
        param_cb_handle_ = add_on_set_parameters_callback(
            std::bind(&HeightAdjustNode::onParamsChanged, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Height Adjust Node Started");
    }

private:
    void load_params()
    {
        height_ = get_parameter("default_height").as_double();
        min_height_ = get_parameter("min_height").as_double();
        max_height_ = get_parameter("max_height").as_double();
        foot_offset_x_ = get_parameter("foot_offset_x").as_double();
        foot_offset_y_ = get_parameter("foot_offset_y").as_double();

        topic_height_cmd_ = get_parameter("topic_height_cmd").as_string();
        topic_left_foot_target_ = get_parameter("topic_left_foot_target").as_string();
        topic_right_foot_target_ = get_parameter("topic_right_foot_target").as_string();
    }

    void setup_interfaces()
    {
        // reset old interfaces (important when runtime change)
        sub_height_.reset();
        pub_left_.reset();
        pub_right_.reset();

        // ---- Subscriber ----
        sub_height_ = create_subscription<std_msgs::msg::Float64>(
            topic_height_cmd_,
            rclcpp::QoS(10),
            std::bind(&HeightAdjustNode::heightCallback, this, std::placeholders::_1));

        // ---- Publishers ----
        pub_left_ = create_publisher<geometry_msgs::msg::Point>(
            topic_left_foot_target_, rclcpp::QoS(10));

        pub_right_ = create_publisher<geometry_msgs::msg::Point>(
            topic_right_foot_target_, rclcpp::QoS(10));

        RCLCPP_INFO(get_logger(), "Subscribed: %s", topic_height_cmd_.c_str());
        RCLCPP_INFO(get_logger(), "Publish L : %s", topic_left_foot_target_.c_str());
        RCLCPP_INFO(get_logger(), "Publish R : %s", topic_right_foot_target_.c_str());
    }

    rcl_interfaces::msg::SetParametersResult
    onParamsChanged(const std::vector<rclcpp::Parameter> &params)
    {
        bool topics_changed = false;
        bool height_params_changed = false;

        // validate & detect changes
        for (const auto &p : params)
        {
            const auto &name = p.get_name();

            if (name == "topic_height_cmd" ||
                name == "topic_left_foot_target" ||
                name == "topic_right_foot_target")
            {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
                {
                    return fail("Topic parameters must be string.");
                }
                topics_changed = true;
            }

            if (name == "default_height" || name == "min_height" || name == "max_height" ||
                name == "foot_offset_x" || name == "foot_offset_y")
            {
                height_params_changed = true;
            }
        }

        // apply: reload params from node storage
        if (topics_changed || height_params_changed)
        {
            load_params();

            // sanity check ranges
            if (min_height_ > max_height_)
                return fail("min_height must be <= max_height.");

            // clamp current height to new limits
            if (height_ < min_height_)
                height_ = min_height_;
            if (height_ > max_height_)
                height_ = max_height_;

            if (topics_changed)
            {
                setup_interfaces();
            }

            publishTargets();
        }

        rcl_interfaces::msg::SetParametersResult ok;
        ok.successful = true;
        ok.reason = "ok";
        return ok;
    }

    rcl_interfaces::msg::SetParametersResult fail(const std::string &reason)
    {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = false;
        res.reason = reason;
        return res;
    }

    void heightCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double new_height = msg->data;

        // Clamp height
        if (new_height < min_height_)
            new_height = min_height_;
        if (new_height > max_height_)
            new_height = max_height_;

        height_ = new_height;
        publishTargets();
    }

    void publishTargets()
    {
        if (!pub_left_ || !pub_right_)
            return;

        geometry_msgs::msg::Point left_target;
        geometry_msgs::msg::Point right_target;

        // x = forward/back, y = lateral, z = vertical(up+)
        left_target.x = foot_offset_x_;
        left_target.y = foot_offset_y_;
        left_target.z = height_;

        right_target.x = foot_offset_x_;
        right_target.y = -foot_offset_y_;
        right_target.z = height_;

        pub_left_->publish(left_target);
        pub_right_->publish(right_target);

        RCLCPP_INFO(get_logger(),
                    "Height=%.3f | L(z)=%.3f R(z)=%.3f",
                    height_, left_target.z, right_target.z);
    }

    // ---- Parameters ----
    double height_{0.35};
    double min_height_{0.20};
    double max_height_{0.60};
    double foot_offset_x_{0.0};
    double foot_offset_y_{0.10};

    std::string topic_height_cmd_{"/ambula/height"};
    std::string topic_left_foot_target_{"/ambula/left_foot_target"};
    std::string topic_right_foot_target_{"/ambula/right_foot_target"};

    // ---- ROS Interfaces ----
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_height_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_left_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_right_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeightAdjustNode>());
    rclcpp::shutdown();
    return 0;
}
