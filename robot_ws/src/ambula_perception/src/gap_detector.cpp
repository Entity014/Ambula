#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker.hpp"

// โครงสร้างข้อมูลสำหรับจุดพิกัด
struct ScanPoint
{
    float x;
    float y;
};

// โครงสร้างข้อมูลสำหรับวิเคราะห์ช่องว่าง
struct GapInfo
{
    float width;
    float x;
    float y;
};

class GapDetectorLaser : public rclcpp::Node
{
public:
    GapDetectorLaser() : Node("gap_detector_laser_node")
    {
        // --- Parameters ---
        this->declare_parameter("scan_topic", "/scan");
        this->declare_parameter("robot_width", 0.4);    // ความกว้างหุ่นยนต์ (เมตร)
        this->declare_parameter("safety_margin", 0.05); // ระยะเผื่อซ้าย-ขวา (เมตร)

        // ROI: กำหนดขอบเขตการมองเห็น
        this->declare_parameter("roi_x_min", 0.3);
        this->declare_parameter("roi_x_max", 3.0);  // มองไกลได้ แต่ Logic จะเลือกอันใกล้ก่อน
        this->declare_parameter("roi_y_min", -0.5); // ขอบซ้าย
        this->declare_parameter("roi_y_max", 0.5);  // ขอบขวา

        std::string scan_topic = this->get_parameter("scan_topic").as_string();

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10, std::bind(&GapDetectorLaser::scan_callback, this, std::placeholders::_1));

        roi_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/laser_roi", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/perception/gap_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Nearest-Gap Detector Started. Prioritizing close obstacles.");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        double x_min = this->get_parameter("roi_x_min").as_double();
        double x_max = this->get_parameter("roi_x_max").as_double();
        double y_min = this->get_parameter("roi_y_min").as_double();
        double y_max = this->get_parameter("roi_y_max").as_double();

        std::vector<ScanPoint> roi_points;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max || std::isinf(range) || std::isnan(range))
                continue;

            float angle = msg->angle_min + (i * msg->angle_increment);
            float x = range * std::cos(angle);
            float y = range * std::sin(angle);

            // เก็บเฉพาะจุดที่อยู่ใน ROI
            if (x >= x_min && x <= x_max && y >= y_min && y <= y_max)
            {
                roi_points.push_back({x, y});
            }
        }

        // 1. เรียงจุดตามแกน Y (ซ้ายไปขวา) เพื่อหาช่องว่างขวางลำตัว
        std::sort(roi_points.begin(), roi_points.end(), [](const ScanPoint &a, const ScanPoint &b)
                  { return a.y < b.y; });

        publish_roi_cloud(msg->header, roi_points);
        find_nearest_valid_gap(msg->header, roi_points, y_min, y_max);
    }

    void find_nearest_valid_gap(const std_msgs::msg::Header &header, const std::vector<ScanPoint> &points, float y_min, float y_max)
    {
        float robot_width = this->get_parameter("robot_width").as_double();
        float safety = this->get_parameter("safety_margin").as_double();
        float required_width = robot_width + (2.0 * safety);

        std::vector<GapInfo> all_gaps;

        // --- รวบรวม Gap ทั้งหมดที่มี ---
        if (points.empty())
        {
            all_gaps.push_back({y_max - y_min, 1.0f, (y_max + y_min) / 2.0f});
        }
        else
        {
            // Gap ระหว่างขอบซ้าย ROI กับจุดแรก
            all_gaps.push_back({points[0].y - y_min, points[0].x, (points[0].y + y_min) / 2.0f});

            // Gap ระหว่างจุดต่อจุด
            for (size_t i = 0; i < points.size() - 1; ++i)
            {
                float width = points[i + 1].y - points[i].y;
                float avg_x = (points[i].x + points[i + 1].x) / 2.0f;
                float avg_y = (points[i].y + points[i + 1].y) / 2.0f;
                all_gaps.push_back({width, avg_x, avg_y});
            }

            // Gap ระหว่างจุดสุดท้ายกับขอบขวา ROI
            all_gaps.push_back({y_max - points.back().y, points.back().x, (y_max + points.back().y) / 2.0f});
        }

        // --- LOGIC: เรียง Gap ตามระยะทาง X (ใกล้ไปไกล) ---
        std::sort(all_gaps.begin(), all_gaps.end(), [](const GapInfo &a, const GapInfo &b)
                  { return a.x < b.x; });

        GapInfo selected_gap = {0.0, 1.0, 0.0};
        bool found = false;

        // เลือกช่องที่ใกล้ที่สุดที่หุ่นผ่านได้
        for (const auto &gap : all_gaps)
        {
            if (gap.width >= required_width)
            {
                selected_gap = gap;
                found = true;
                break; // เจออันที่ใกล้และกว้างพอแล้ว หยุดตรวจทันที
            }
        }

        // ถ้าไม่มีอันไหนกว้างพอเลย ให้โชว์อันที่กว้างที่สุด (เป็นสีแดง)
        if (!found && !all_gaps.empty())
        {
            auto max_it = std::max_element(all_gaps.begin(), all_gaps.end(), [](const GapInfo &a, const GapInfo &b)
                                           { return a.width < b.width; });
            selected_gap = *max_it;
        }

        publish_gap_marker(header, selected_gap.x, selected_gap.y, selected_gap.width, found);
    }

    void publish_gap_marker(const std_msgs::msg::Header &header, float x, float y, float width, bool valid)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "gap_indicator";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.15;

        marker.scale.x = 0.15;
        marker.scale.y = width; // ยืด Marker ตามความกว้าง Gap
        marker.scale.z = 0.3;

        marker.color.a = 0.8f;
        if (valid)
        {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f; // เขียว = ผ่านได้
        }
        else
        {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f; // แดง = แคบไป
        }

        marker_pub_->publish(marker);
    }

    void publish_roi_cloud(const std_msgs::msg::Header &header, const std::vector<ScanPoint> &points)
    {
        auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pc2_msg->header = header;
        pc2_msg->height = 1;
        pc2_msg->width = points.size();
        sensor_msgs::PointCloud2Modifier modifier(*pc2_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());
        sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2_msg, "x"), iter_y(*pc2_msg, "y"), iter_z(*pc2_msg, "z");
        for (const auto &p : points)
        {
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = 0.0;
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
        roi_pub_->publish(*pc2_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr roi_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapDetectorLaser>());
    rclcpp::shutdown();
    return 0;
}