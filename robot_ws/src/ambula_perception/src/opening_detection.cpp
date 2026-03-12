#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

struct Opening
{
    // center in OPTICAL frame (for publishing markers alongside raw pointcloud)
    float center_x, center_y, center_z;
    float width, height;
    float confidence;
};

// ---------------------------------------------------------------------------
// optical frame  : X=right, Y=down,    Z=forward   (RealSense default)
// robot frame    : X=forward, Y=left,  Z=up
//
//   robot  ← optical :  rx= oz,  ry=-ox,  rz=-oy
//   optical← robot   :  ox=-ry,  oy=-rz,  oz= rx
// ---------------------------------------------------------------------------
inline PointT toRobotFrame(const PointT &p)
{
    PointT r;
    r.x = p.z;
    r.y = -p.x;
    r.z = -p.y;
    return r;
}

inline Eigen::Vector3f robotToOptical(const Eigen::Vector3f &r)
{
    return Eigen::Vector3f(-r.y(), -r.z(), r.x());
}

inline Eigen::Vector3f normalRobotToOptical(const Eigen::Vector3f &n)
{
    return robotToOptical(n);
}

class OpeningDetector : public rclcpp::Node
{
public:
    OpeningDetector() : Node("opening_detector")
    {
        this->declare_parameter("input_topic", "/camera/camera/depth/color/points");
        this->declare_parameter("voxel_size", 0.04f);

        // ROI in ROBOT frame (X=forward, Y=left, Z=up)
        this->declare_parameter("pass_x_min", 0.3f);  // depth near
        this->declare_parameter("pass_x_max", 1.5f);  // depth far
        this->declare_parameter("pass_y_min", -0.5f); // lateral left
        this->declare_parameter("pass_y_max", 0.5f);  // lateral right
        this->declare_parameter("pass_z_min", -0.5f); // height floor
        this->declare_parameter("pass_z_max", 0.5f);  // height ceiling

        this->declare_parameter("ransac_dist", 0.03f);
        this->declare_parameter("grid_res", 0.02f);
        this->declare_parameter("min_open_area", 0.1f);
        this->declare_parameter("max_open_area", 5.0f);
        this->declare_parameter("min_open_w", 0.1f);
        this->declare_parameter("min_open_h", 0.1f);
        this->declare_parameter("max_wall_tilt", 0.3f);
        this->declare_parameter("max_plane_iters", 5);
        this->declare_parameter("min_plane_inliers", 30);

        std::string topic = this->get_parameter("input_topic").as_string();

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(),
            std::bind(&OpeningDetector::cloudCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/opening_detection/markers", 10);

        roi_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/opening_detection/roi_cloud", 10);

        plane_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/opening_detection/plane_cloud", 10);

        RCLCPP_INFO(this->get_logger(),
                    "OpeningDetector ready (no-TF) — '%s'", topic.c_str());
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // ── 1. ROS → PCL  (optical frame) ────────────────────────────────────
        CloudT::Ptr cloud_optical(new CloudT);
        pcl::fromROSMsg(*msg, *cloud_optical);
        if (cloud_optical->empty())
            return;

        // ── 2. Re-express in robot frame so Z=up ─────────────────────────────
        CloudT::Ptr cloud_robot(new CloudT);
        cloud_robot->reserve(cloud_optical->size());
        for (const auto &p : cloud_optical->points)
            cloud_robot->push_back(toRobotFrame(p));

        // ── 3. ROI filter — all 3 axes in robot frame ────────────────────────
        //       X: depth (forward)  0.3 – 1.5 m
        //       Y: lateral (left)  -0.5 – 0.5 m
        //       Z: height (up)     -0.5 – 0.5 m
        float x_min = this->get_parameter("pass_x_min").as_double();
        float x_max = this->get_parameter("pass_x_max").as_double();
        float y_min = this->get_parameter("pass_y_min").as_double();
        float y_max = this->get_parameter("pass_y_max").as_double();
        float z_min = this->get_parameter("pass_z_min").as_double();
        float z_max = this->get_parameter("pass_z_max").as_double();

        CloudT::Ptr filtered(new CloudT);
        {
            CloudT::Ptr tmp1(new CloudT), tmp2(new CloudT);
            pcl::PassThrough<PointT> pass;

            // X: depth
            pass.setInputCloud(cloud_robot);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(x_min, x_max);
            pass.filter(*tmp1);

            // Y: lateral
            pass.setInputCloud(tmp1);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(y_min, y_max);
            pass.filter(*tmp2);

            // Z: height
            pass.setInputCloud(tmp2);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(z_min, z_max);
            pass.filter(*filtered);
        }
        if (filtered->empty())
            return;

        // ── 4. Publish ROI cloud (convert back to optical frame) ─────────────
        //       so it overlaps the raw cloud in Foxglove / RViz
        {
            CloudT::Ptr roi_optical(new CloudT);
            roi_optical->reserve(filtered->size());
            for (const auto &p : filtered->points)
            {
                PointT o;
                o.x = -p.y; // robot→optical: ox = -ry
                o.y = -p.z; //                oy = -rz
                o.z = p.x;  //                oz =  rx
                roi_optical->push_back(o);
            }
            sensor_msgs::msg::PointCloud2 roi_msg;
            pcl::toROSMsg(*roi_optical, roi_msg);
            roi_msg.header = msg->header; // same frame_id as input ✅
            roi_cloud_pub_->publish(roi_msg);
        }

        // ── 5. VoxelGrid ─────────────────────────────────────────────────────
        float voxel = this->get_parameter("voxel_size").as_double();
        CloudT::Ptr downsampled(new CloudT);
        {
            pcl::VoxelGrid<PointT> vg;
            vg.setInputCloud(filtered);
            vg.setLeafSize(voxel, voxel, voxel);
            vg.filter(*downsampled);
        }
        if (downsampled->size() < 50)
            return;

        // ── 6. Iterative RANSAC in robot frame ───────────────────────────────
        float ransac_dist = this->get_parameter("ransac_dist").as_double();
        float max_tilt = this->get_parameter("max_wall_tilt").as_double();
        int max_iters = this->get_parameter("max_plane_iters").as_int();
        int min_inliers = this->get_parameter("min_plane_inliers").as_int();

        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(ransac_dist);

        CloudT::Ptr remaining(new CloudT(*downsampled));
        pcl::ModelCoefficients::Ptr best_coeffs;
        pcl::PointIndices::Ptr best_inliers;

        for (int iter = 0; iter < max_iters; ++iter)
        {
            if (static_cast<int>(remaining->size()) < min_inliers)
                break;

            pcl::ModelCoefficients::Ptr c(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr idx(new pcl::PointIndices);
            seg.setInputCloud(remaining);
            seg.segment(*idx, *c);

            if (static_cast<int>(idx->indices.size()) < min_inliers)
                break;

            Eigen::Vector3f n_robot(c->values[0], c->values[1], c->values[2]);
            n_robot.normalize();

            // Wall plane: small Z-component (not floor/ceiling), large X-component (faces camera)
            if (std::abs(n_robot.z()) < max_tilt && std::abs(n_robot.x()) > 0.5f)
            {
                best_coeffs = c;
                best_inliers = idx;
                break;
            }

            // Peel non-wall plane and retry
            pcl::ExtractIndices<PointT> ex;
            ex.setInputCloud(remaining);
            ex.setIndices(idx);
            ex.setNegative(true);
            CloudT::Ptr next(new CloudT);
            ex.filter(*next);
            remaining = next;
        }

        if (!best_coeffs || !best_inliers)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "No wall plane found");
            return;
        }

        // ── 7. Extract wall inliers (robot frame) ────────────────────────────
        CloudT::Ptr plane_robot(new CloudT);
        {
            pcl::ExtractIndices<PointT> ex;
            ex.setInputCloud(remaining);
            ex.setIndices(best_inliers);
            ex.setNegative(false);
            ex.filter(*plane_robot);
        }

        // ── 8. Publish plane cloud (optical frame, overlaps raw cloud) ────────
        {
            CloudT::Ptr plane_optical(new CloudT);
            plane_optical->reserve(plane_robot->size());
            for (const auto &p : plane_robot->points)
            {
                PointT o;
                o.x = -p.y;
                o.y = -p.z;
                o.z = p.x;
                plane_optical->push_back(o);
            }
            sensor_msgs::msg::PointCloud2 plane_msg;
            pcl::toROSMsg(*plane_optical, plane_msg);
            plane_msg.header = msg->header;
            plane_cloud_pub_->publish(plane_msg);
        }

        // ── 9. Wall-plane basis in robot frame ───────────────────────────────
        Eigen::Vector3f n_robot(best_coeffs->values[0],
                                best_coeffs->values[1],
                                best_coeffs->values[2]);
        n_robot.normalize();

        const Eigen::Vector3f world_up(0.0f, 0.0f, 1.0f);
        Eigen::Vector3f u_robot = n_robot.cross(world_up).normalized();
        Eigen::Vector3f v_robot = n_robot.cross(u_robot).normalized();

        // ── 10. Bounding box in (u, v) plane space ───────────────────────────
        float u_min = 1e9f, u_max = -1e9f;
        float v_min = 1e9f, v_max = -1e9f;
        for (auto &pt : plane_robot->points)
        {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            u_min = std::min(u_min, p.dot(u_robot));
            u_max = std::max(u_max, p.dot(u_robot));
            v_min = std::min(v_min, p.dot(v_robot));
            v_max = std::max(v_max, p.dot(v_robot));
        }

        // ── 11. Rasterize → 2D occupancy grid ────────────────────────────────
        float grid_res = this->get_parameter("grid_res").as_double();
        int cols = static_cast<int>((u_max - u_min) / grid_res) + 1;
        int rows = static_cast<int>((v_max - v_min) / grid_res) + 1;
        if (cols <= 0 || rows <= 0 || cols > 2000 || rows > 2000)
            return;

        cv::Mat grid_img = cv::Mat::zeros(rows, cols, CV_8UC1);
        for (auto &pt : plane_robot->points)
        {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            int c = static_cast<int>((p.dot(u_robot) - u_min) / grid_res);
            int r = static_cast<int>((p.dot(v_robot) - v_min) / grid_res);
            if (c >= 0 && c < cols && r >= 0 && r < rows)
                grid_img.at<uint8_t>(r, c) = 255;
        }

        // ── 12. Morphological close ───────────────────────────────────────────
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::Mat closed;
        cv::morphologyEx(grid_img, closed, cv::MORPH_CLOSE, kernel);

        // ── 13. Invert → holes ────────────────────────────────────────────────
        cv::Mat inv;
        cv::bitwise_not(closed, inv);

        int border = static_cast<int>(0.1f / grid_res);
        cv::Mat mask = cv::Mat::zeros(rows, cols, CV_8UC1);
        if (rows > 2 * border && cols > 2 * border)
            mask(cv::Rect(border, border,
                          cols - 2 * border, rows - 2 * border)) = 255;

        cv::Mat holes;
        cv::bitwise_and(inv, mask, holes);

        // ── 14. Contours ──────────────────────────────────────────────────────
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(holes, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // ── 15. Filter → Opening list ─────────────────────────────────────────
        float min_area = this->get_parameter("min_open_area").as_double();
        float max_area = this->get_parameter("max_open_area").as_double();
        float min_w = this->get_parameter("min_open_w").as_double();
        float min_h = this->get_parameter("min_open_h").as_double();

        std::vector<Opening> openings;
        for (auto &cnt : contours)
        {
            cv::Rect bbox = cv::boundingRect(cnt);
            float w = (bbox.width + 1) * grid_res;
            float h = (bbox.height + 1) * grid_res + 0.1f;
            float area = w * h;

            if (area < min_area || area > max_area)
                continue;
            if (w < min_w || h < min_h)
                continue;

            float pu_center = u_min + (bbox.x + bbox.width * 0.5f) * grid_res;
            float pv_center = v_min + (bbox.y + bbox.height * 0.5f) * grid_res;
            float d_plane = -best_coeffs->values[3];

            Eigen::Vector3f c_robot =
                pu_center * u_robot + pv_center * v_robot + d_plane * n_robot;

            Eigen::Vector3f c_optical = robotToOptical(c_robot);

            Opening op;
            op.center_x = c_optical.x();
            op.center_y = c_optical.y();
            op.center_z = c_optical.z();
            op.width = w;
            op.height = h;
            op.confidence = std::min(1.0f, area / 2.0f);
            openings.push_back(op);

            // RCLCPP_INFO(this->get_logger(),
            //             "Opening: center_optical=(%.2f,%.2f,%.2f) size=%.2fx%.2fm",
            //             op.center_x, op.center_y, op.center_z, op.width, op.height);
        }

        if (!openings.empty())
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                 "Detection Status: Found %zu opening(s). Example size: %.2fx%.2fm",
                                 openings.size(), openings[0].width, openings[0].height);
        }
        else
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                 "Detection Status: Wall found, but no openings detected.");
        }

        // Normal back to optical frame for correct marker orientation
        Eigen::Vector3f n_optical = normalRobotToOptical(n_robot);

        // ── 16. Publish markers (optical frame) ───────────────────────────────
        publishMarkers(openings, msg->header, n_optical);
    }

    // -----------------------------------------------------------------------
    void publishMarkers(const std::vector<Opening> &openings,
                        const std_msgs::msg::Header &header,
                        const Eigen::Vector3f &normal)
    {
        visualization_msgs::msg::MarkerArray arr;

        visualization_msgs::msg::Marker del;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del);

        int id = 0;
        for (auto &op : openings)
        {
            // ── Box marker ──────────────────────────────────────────────────
            visualization_msgs::msg::Marker box;
            box.header = header;
            box.ns = "opening_box";
            box.id = id++;
            box.type = visualization_msgs::msg::Marker::CUBE;
            box.action = visualization_msgs::msg::Marker::ADD;

            box.pose.position.x = op.center_x;
            box.pose.position.y = op.center_y;
            box.pose.position.z = op.center_z;

            Eigen::Quaternionf q =
                Eigen::Quaternionf::FromTwoVectors(
                    Eigen::Vector3f(0, 0, 1), normal);
            box.pose.orientation.x = q.x();
            box.pose.orientation.y = q.y();
            box.pose.orientation.z = q.z();
            box.pose.orientation.w = q.w();

            box.scale.x = op.width;
            box.scale.y = op.height;
            box.scale.z = 0.02f;
            box.color.r = 0.0f;
            box.color.g = 1.0f;
            box.color.b = 0.3f;
            box.color.a = 0.4f;
            box.lifetime = rclcpp::Duration::from_seconds(0.5);
            arr.markers.push_back(box);

            // ── Text marker ─────────────────────────────────────────────────
            visualization_msgs::msg::Marker txt;
            txt.header = header;
            txt.ns = "opening_text";
            txt.id = id++;
            txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            txt.action = visualization_msgs::msg::Marker::ADD;

            txt.pose.position.x = op.center_x;
            txt.pose.position.y = op.center_y;
            txt.pose.position.z = op.center_z - 0.1f;
            txt.pose.orientation.w = 1.0;
            txt.scale.z = 0.15f;
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0f;

            char buf[64];
            snprintf(buf, sizeof(buf), "%.2fx%.2fm", op.width, op.height);
            txt.text = std::string(buf);
            txt.lifetime = rclcpp::Duration::from_seconds(0.5);
            arr.markers.push_back(txt);
        }

        marker_pub_->publish(arr);
    }

    // -----------------------------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr roi_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_cloud_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpeningDetector>());
    rclcpp::shutdown();
    return 0;
}