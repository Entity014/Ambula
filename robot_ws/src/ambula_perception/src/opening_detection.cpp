#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
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
#include <chrono>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

// ---------------------------------------------------------------------------
// PassState:
//   PASS    — opening width >= robot_width AND height >= robot_height
//             → robot passes standing upright              (GREEN)
//   CROUCH  — opening width >= robot_width AND
//             robot_height_min <= height < robot_height
//             → robot can crouch through                   (YELLOW)
//   BLOCKED — opening width < robot_width  OR
//             height < robot_height_min
//             → robot cannot pass                          (RED)
// ---------------------------------------------------------------------------
enum class PassState
{
    PASS,
    CROUCH,
    BLOCKED
};

struct Opening
{
    float center_x, center_y, center_z;
    float width, height;
    float confidence;
    PassState state;
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

// ---------------------------------------------------------------------------
class OpeningDetector : public rclcpp::Node
{
public:
    OpeningDetector() : Node("opening_detector")
    {
        this->declare_parameter("input_topic", "/camera/camera/depth/color/points");
        this->declare_parameter("voxel_size", 0.04f);

        // ROI in ROBOT frame (X=forward, Y=left, Z=up)
        this->declare_parameter("pass_x_min", 0.3f);
        this->declare_parameter("pass_x_max", 1.5f);
        this->declare_parameter("pass_y_min", -0.5f);
        this->declare_parameter("pass_y_max", 0.5f);
        this->declare_parameter("pass_z_min", -0.5f);
        this->declare_parameter("pass_z_max", 0.5f);

        this->declare_parameter("ransac_dist", 0.03f);
        this->declare_parameter("grid_res", 0.02f);
        this->declare_parameter("min_open_area", 0.1f);
        this->declare_parameter("max_open_area", 5.0f);
        this->declare_parameter("min_open_w", 0.1f);
        this->declare_parameter("min_open_h", 0.1f);
        this->declare_parameter("max_wall_tilt", 0.3f);
        this->declare_parameter("max_plane_iters", 5);
        this->declare_parameter("min_plane_inliers", 30);

        // ── Robot body dimensions ────────────────────────────────────────────
        this->declare_parameter("robot_width", 0.4f);
        this->declare_parameter("robot_height", 0.4f);
        this->declare_parameter("robot_height_min", 0.2f);
        this->declare_parameter("safety_margin", 0.05f);

        // ── Latch parameters (NEW) ───────────────────────────────────────────
        //  pass_confirm_frames  : consecutive PASS frames required to set latch
        //  pass_decay_frames    : frames of no-PASS before latch clears
        //  pass_latch_timeout   : hard timeout (seconds) to clear a stuck latch
        this->declare_parameter("pass_confirm_frames", 3);
        this->declare_parameter("pass_decay_frames", 5);
        this->declare_parameter("pass_latch_timeout", 8.0);

        // ── Topic names ──────────────────────────────────────────────────────
        this->declare_parameter("sub_robot_height", "/robot_body_height");
        this->declare_parameter("pub_markers", "/opening_detection/markers");
        this->declare_parameter("pub_roi_cloud", "/opening_detection/roi_cloud");
        this->declare_parameter("pub_plane_cloud", "/opening_detection/plane_cloud");
        this->declare_parameter("pub_estop", "/opening_detection/estop");
        this->declare_parameter("pub_crouch_height", "/opening_detection/crouch_height");
        this->declare_parameter("pub_wall_distance", "/opening_detection/wall_distance");

        // ── Latency logger ───────────────────────────────────────────────────
        this->declare_parameter("log_latency", false);
        this->declare_parameter("log_latency_throttle_ms", 1000);

        std::string topic = this->get_parameter("input_topic").as_string();
        std::string height_topic = this->get_parameter("sub_robot_height").as_string();

        current_robot_height_ = static_cast<float>(
            this->get_parameter("robot_height").as_double());

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(),
            std::bind(&OpeningDetector::cloudCallback, this, std::placeholders::_1));

        robot_height_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            height_topic, 10,
            [this](const std_msgs::msg::Float32::SharedPtr m)
            {
                current_robot_height_ = m->data;
            });

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->get_parameter("pub_markers").as_string(), 10);

        roi_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("pub_roi_cloud").as_string(), 10);

        plane_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("pub_plane_cloud").as_string(), 10);

        estop_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            this->get_parameter("pub_estop").as_string(), 10);

        crouch_height_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            this->get_parameter("pub_crouch_height").as_string(), 10);

        wall_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            this->get_parameter("pub_wall_distance").as_string(), 10);

        RCLCPP_INFO(this->get_logger(),
                    "OpeningDetector ready (no-TF)\n"
                    "  SUB  cloud        : %s\n"
                    "  SUB  robot_height : %s\n"
                    "  PUB  markers      : %s\n"
                    "  PUB  roi_cloud    : %s\n"
                    "  PUB  plane_cloud  : %s\n"
                    "  PUB  estop        : %s\n"
                    "  PUB  crouch_height: %s\n"
                    "  PUB  wall_distance: %s\n"
                    "  LATCH confirm=%d frames  decay=%d frames  timeout=%.1fs\n"
                    "  LOG  latency      : %s (throttle %d ms)",
                    topic.c_str(),
                    height_topic.c_str(),
                    this->get_parameter("pub_markers").as_string().c_str(),
                    this->get_parameter("pub_roi_cloud").as_string().c_str(),
                    this->get_parameter("pub_plane_cloud").as_string().c_str(),
                    this->get_parameter("pub_estop").as_string().c_str(),
                    this->get_parameter("pub_crouch_height").as_string().c_str(),
                    this->get_parameter("pub_wall_distance").as_string().c_str(),
                    this->get_parameter("pass_confirm_frames").as_int(),
                    this->get_parameter("pass_decay_frames").as_int(),
                    this->get_parameter("pass_latch_timeout").as_double(),
                    this->get_parameter("log_latency").as_bool() ? "ON" : "OFF",
                    this->get_parameter("log_latency_throttle_ms").as_int());
    }

private:
    // -----------------------------------------------------------------------
    // Latch state (replaces single bool latch_pass_)
    // -----------------------------------------------------------------------
    // pass_confirm_count_ : increments each frame we see a PASS opening,
    //                       decrements (floor 0) each frame we do NOT.
    //                       latch_pass_ is set when count >= pass_confirm_frames,
    //                       and cleared when count reaches 0 OR timeout fires.
    // latch_set_time_     : wall-clock time when latch was last SET (for timeout).
    // -----------------------------------------------------------------------
    int pass_confirm_count_{0};
    bool latch_pass_{false};
    rclcpp::Time latch_set_time_{0, 0, RCL_ROS_TIME};

    // -----------------------------------------------------------------------
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // ── Latency logger ───────────────────────────────────────────────────
        const auto t_recv = this->get_clock()->now();
        const rclcpp::Time t_msg(msg->header.stamp);
        const double latency_ms = (t_recv - t_msg).seconds() * 1000.0;
        const auto t_start = std::chrono::steady_clock::now();

        if (this->get_parameter("log_latency").as_bool())
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                                 this->get_parameter("log_latency_throttle_ms").as_int(),
                                 "[Latency] cloud msg → callback: %.2f ms", latency_ms);
        }

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

            pass.setInputCloud(cloud_robot);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(x_min, x_max);
            pass.filter(*tmp1);

            pass.setInputCloud(tmp1);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(y_min, y_max);
            pass.filter(*tmp2);

            pass.setInputCloud(tmp2);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(z_min, z_max);
            pass.filter(*filtered);
        }
        if (filtered->empty())
            return;

        // ── 4. Publish ROI cloud (convert back to optical frame) ─────────────
        {
            CloudT::Ptr roi_optical(new CloudT);
            roi_optical->reserve(filtered->size());
            for (const auto &p : filtered->points)
            {
                PointT o;
                o.x = -p.y;
                o.y = -p.z;
                o.z = p.x;
                roi_optical->push_back(o);
            }
            sensor_msgs::msg::PointCloud2 roi_msg;
            pcl::toROSMsg(*roi_optical, roi_msg);
            roi_msg.header = msg->header;
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

            if (std::abs(n_robot.z()) < max_tilt && std::abs(n_robot.x()) > 0.5f)
            {
                best_coeffs = c;
                best_inliers = idx;
                break;
            }

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
            // Wall gone from FOV → decay counter, may clear latch
            tickLatchNoPass("wall not visible");
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

        // ── 8. Publish plane cloud (optical frame) ────────────────────────────
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
        float robot_w = this->get_parameter("robot_width").as_double();
        float robot_h = current_robot_height_;
        float robot_h_min = this->get_parameter("robot_height_min").as_double();

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

            if (w < robot_w)
                op.state = PassState::BLOCKED;
            else if (h >= robot_h)
                op.state = PassState::PASS;
            else if (h >= robot_h_min)
                op.state = PassState::CROUCH;
            else
                op.state = PassState::BLOCKED;

            openings.push_back(op);
        }

        // ── Normal / wall center back to optical frame ────────────────────────
        Eigen::Vector3f n_optical = normalRobotToOptical(n_robot);

        float wall_w = u_max - u_min;
        float wall_h = v_max - v_min;
        float d_plane_val = -best_coeffs->values[3];

        Eigen::Vector3f wall_center_robot =
            ((u_min + u_max) * 0.5f) * u_robot +
            ((v_min + v_max) * 0.5f) * v_robot +
            d_plane_val * n_robot;
        Eigen::Vector3f wall_center_optical = robotToOptical(wall_center_robot);

        // ── Publish wall distance ─────────────────────────────────────────────
        {
            std_msgs::msg::Float32 dist_msg;
            dist_msg.data = wall_center_robot.x();
            wall_distance_pub_->publish(dist_msg);
        }

        // ── Latch-clear: robot passed through wall ────────────────────────────
        if (latch_pass_ && wall_center_robot.x() < x_min)
        {
            clearLatch("wall dist %.3fm < pass_x_min %.3fm (passed through)",
                       wall_center_robot.x(), x_min);
        }

        // ── Opening state + size logger ───────────────────────────────────────
        if (!openings.empty())
        {
            for (size_t i = 0; i < openings.size(); ++i)
            {
                const auto &op = openings[i];
                const char *state_str = stateStr(op.state);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "[Opening #%zu] state=%-7s  w=%.3f m  h=%.3f m",
                                     i, state_str, op.width, op.height);
            }
        }
        else
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "[Opening] SOLID WALL — no openings detected  "
                                 "wall: %.3fw x %.3fh m",
                                 wall_w, wall_h);
        }

        // ── 16. Publish markers ───────────────────────────────────────────────
        publishMarkers(openings, msg->header, n_optical,
                       wall_center_optical, wall_w, wall_h);

        // ── 17. Teleop control outputs ────────────────────────────────────────
        publishTeleopOutputs(openings, robot_h_min);

        // ── Process latency logger ────────────────────────────────────────────
        if (this->get_parameter("log_latency").as_bool())
        {
            const auto t_end = std::chrono::steady_clock::now();
            const double proc_ms = std::chrono::duration<double, std::milli>(
                                       t_end - t_start)
                                       .count();

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                                 this->get_parameter("log_latency_throttle_ms").as_int(),
                                 "[Latency] msg→cb: %.2f ms | process: %.2f ms | total: %.2f ms",
                                 latency_ms, proc_ms, latency_ms + proc_ms);
        }
    }

    // -----------------------------------------------------------------------
    // Latch helpers
    // -----------------------------------------------------------------------

    static const char *stateStr(PassState s)
    {
        switch (s)
        {
        case PassState::PASS:
            return "PASS";
        case PassState::CROUCH:
            return "CROUCH";
        case PassState::BLOCKED:
            return "BLOCKED";
        default:
            return "UNKNOWN";
        }
    }

    // Call each frame where we saw at least one PASS opening.
    void tickLatchPass()
    {
        const int confirm_thresh = this->get_parameter("pass_confirm_frames").as_int();

        pass_confirm_count_++;
        RCLCPP_DEBUG(this->get_logger(),
                     "[Latch] PASS tick: count=%d / %d",
                     pass_confirm_count_, confirm_thresh);

        if (!latch_pass_ && pass_confirm_count_ >= confirm_thresh)
        {
            latch_pass_ = true;
            latch_set_time_ = this->get_clock()->now();
            RCLCPP_INFO(this->get_logger(),
                        "[Latch] SET — confirmed PASS after %d consecutive frames",
                        pass_confirm_count_);
        }
    }

    // Call each frame where NO PASS opening was seen (wall lost or no opening).
    // reason is used only for the log message.
    void tickLatchNoPass(const char *reason)
    {
        const int decay_frames = this->get_parameter("pass_decay_frames").as_int();
        const double timeout = this->get_parameter("pass_latch_timeout").as_double();

        // Decay the confirmation counter
        if (pass_confirm_count_ > 0)
            pass_confirm_count_--;

        // Hard timeout check (even if count hasn't reached 0 yet)
        if (latch_pass_)
        {
            const double age = (this->get_clock()->now() - latch_set_time_).seconds();
            if (age > timeout)
            {
                clearLatch("timeout %.1fs elapsed (%s)", age, reason);
                return;
            }
        }

        // Counter-based clear
        if (latch_pass_ && pass_confirm_count_ == 0)
        {
            clearLatch("count decayed to 0 after %d no-PASS frames (%s)",
                       decay_frames, reason);
        }
    }

    template <typename... Args>
    void clearLatch(const char *fmt, Args... args)
    {
        latch_pass_ = false;
        pass_confirm_count_ = 0;
        char buf[256];
        snprintf(buf, sizeof(buf), fmt, args...);
        RCLCPP_INFO(this->get_logger(), "[Latch] CLEARED — %s", buf);
    }

    // -----------------------------------------------------------------------
    void publishTeleopOutputs(const std::vector<Opening> &openings,
                              float robot_h_min)
    {
        float safety_margin = this->get_parameter("safety_margin").as_double();

        const Opening *best_pass = nullptr;
        const Opening *best_crouch = nullptr;

        for (const auto &op : openings)
        {
            float area = op.width * op.height;
            if (op.state == PassState::PASS)
            {
                if (!best_pass || area > best_pass->width * best_pass->height)
                    best_pass = &op;
            }
            else if (op.state == PassState::CROUCH)
            {
                if (!best_crouch || area > best_crouch->width * best_crouch->height)
                    best_crouch = &op;
            }
        }

        // ── Advance latch state machine ───────────────────────────────────────
        if (best_pass)
            tickLatchPass();
        else
            tickLatchNoPass(best_crouch ? "best is CROUCH" : "no valid opening");

        // ── Publish estop / crouch_height ─────────────────────────────────────
        std_msgs::msg::Bool estop_msg;

        if (latch_pass_)
        {
            estop_msg.data = false;
            estop_pub_->publish(estop_msg);
            RCLCPP_DEBUG(this->get_logger(),
                         "[Latch] active — GO (count=%d, best_pass=%s)",
                         pass_confirm_count_, best_pass ? "yes" : "no (partial view)");
        }
        else if (best_crouch)
        {
            estop_msg.data = true;
            estop_pub_->publish(estop_msg);

            float target_h = best_crouch->height - safety_margin;
            target_h = std::max(target_h, robot_h_min);

            std_msgs::msg::Float32 ch_msg;
            ch_msg.data = target_h;
            crouch_height_pub_->publish(ch_msg);

            RCLCPP_DEBUG(this->get_logger(),
                         "STOP(CROUCH) opening=%.3fm  margin=%.3fm  "
                         "target=%.3fm  min=%.3fm",
                         best_crouch->height, safety_margin,
                         target_h, robot_h_min);
        }
        else
        {
            estop_msg.data = true;
            estop_pub_->publish(estop_msg);
        }
    }

    // -----------------------------------------------------------------------
    void publishMarkers(const std::vector<Opening> &openings,
                        const std_msgs::msg::Header &header,
                        const Eigen::Vector3f &normal,
                        const Eigen::Vector3f &wall_center,
                        float wall_w,
                        float wall_h)
    {
        visualization_msgs::msg::MarkerArray arr;

        visualization_msgs::msg::Marker del;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del);

        Eigen::Quaternionf q_wall =
            Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0, 0, 1), normal);

        if (openings.empty())
        {
            visualization_msgs::msg::Marker wall_box;
            wall_box.header = header;
            wall_box.ns = "solid_wall";
            wall_box.id = 0;
            wall_box.type = visualization_msgs::msg::Marker::CUBE;
            wall_box.action = visualization_msgs::msg::Marker::ADD;

            wall_box.pose.position.x = wall_center.x();
            wall_box.pose.position.y = wall_center.y();
            wall_box.pose.position.z = wall_center.z();
            wall_box.pose.orientation.x = q_wall.x();
            wall_box.pose.orientation.y = q_wall.y();
            wall_box.pose.orientation.z = q_wall.z();
            wall_box.pose.orientation.w = q_wall.w();

            wall_box.scale.x = std::max(wall_w, 0.1f);
            wall_box.scale.y = std::max(wall_h, 0.1f);
            wall_box.scale.z = 0.02f;

            wall_box.color.r = 1.0f;
            wall_box.color.g = 0.0f;
            wall_box.color.b = 0.0f;
            wall_box.color.a = 0.35f;
            wall_box.lifetime = rclcpp::Duration::from_seconds(0.5);
            arr.markers.push_back(wall_box);

            visualization_msgs::msg::Marker wall_txt;
            wall_txt.header = header;
            wall_txt.ns = "solid_wall_text";
            wall_txt.id = 1;
            wall_txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            wall_txt.action = visualization_msgs::msg::Marker::ADD;

            wall_txt.pose.position.x = wall_center.x();
            wall_txt.pose.position.y = wall_center.y();
            wall_txt.pose.position.z = wall_center.z() - 0.1f;
            wall_txt.pose.orientation.w = 1.0;
            wall_txt.scale.z = 0.15f;

            wall_txt.color.r = 1.0f;
            wall_txt.color.g = 0.3f;
            wall_txt.color.b = 0.3f;
            wall_txt.color.a = 1.0f;
            wall_txt.text = "[SOLID WALL]";
            wall_txt.lifetime = rclcpp::Duration::from_seconds(0.5);
            arr.markers.push_back(wall_txt);

            marker_pub_->publish(arr);
            return;
        }

        int id = 0;
        for (const auto &op : openings)
        {
            visualization_msgs::msg::Marker box;
            box.header = header;
            box.ns = "opening_box";
            box.id = id++;
            box.type = visualization_msgs::msg::Marker::CUBE;
            box.action = visualization_msgs::msg::Marker::ADD;

            box.pose.position.x = op.center_x;
            box.pose.position.y = op.center_y;
            box.pose.position.z = op.center_z;
            box.pose.orientation.x = q_wall.x();
            box.pose.orientation.y = q_wall.y();
            box.pose.orientation.z = q_wall.z();
            box.pose.orientation.w = q_wall.w();

            box.scale.x = op.width;
            box.scale.y = op.height;
            box.scale.z = 0.02f;

            switch (op.state)
            {
            case PassState::PASS:
                box.color.r = 0.0f;
                box.color.g = 1.0f;
                box.color.b = 0.3f;
                break;
            case PassState::CROUCH:
                box.color.r = 1.0f;
                box.color.g = 0.85f;
                box.color.b = 0.0f;
                break;
            case PassState::BLOCKED:
            default:
                box.color.r = 1.0f;
                box.color.g = 0.0f;
                box.color.b = 0.0f;
                break;
            }
            box.color.a = 0.4f;
            box.lifetime = rclcpp::Duration::from_seconds(0.5);
            arr.markers.push_back(box);

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

            const char *label = nullptr;
            switch (op.state)
            {
            case PassState::PASS:
                txt.color.r = 0.2f;
                txt.color.g = 1.0f;
                txt.color.b = 0.4f;
                label = "[PASS]";
                break;
            case PassState::CROUCH:
                txt.color.r = 1.0f;
                txt.color.g = 0.9f;
                txt.color.b = 0.0f;
                label = "[CROUCH]";
                break;
            case PassState::BLOCKED:
            default:
                txt.color.r = 1.0f;
                txt.color.g = 0.3f;
                txt.color.b = 0.3f;
                label = "[BLOCKED]";
                break;
            }
            txt.color.a = 1.0f;

            char buf[80];
            snprintf(buf, sizeof(buf), "%.2fx%.2fm %s", op.width, op.height, label);
            txt.text = std::string(buf);
            txt.lifetime = rclcpp::Duration::from_seconds(0.5);
            arr.markers.push_back(txt);
        }

        marker_pub_->publish(arr);
    }

    // -----------------------------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr robot_height_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr roi_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr crouch_height_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wall_distance_pub_;

    float current_robot_height_{0.4f};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpeningDetector>());
    rclcpp::shutdown();
    return 0;
}