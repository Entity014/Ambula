// src/stair_perception.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>                      // removeNaNFromPointCloud
#include <pcl/filters/passthrough.h>                 // PassThrough
#include <pcl/filters/statistical_outlier_removal.h> // SOR
#include <pcl/common/transforms.h>                   // transformPointCloud
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>

// tf2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

#include <cmath>
#include <string>
#include <vector>

class StairPerception : public rclcpp::Node
{
public:
    StairPerception() : Node("stair_perception_node"),
                        tf_buffer_(this->get_clock()),
                        tf_listener_(tf_buffer_)
    {
        topic_in_ = declare_parameter<std::string>("pointcloud_topic_in", "/depth_camera/points");
        topic_out_ = declare_parameter<std::string>("pointcloud_topic_out", "/depth_camera/points_filtered");
        voxel_size_ = declare_parameter<double>("voxel_size", 0.03); // 3 cm

        // Z-ROI (meters) in target_frame (e.g., base_link)
        z_min_ = declare_parameter<double>("z_min", 0.05);
        z_max_ = declare_parameter<double>("z_max", 3.00);

        // Outlier removal
        use_sor_ = declare_parameter<bool>("use_sor", true);
        sor_mean_k_ = declare_parameter<int>("sor_mean_k", 20);
        sor_std_mul_ = declare_parameter<double>("sor_std_mul", 1.0);

        // frame that defines the filtering axis (usually "base_link")
        target_frame_ = declare_parameter<std::string>("target_frame", "base_link");

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_in_, 10,
            std::bind(&StairPerception::cloudCallback, this, std::placeholders::_1));

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_out_, 10);

        RCLCPP_INFO(get_logger(), "Listening on: %s", topic_in_.c_str());
        RCLCPP_INFO(get_logger(), "Publishing filtered cloud to: %s", topic_out_.c_str());
        RCLCPP_INFO(get_logger(),
                    "Pipeline: NaN->Voxel(src)->TF(%s)->SOR(%s)->PassThrough(x,y,z) | voxel=%.3f m | "
                    "z=[%.2f,%.2f] | SOR(k=%d,std=%.2f)",
                    target_frame_.c_str(),
                    use_sor_ ? "on" : "off",
                    voxel_size_,
                    z_min_, z_max_,
                    sor_mean_k_, sor_std_mul_);
    }

private:
    using PointT = pcl::PointXYZ;

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const std::string source_frame = msg->header.frame_id;

        // 0) ROS2 -> PCL
        pcl::PointCloud<PointT>::Ptr cloud_raw(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg, *cloud_raw);

        // 0.1) Remove NaN
        pcl::PointCloud<PointT>::Ptr cloud_clean(new pcl::PointCloud<PointT>);
        std::vector<int> idx;
        pcl::removeNaNFromPointCloud(*cloud_raw, *cloud_clean, idx);

        // 0.2) Downsample (source frame)
        pcl::PointCloud<PointT>::Ptr cloud_ds_src(new pcl::PointCloud<PointT>);
        {
            pcl::VoxelGrid<PointT> vg;
            vg.setInputCloud(cloud_clean);
            vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            vg.filter(*cloud_ds_src);
        }

        // 0.3) Transform ds cloud -> target_frame (e.g., base_link)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_target(new pcl::PointCloud<pcl::PointXYZ>);
        try
        {
            geometry_msgs::msg::TransformStamped tf =
                tf_buffer_.lookupTransform(target_frame_, source_frame, msg->header.stamp);

            Eigen::Isometry3d iso = tf2::transformToEigen(tf.transform);
            Eigen::Matrix4f tf_mat = iso.matrix().cast<float>();
            pcl::transformPointCloud(*cloud_ds_src, *cloud_in_target, tf_mat);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "TF lookup failed (%s -> %s): %s",
                                 source_frame.c_str(), target_frame_.c_str(), ex.what());
            return;
        }

        // 0.4) Statistical Outlier Removal (in target frame)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_den(new pcl::PointCloud<pcl::PointXYZ>);
        if (use_sor_)
        {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_in_target);
            sor.setMeanK(sor_mean_k_);
            sor.setStddevMulThresh(sor_std_mul_);
            sor.filter(*cloud_den);
        }
        else
        {
            cloud_den = cloud_in_target;
        }

        // 0.5) PassThrough on Z (in target_frame)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
        {
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud_den);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(z_min_, z_max_);
            pass.filter(*cloud_roi);
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "[%s] In:%u -> NaN-free:%u -> Voxel(src):%u -> TF->%s:%u -> SOR:%u -> ROI(z):%u",
            source_frame.c_str(),
            static_cast<unsigned>(cloud_raw->size()),
            static_cast<unsigned>(cloud_clean->size()),
            static_cast<unsigned>(cloud_ds_src->size()),
            target_frame_.c_str(),
            static_cast<unsigned>(cloud_in_target->size()),
            static_cast<unsigned>(cloud_den->size()),
            static_cast<unsigned>(cloud_roi->size()));

        // publish (cloud is in target_frame)
        sensor_msgs::msg::PointCloud2 cloud_out;
        pcl::toROSMsg(*cloud_roi, cloud_out);
        cloud_out.header = msg->header;            // keep original timestamp
        cloud_out.header.frame_id = target_frame_; // but now in target frame
        pub_->publish(cloud_out);
    }

    // params
    std::string topic_in_, topic_out_, target_frame_;
    double voxel_size_;
    double z_min_, z_max_;
    bool use_sor_;
    int sor_mean_k_;
    double sor_std_mul_;

    // ros
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StairPerception>());
    rclcpp::shutdown();
    return 0;
}
