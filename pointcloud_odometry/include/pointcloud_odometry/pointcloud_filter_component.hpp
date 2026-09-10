#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include <mutex>
#include <string>
#include <vector>
#include <memory>

namespace pointcloud_odometry {

class PointcloudFilterComponent : public rclcpp::Node {
public:
    explicit PointcloudFilterComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~PointcloudFilterComponent() = default;

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void maskCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    bool isDynamicPixel(uint32_t v, uint32_t u, const builtin_interfaces::msg::Time& stamp);

    // ROS 2 Communication
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_mask_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_cloud_;

    // Parameters
    std::string input_cloud_topic_;
    std::string output_cloud_topic_;
    std::string mask_topic_;
    int stride_u_;
    int stride_v_;
    double min_range_;
    double max_range_;
    bool enable_dynamic_filter_;
    double mask_timeout_sec_;

    // Dynamic Mask Buffer
    std::mutex mask_mutex_;
    cv::Mat latest_mask_;
    builtin_interfaces::msg::Time latest_mask_stamp_;
    bool has_mask_{false};

    // Pre-allocated point buffer for efficiency
    std::vector<float> point_buffer_;
};

} // namespace pointcloud_odometry
