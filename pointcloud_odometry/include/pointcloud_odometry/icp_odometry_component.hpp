#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/registration/registration_helper.hpp>

#include <deque>
#include <mutex>
#include <string>
#include <vector>
#include <memory>

namespace pointcloud_odometry {

struct OdomSample {
    double timestamp;
    Eigen::Isometry3d pose;
};

enum class TrackingState {
    TRACKING = 0,
    LOST = 1
};

class IcpOdometryComponent : public rclcpp::Node {
public:
    explicit IcpOdometryComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~IcpOdometryComponent();

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void initialGuessCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

    bool computeInitialGuess(double current_stamp, Eigen::Isometry3d& guess_pose);
    void handleSaveMap(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    bool saveMapToFile(const std::string& filepath, size_t* out_saved_points = nullptr);
    std::vector<Eigen::Vector3f> applySORFilter(
        const std::vector<Eigen::Vector3f>& input_points,
        int k_neighbors = 16,
        double std_ratio = 1.2,
        int num_threads = 4);

    void publishOdometry(const rclcpp::Time& stamp,
                         const Eigen::Isometry3d& pose,
                         const Eigen::Matrix<double, 6, 6>& H,
                         const Eigen::Vector3d& linear_vel,
                         const Eigen::Vector3d& angular_vel,
                         bool measurement_valid = true,
                         double residual_scale = 1.0,
                         double dt = 0.033);

    void publishSubmapCloud(const rclcpp::Time& stamp);

    // ROS 2 Communication
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_initial_guess_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_telemetry_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap_cloud_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_map_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_submap_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_map_alias_;

    // TF Broadcaster & Listener
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Configuration Parameters
    std::string input_cloud_topic_;
    std::string odom_topic_;
    std::string initial_guess_topic_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    bool publish_tf_;
    bool use_tf_extrinsic_;
    double voxel_resolution_;
    double min_pos_variance_;
    double min_rot_variance_;
    double lost_variance_;
    double keyframe_delta_trans_;
    double keyframe_delta_rot_rad_;
    int num_threads_;
    int max_iterations_;
    double max_corr_dist_;
    std::string default_map_path_;
    bool save_on_shutdown_;
    bool publish_submap_cloud_{true};
    std::string submap_cloud_topic_{"/icp/submap_cloud"};
    double submap_publish_interval_sec_{1.0};
    rclcpp::Time last_submap_pub_time_{0, 0, RCL_ROS_TIME};

    // Initial Guess Buffer
    std::mutex odom_mutex_;
    std::deque<OdomSample> initial_guess_buffer_;
    double last_cloud_stamp_{0.0};

    // Submap & Registration State
    std::shared_ptr<small_gicp::GaussianVoxelMap> submap_voxelmap_;
    Eigen::Isometry3d current_pose_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d last_pose_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d last_keyframe_pose_{Eigen::Isometry3d::Identity()};
    bool is_first_frame_{true};
    size_t keyframe_count_{0};

    // Tracking state & relocalization recovery
    TrackingState tracking_state_{TrackingState::TRACKING};
    int lost_frames_counter_{0};
    int relocalize_attempt_counter_{0};

    // Accumulated Submap Points for Paper Figures
    std::mutex map_export_mutex_;
    std::vector<Eigen::Vector3f> accumulated_map_points_;

    // Real throughput monitoring
    int throughput_frame_count_{0};
    std::chrono::steady_clock::time_point last_throughput_time_{std::chrono::steady_clock::now()};
    double real_throughput_hz_{30.0};
};

} // namespace pointcloud_odometry
