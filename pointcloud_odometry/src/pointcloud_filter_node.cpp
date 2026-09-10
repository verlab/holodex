#include "pointcloud_odometry/pointcloud_filter_component.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pointcloud_odometry::PointcloudFilterComponent>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
