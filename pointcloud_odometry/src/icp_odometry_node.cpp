#include "pointcloud_odometry/icp_odometry_component.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pointcloud_odometry::IcpOdometryComponent>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
