import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_wheel_odometry')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    odom_node = Node(
        package='mecanum_wheel_odometry',
        executable='mecanum_wheel_odometry_node',
        name='mecanum_wheel_odometry_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        odom_node
    ])
