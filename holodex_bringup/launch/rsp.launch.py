import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('holodex_bringup')
    urdf_path = os.path.join(pkg_share, 'urdf', 'holodex.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'publish_frequency': 30.0
        }]
    )

    return LaunchDescription([
        rsp_node
    ])
