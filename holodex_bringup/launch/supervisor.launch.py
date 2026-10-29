import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('holodex_bringup')
    default_config = os.path.join(pkg_share, 'config', 'supervisor.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Caminho para o arquivo YAML de configuração do motion supervisor'
    )

    supervisor_node = Node(
        package='holodex_bringup',
        executable='motion_supervisor',
        name='holodex_motion_supervisor',
        output='screen',
        parameters=[LaunchConfiguration('config')]
    )

    return LaunchDescription([
        config_arg,
        supervisor_node
    ])
