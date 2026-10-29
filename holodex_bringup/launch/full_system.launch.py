import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('holodex_bringup')

    # Argumentos
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Habilita stack Nav2 e supervisor de movimento'
    )
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Habilita câmera Orbbec Astra Pro'
    )
    use_esp32_arg = DeclareLaunchArgument(
        'use_esp32',
        default_value='true',
        description='Habilita agente micro-ROS da base'
    )

    # 1. Base Hardware & Percepção
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_camera': LaunchConfiguration('use_camera'),
            'use_esp32': LaunchConfiguration('use_esp32'),
            'use_icp_odom': 'true'
        }.items()
    )

    # 2. Navegação Autônoma & Supervisor
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_nav2'))
    )

    return LaunchDescription([
        use_nav2_arg,
        use_camera_arg,
        use_esp32_arg,
        robot_launch,
        navigation_launch
    ])
