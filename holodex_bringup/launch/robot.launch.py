import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Argumentos configuráveis
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Porta serial USB da ESP32'
    )
    use_esp32_arg = DeclareLaunchArgument(
        'use_esp32',
        default_value='true',
        description='Habilita nó micro-ROS agent'
    )
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Habilita driver da câmera Orbbec Astra Pro'
    )
    use_icp_odom_arg = DeclareLaunchArgument(
        'use_icp_odom',
        default_value='true',
        description='Habilita odometria visual 3D small_gicp'
    )

    serial_port = LaunchConfiguration('serial_port')
    use_esp32 = LaunchConfiguration('use_esp32')
    use_camera = LaunchConfiguration('use_camera')
    use_icp_odom = LaunchConfiguration('use_icp_odom')

    # 1. micro-ROS Agent Node
    micro_ros_node = Node(
        condition=IfCondition(use_esp32),
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', serial_port, '-b', '921600']
    )

    # 2. Base Fusion (Odometria de Rodas Mecanum + Madgwick + EKF)
    base_fusion_pkg = get_package_share_directory('mecanum_wheel_odometry')
    base_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_fusion_pkg, 'launch', 'base_fusion.launch.py')
        )
    )

    # 3. Robot State Publisher (URDF: base_footprint -> base_link -> camera_link, imu_link, rodas)
    bringup_pkg = get_package_share_directory('holodex_bringup')
    urdf_file = os.path.join(bringup_pkg, 'urdf', 'holodex.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0
        }]
    )

    # 4. Odometria Visual 3D (small_gicp VGICP + Filtro Striding)
    pointcloud_pkg = get_package_share_directory('pointcloud_odometry')
    pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pointcloud_pkg, 'launch', 'pointcloud_odometry.launch.py')
        ),
        condition=IfCondition(use_icp_odom)
    )

    # 4. Câmera Orbbec Astra Pro
    try:
        astra_pkg = get_package_share_directory('astra_pro_bringup')
        astra_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(astra_pkg, 'launch', 'astra_pro.launch.py')
            ),
            condition=IfCondition(use_camera)
        )
        camera_actions = [astra_launch]
    except Exception:
        camera_actions = []

    return LaunchDescription([
        serial_port_arg,
        use_esp32_arg,
        use_camera_arg,
        use_icp_odom_arg,
        rsp_node,
        micro_ros_node,
        base_fusion_launch,
        pointcloud_launch,
        *camera_actions
    ])
