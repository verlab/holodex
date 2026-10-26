import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_wheel_odometry')
    wheel_params = os.path.join(pkg_share, 'config', 'params.yaml')
    madgwick_params = os.path.join(pkg_share, 'config', 'madgwick.yaml')
    ekf_params = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # 1. Nó de Odometria Mecanum 4WD (calcula /odom/wheel sem TF)
    wheel_odom_node = Node(
        package='mecanum_wheel_odometry',
        executable='mecanum_wheel_odometry_node',
        name='mecanum_wheel_odometry_node',
        output='screen',
        parameters=[wheel_params]
    )

    # 2. Filtro Madgwick para IMU (converte /imu/data_raw -> /imu/data com quaternion)
    madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        output='screen',
        parameters=[madgwick_params],
        remappings=[
            ('imu/data_raw', '/imu/data_raw'),
            ('imu/data', '/imu/data')
        ]
    )

    # 3. EKF (Robot Localization: funde rodas + IMU e gera TF odom -> base_link)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ]
    )

    # 4. TF estático base_link -> imu_link (o EKF precisa dele para usar /imu/data).
    # Ajuste x y z yaw pitch roll conforme a posição real do BMI160 no chassi.
    imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_static_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link']
    )

    return LaunchDescription([
        imu_tf_node,
        wheel_odom_node,
        madgwick_node,
        ekf_node
    ])
