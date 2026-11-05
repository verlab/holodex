import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share = get_package_share_directory('pointcloud_odometry')
    default_params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Launch Arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Caminho completo para o arquivo YAML de parametros'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='false',
        description='Se true, publica TF odom -> base_link (Mantenha false para evitar colisão com EKF)'
    )

    enable_dynamic_filter_arg = DeclareLaunchArgument(
        'enable_dynamic_filter',
        default_value='false',
        description='Se true, assina máscara do YOLO para descartar pontos dinâmicos'
    )

    initial_guess_topic_arg = DeclareLaunchArgument(
        'initial_guess_topic',
        default_value='/odom/wheel',
        description='Odometria de rodas (independente do ICP) para chute inicial. Nao use a saida do EKF.'
    )

    # Zero-Copy Composable Node Container
    container = ComposableNodeContainer(
        name='odometry_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt', # Multithreaded container
        composable_node_descriptions=[
            ComposableNode(
                package='pointcloud_odometry',
                plugin='pointcloud_odometry::PointcloudFilterComponent',
                name='pointcloud_filter_component',
                parameters=[
                    LaunchConfiguration('params_file'),
                    {'enable_dynamic_filter': LaunchConfiguration('enable_dynamic_filter')}
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='pointcloud_odometry',
                plugin='pointcloud_odometry::IcpOdometryComponent',
                name='icp_odometry_component',
                parameters=[
                    LaunchConfiguration('params_file'),
                    {
                        'publish_tf': LaunchConfiguration('publish_tf'),
                        'initial_guess_topic': LaunchConfiguration('initial_guess_topic')
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        params_file_arg,
        publish_tf_arg,
        enable_dynamic_filter_arg,
        initial_guess_topic_arg,
        container
    ])
