import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile

def generate_launch_description():
    pkg_share = get_package_share_directory('holodex_bringup')
    
    # Arquivos de configuração padrão
    default_nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_supervisor_params = os.path.join(pkg_share, 'config', 'supervisor.yaml')

    # Declaração de Argumentos
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params,
        description='Caminho completo para o arquivo de parâmetros do Nav2'
    )
    supervisor_params_arg = DeclareLaunchArgument(
        'supervisor_config',
        default_value=default_supervisor_params,
        description='Caminho completo para os parâmetros do Motion Supervisor'
    )
    launch_supervisor_arg = DeclareLaunchArgument(
        'launch_supervisor',
        default_value='false',
        description='Lança também o motion supervisor (deixe false se já executado pelo orquestrador)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Utiliza sim_time (Clock simulado)'
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Inicia os nós gerenciados pelo ciclo de vida automaticamente'
    )

    params_file = LaunchConfiguration('params_file')
    supervisor_config = LaunchConfiguration('supervisor_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Nós de Ciclo de Vida do Nav2
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    # Remapeamentos: A saída de velocidade do Nav2 é direcionada a /cmd_vel_nav
    # para ser interceptada e validada pelo supervisor de movimento
    nav2_remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/cmd_vel', '/cmd_vel_nav')
    ]

    # 1. Controller Server (DWB Holonômico Mecanum)
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=nav2_remappings
    )

    # 2. Smoother Server
    smoother_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file],
        remappings=nav2_remappings
    )

    # 3. Planner Server (SmacPlanner2D)
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
        remappings=nav2_remappings
    )

    # 4. Behavior Server (Recuperações)
    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=nav2_remappings
    )

    # 5. BT Navigator (Árvores de Comportamento de Navegação)
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
        remappings=nav2_remappings
    )

    # 6. Waypoint Follower
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file],
        remappings=nav2_remappings
    )

    # 7. Gerenciador de Ciclo de Vida (Lifecycle Manager)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes
        }]
    )

    # 8. Supervisor de Movimento (Camada Intermediária de Segurança Holonômica)
    # Assina /cmd_vel_nav e publica /cmd_vel real para a ESP32
    motion_supervisor_node = Node(
        condition=IfCondition(LaunchConfiguration('launch_supervisor')),
        package='holodex_bringup',
        executable='motion_supervisor',
        name='holodex_motion_supervisor',
        output='screen',
        parameters=[supervisor_config]
    )

    return LaunchDescription([
        params_file_arg,
        supervisor_params_arg,
        launch_supervisor_arg,
        use_sim_time_arg,
        autostart_arg,
        controller_node,
        smoother_node,
        planner_node,
        behavior_node,
        bt_navigator_node,
        waypoint_follower_node,
        lifecycle_manager_node,
        motion_supervisor_node
    ])
