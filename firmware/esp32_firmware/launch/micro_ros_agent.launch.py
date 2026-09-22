from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Dispositivo serial USB do ESP32 (ex: /dev/ttyUSB0 ou /dev/esp32)'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='921600',
        description='Baudrate da comunicação serial micro-ROS'
    )

    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=[
            'serial',
            '--dev', LaunchConfiguration('serial_port'),
            '-b', LaunchConfiguration('baudrate')
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        micro_ros_agent_node
    ])
