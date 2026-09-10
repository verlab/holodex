from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    astra_camera_dir = get_package_share_directory('astra_camera')
    launch_path = os.path.join(astra_camera_dir, 'launch', 'astra_pro.launch.xml')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_path),
            launch_arguments={
                'uvc_vendor_id': '11205',
                'uvc_product_id': '1281',
                'enable_point_cloud': 'true',
            }.items()
        )
    ])
