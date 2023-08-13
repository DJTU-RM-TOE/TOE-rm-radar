import os
import sys

from launch_ros.actions import ComposableNodeContainer 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch.actions import TimerAction

sys.path.append(os.path.join(get_package_share_directory('radar_master'), 'launch'))

from common import launch_params

def get_radar_log_container(vidio_log_node):
    return ComposableNodeContainer(
        name='radar_cv_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            vidio_log_node
        ],
        output='screen',
    )
    
def generate_launch_description():
    from common import get_vidio_log_node
    
    vidio_log_node = get_vidio_log_node('radar_master', 'ImageSubscriber','vidio_log_node')
    
    radar_log_module = get_radar_log_container(vidio_log_node)
    return LaunchDescription(
        [
            radar_log_module
        ]
    )