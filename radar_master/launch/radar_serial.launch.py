import os
import sys

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer 

from launch.actions import TimerAction

from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('radar_master'), 'launch'))


def generate_launch_description():
    
    from common import get_serial_node

    #######################################################################
        
    serial_node = get_serial_node('radar_serial', 'radar_serial_driver::RadarSerialDriver')
    
    def container(serial_node):
        return ComposableNodeContainer(
            name='serial_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                serial_node
            ],
            output='screen'
        )
        
    container_module = container(serial_node)
    

    return LaunchDescription(
        [
            container_module
        ]
    )

