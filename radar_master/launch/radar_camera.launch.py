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
    
    from common import get_camera_node,launch_params
    
    #######################################################################
    hik_camera_node_1 = get_camera_node('hik_camera', 'hik_camera::HikCameraNode', "camera_1_node")
    hik_camera_node_2 = get_camera_node('hik_camera', 'hik_camera::HikCameraNode', "camera_2_node")
    
    def get_radar_cv_container(camera_node_1,camera_node_2):
        return ComposableNodeContainer(
        name='radar_cv_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            camera_node_1,
            camera_node_2,
        ],
        output='screen',
        ros_arguments=['--ros-args', '--log-level',
                       'radar_robot_detector_node:=' + launch_params['detector_log_level']
        ],
    )
    
    radar_cv_module = get_radar_cv_container(hik_camera_node_1,hik_camera_node_2)
    return LaunchDescription(
        [
            radar_cv_module,
            #radar_ui_module
        ]
    )

