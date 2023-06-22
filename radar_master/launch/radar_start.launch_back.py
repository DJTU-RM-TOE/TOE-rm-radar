import os
import yaml

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer 
from launch.actions import TimerAction


def generate_launch_description():
    
    
    launch_params = yaml.safe_load(
        open(
            os.path.join(
                get_package_share_directory("radar_master"),
                "config",
                "launch_params.yaml",
            )
        )
    )
    
    node_params = os.path.join(
        get_package_share_directory("radar_master"), "config", "node_params.yaml"
    )   

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name="camera_node",
            parameters=[node_params],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    
    def get_radar_robot_detector_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name="radar_robot_detector_node",
            parameters=[node_params],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        
    def get_radar_cv_container(camera_node):
        return ComposableNodeContainer(
            name='radar_cv_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                radar_robot_detector_node,
                camera_node
            ],
            output='screen',
            ros_arguments=['--ros-args', '--log-level',
                           'radar_robot_detector_node:=' + launch_params['detector_log_level']
            ],
        )
        
    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
    radar_robot_detector_node = get_radar_robot_detector_node('radar_robot_detector', 'radar_detector::detector_node')
    
    radar_cv_module = get_radar_cv_container(hik_camera_node)

    #######################################################################
    def get_map_2D_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name="map_2D_node",
            parameters=[node_params],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        
    map_2D_node = get_map_2D_node('radar_ui', 'map_2D::map_2D_pub')
    
    def get_radar_ui_container(ui_node):
        return ComposableNodeContainer(
            name='radar_ui_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ui_node
            ],
            output='screen',
            ros_arguments=['--ros-args', '--log-level',
                           'map_2D_node:=' + launch_params['detector_log_level']
            ],
        )
        
    radar_ui_module = get_radar_ui_container(map_2D_node)
    
    delay_radar_ui_module = TimerAction(
        period='1.0',
        actions=[radar_ui_module]
    )
    ########################################################################

    return LaunchDescription(
        [
            radar_cv_module,
            delay_radar_ui_module 
        ]
    )
