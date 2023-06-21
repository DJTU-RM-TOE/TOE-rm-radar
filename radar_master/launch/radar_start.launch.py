import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    
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
    
    def radar_robot_detector_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name="detector_node",
            parameters=[node_params],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    
    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
    radar_detector_node = radar_robot_detector_node('radar_robot_detector', 'radar_detector::detector_node')
        
        
    def radar_cv_container(camera_node):
        return ComposableNodeContainer(
            name='radar_cv_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                radar_detector_node,
                camera_node
            ],
            output='screen',
        )
        
    radar_cv_module = radar_cv_container(hik_camera_node)

    #######################################################################
    def map_2D(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name="map_2D_node",
            parameters=[node_params],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        
    map_2D_node = map_2D('radar_ui', 'map_2D::map_2D_pub')
    
    def radar_ui_container(ui_node):
        return ComposableNodeContainer(
            name='radar_ui_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ui_node,
            ],
            output='screen',
        )
        
    radar_ui_module = radar_ui_container(map_2D_node)
    ########################################################################

    return LaunchDescription(
        [
            radar_cv_module,
            radar_ui_module
        ]
    )
