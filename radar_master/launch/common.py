import os
import yaml

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer 
from launch.actions import TimerAction

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

def get_radar_orientation_node(package, plugin):
    return ComposableNode(
        package=package,
        plugin=plugin,
        name="orientation_node",
        parameters=[node_params],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    
def get_calibration_ui_node(package, plugin):
    return ComposableNode(
    package=package,
    plugin=plugin,
    name="calibration_ui_node",
    parameters=[node_params],
    extra_arguments=[{"use_intra_process_comms": True}],
)
    
def get_radar_cv_container(camera_node,radar_robot_detector_node,calibration_ui_node):
    return ComposableNodeContainer(
        name='radar_cv_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            radar_robot_detector_node,
            camera_node,
            calibration_ui_node
        ],
        output='screen',
        ros_arguments=['--ros-args', '--log-level',
                       'radar_robot_detector_node:=' + launch_params['detector_log_level']
        ],
    )
    
def get_map_2D_node(package, plugin):
    return ComposableNode(
        package=package,
        plugin=plugin,
        name="map_2D_node",
        parameters=[node_params],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    
def get_save_node(package, plugin):
    return ComposableNode(
        package=package,
        plugin=plugin,
        name="save_node",
        parameters=[node_params],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    
def get_radar_ui_container(ui_node,save_node):
    return ComposableNodeContainer(
        name='radar_ui_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ui_node,
            save_node
        ],
        output='screen',
        ros_arguments=['--ros-args', '--log-level',
                       'map_2D_node:=' + launch_params['detector_log_level']
        ],
    )