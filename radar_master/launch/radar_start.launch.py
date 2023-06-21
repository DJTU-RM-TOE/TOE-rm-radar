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
    
    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
        
    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
            ],
            output='screen',
        )

    cam_detector = get_camera_detector_container(hik_camera_node)



    return LaunchDescription(
        [cam_detector

        ]
    )
