import os
import yaml

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from launch_ros.descriptions import ComposableNode

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

def get_radar_parma_node(package, executable, name):
    return Node(
        package=package,  # 替换为你的包名
        executable=executable,  # 替换为你的可执行文件名
        name=name,
        parameters=[node_params],
    )
    
def get_camera_node(package, plugin, name):
    return ComposableNode(
        package=package,
        plugin=plugin,
        name= name,
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
    
def get_map_2D_node(package, plugin):
    return ComposableNode(
        package=package,
        plugin=plugin,
        name="map_2D_node",
        parameters=[node_params],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

def get_qt5_ui_node(package, plugin, name):
    return ComposableNode(
        package=package,  # 替换为你的包名
        plugin=plugin,  # 替换为你的可执行文件名
        name=name,
        parameters=[node_params],
    )
    
def get_serial_node(package, plugin):
    return ComposableNode(
        package=package,
        plugin=plugin,
        name="serial_node",
        parameters=[node_params],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

def get_radar_identification_node(package, executable, name):
    return Node(
        package=package,  # 替换为你的包名
        executable=executable,  # 替换为你的可执行文件名
        name=name,
        parameters=[node_params],
    )
    
def get_keyboard_node(package, executable, name):
    return Node(
        package=package,  # 替换为你的包名
        executable=executable,  # 替换为你的可执行文件名
        name=name,
        parameters=[node_params],
    )