import os
import sys

from launch_ros.actions import ComposableNodeContainer 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch.actions import TimerAction

sys.path.append(os.path.join(get_package_share_directory('radar_master'), 'launch'))

from common import launch_params

def get_radar_cv_container(camera_node_1,camera_node_2,radar_robot_detector_node,calibration_ui_node):
    return ComposableNodeContainer(
        name='radar_cv_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            camera_node_1,
            camera_node_2,
            radar_robot_detector_node,
            calibration_ui_node
        ],
        output='screen',
    )

def get_radar_ui_container(ui_node):
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

def get_radar_serial_container(serial_node):
    return ComposableNodeContainer(
        name='radar_ui_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            serial_node,
        ],
        output='screen',
    )
    
def get_radar_qt5_ui_container(qt5_ui_node):
    return ComposableNodeContainer(
        name='radar_qt5_ui_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            qt5_ui_node,
        ],
        output='screen',
    )

def generate_launch_description():
    from common import get_camera_node,get_radar_orientation_node,get_calibration_ui_node
    from common import get_map_2D_node, get_serial_node
    from common import get_radar_identification_node, get_radar_parma_node,get_keyboard_node

    ##############
    # 参数服务进程 #
    ##############
    
    radar_parma_node = get_radar_parma_node("radar_master", "global_parameter_server", "global_parameter_server")
    
    ##############
    # 图像标定进程 #
    ##############
    
    hik_camera_node_1 = get_camera_node("hik_camera", "hik_camera::HikCameraNode", "camera_1_node")
    hik_camera_node_2 = get_camera_node("hik_camera", "hik_camera::HikCameraNode", "camera_2_node")

    radar_robot_detector_node = get_radar_orientation_node("radar_orientation", "radar_orientation::OrientationNode")
    calibration_ui_node = get_calibration_ui_node("radar_ui", "calibration_ui::calibration_ui_node")

    radar_cv_module = get_radar_cv_container(hik_camera_node_1,hik_camera_node_2,radar_robot_detector_node,calibration_ui_node)
    
    ##############
    # 图像推理进程 #
    ##############

    radar_identification_node_1 = get_radar_identification_node("radar_identification", "python_node", "radar_identification_node_1")
    radar_identification_node_2 = get_radar_identification_node("radar_identification", "python_node", "radar_identification_node_2")

    ##############
    # 辅助功能进程 #
    ##############

    serial_node = get_serial_node('radar_serial', 'radar_serial_driver::RadarSerialDriver')
    map_2D_node = get_map_2D_node('radar_ui', 'map_2d_ui::Map2dUiNode')
    
    radar_ui_module = get_radar_ui_container(map_2D_node)
    radar_serial_module = get_radar_serial_container(serial_node)
    
    ##############
    # 图形界面进程 #
    ##############
    
    #keyboard_node = get_keyboard_node('keyboard', 'keyboard', 'keyboard')
    
    #qt5_ui_node = get_qt5_ui_node('radar_ui', 'qt5_ui::Qt5UiNode','qt5_ui_node')
    #qt5_ui_module = get_radar_qt5_ui_container(qt5_ui_node)
    
    delay_radar_identification_module = TimerAction(
        period=3.0,
        actions=[radar_identification_node_1,radar_identification_node_2]
    )
    
    ########################################################################

    return LaunchDescription(
        [
            radar_cv_module,
            radar_ui_module,
            delay_radar_identification_module,
            radar_parma_node,
            radar_serial_module,
            #keyboard_node
        ]
    )
