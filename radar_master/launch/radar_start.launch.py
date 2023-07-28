import os
import sys

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription

from launch.actions import TimerAction

from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('radar_master'), 'launch'))


def generate_launch_description():
    
    from common import get_camera_node,get_radar_orientation_node,get_calibration_ui_node,get_radar_cv_container
    from common import get_map_2D_node,get_save_node,get_serial_node,get_radar_ui_container
    from common import get_radar_identification_node,get_radar_parma_node
    
    radar_parma_node = get_radar_parma_node('radar_master', 'global_parameter_server', "global_parameter_server")
    #######################################################################
    hik_camera_node_1 = get_camera_node('hik_camera', 'hik_camera::HikCameraNode', "camera_1_node")
    hik_camera_node_2 = get_camera_node('hik_camera', 'hik_camera::HikCameraNode', "camera_2_node")
    
    radar_robot_detector_node = get_radar_orientation_node('radar_orientation', 'radar_orientation::OrientationNode')
    calibration_ui_node = get_calibration_ui_node('radar_ui', 'calibration_ui::calibration_ui_node')
    
    radar_cv_module = get_radar_cv_container(hik_camera_node_1,hik_camera_node_2,radar_robot_detector_node,calibration_ui_node)
    #######################################################################
    
    radar_identification_node_1 = get_radar_identification_node('radar_identification', 'python_node','radar_identification_node_1')
    radar_identification_node_2 = get_radar_identification_node('radar_identification', 'python_node','radar_identification_node_2')

    #######################################################################
    
    serial_node = get_serial_node('radar_serial', 'radar_serial_driver::RadarSerialDriver')
    map_2D_node = get_map_2D_node('radar_ui', 'map_2d_ui::Map2dUiNode')
    save_node = get_save_node('radar_master', 'save_data::SaveDataNode')
    
    radar_ui_module = get_radar_ui_container(map_2D_node,save_node,serial_node)
    
    #delay_radar_ui_module = TimerAction(
    #    period='3.0',
    #    actions=[radar_ui_module]
    #)
    ########################################################################
    
    

    return LaunchDescription(
        [
            radar_cv_module,
            radar_parma_node,
            radar_identification_node_1,
            radar_identification_node_2,
            radar_ui_module
        ]
    )

