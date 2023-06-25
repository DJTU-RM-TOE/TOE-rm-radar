import os
import sys

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription

from launch.actions import TimerAction

from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('radar_master'), 'launch'))


def generate_launch_description():
    
    from common import get_camera_node,get_radar_robot_detector_node,get_calibration_ui_node,get_radar_cv_container
    from common import get_map_2D_node,get_radar_ui_container,get_save_node
    
    hik_camera_node = get_camera_node('none_camera', 'none_camera::NoneCameraNode')
    radar_robot_detector_node = get_radar_robot_detector_node('radar_robot_detector', 'radar_detector::detector_node')
    calibration_ui_node = get_calibration_ui_node('radar_ui', 'calibration_ui::calibration_ui_node')
    
    radar_cv_module = get_radar_cv_container(hik_camera_node,radar_robot_detector_node,calibration_ui_node)

    #######################################################################
        
    map_2D_node = get_map_2D_node('radar_ui', 'map_2d_ui::Map2dUiNode')
    
    save_node = get_save_node('radar_master', 'save_data::SaveDataNode')
    
    radar_ui_module = get_radar_ui_container(map_2D_node,save_node)
    
    #delay_radar_ui_module = TimerAction(
    #    period='3.0',
    #    actions=[radar_ui_module]
    #)
    ########################################################################
    

    return LaunchDescription(
        [
            radar_cv_module,
            radar_ui_module
        ]
    )

