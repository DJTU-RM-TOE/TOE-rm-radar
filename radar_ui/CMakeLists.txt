cmake_minimum_required(VERSION 3.10)
project(radar_ui)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#######################
## Find dependencies ##
#######################
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN map_2d_ui::Map2dUiNode
  EXECUTABLE map_2d_ui_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN calibration_ui::calibration_ui_node
  EXECUTABLE calibration_ui_node
)


#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  image
  urdf
)
