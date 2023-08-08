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
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
find_package(Threads REQUIRED)

#message("thread lib:" ${Threads_LIBS})

set(QT_LIBRARIES Qt5::Widgets)

ament_auto_find_build_dependencies()

###########
## Build ##
###########

#ament_auto_add_executable(test_thead
#  src/test_thead.cpp
#)
#
#target_include_directories(test_thead PUBLIC ${OpenCV_INCLUDE_DIRS})
#target_link_libraries(test_thead ${OpenCV_LIBS} ${QT_LIBRARIES})

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src/component
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${QT_LIBRARIES})

#ament_auto_add_executable(qt_ui 
#  src/qt_ui.cpp
#)
#
#target_include_directories(qt_ui PUBLIC ${OpenCV_INCLUDE_DIRS})
#link_directories(/usr/lib/x86_64-linux-gnu/)
#target_link_libraries(qt_ui ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} Threads::Threads)
# LD_PRELOAD=/snap/core22/current/lib/x86_64-linux-gnu/libpthread.so.0

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
