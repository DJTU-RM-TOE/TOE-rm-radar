cmake_minimum_required(VERSION 3.10)
project(radar_master)

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

ament_auto_add_executable(global_parameter_server
    src/global_parameter_server.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ImageSubscriber
  EXECUTABLE vidio_log_node
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
  config
  launch
)
