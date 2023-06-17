cmake_minimum_required(VERSION 3.10)
project(radar_master)

#######################
## Find dependencies ##
#######################
find_package(ament_cmake_auto REQUIRED)

###########
## Build ##
###########


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
