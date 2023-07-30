cmake_minimum_required(VERSION 3.10)
project(keyboard)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#######################
## Find dependencies ##
#######################
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_executable(keyboard
  src/keyboard.cpp
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

ament_auto_package()
