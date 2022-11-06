cmake_minimum_required(VERSION 3.5.0)
project(ign_robot)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# joy2twist::Joy2Twist ------------------------------------------------------
set(TARGET joy2twist)
ament_auto_add_library(${TARGET} SHARED
  examples/${TARGET}.cpp
)
rclcpp_components_register_node(
  ${TARGET}
    PLUGIN "joy2twist::Joy2Twist"
    EXECUTABLE ${TARGET}_node
)
# ----------------------------------------------

# joy2direct::Joy2Direct ------------------------------------------------------
set(TARGET joy2direct)
ament_auto_add_library(${TARGET} SHARED
  examples/${TARGET}.cpp
)
rclcpp_components_register_node(
  ${TARGET}
    PLUGIN "joy2direct::Joy2Direct"
    EXECUTABLE ${TARGET}_node
)
# ----------------------------------------------


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  urdf
)
