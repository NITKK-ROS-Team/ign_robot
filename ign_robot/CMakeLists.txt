cmake_minimum_required(VERSION 3.5.0)
project(ign_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
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
set(TARGET joy2angle)
ament_auto_add_library(${TARGET} SHARED
  examples/${TARGET}.cpp
)
rclcpp_components_register_node(
  ${TARGET}
    PLUGIN "joy2angle::Joy2Angle"
    EXECUTABLE ${TARGET}_node
)
# ----------------------------------------------
set(TARGET joy2omni)
ament_auto_add_library(${TARGET} SHARED
  examples/${TARGET}.cpp
)
rclcpp_components_register_node(
  ${TARGET}
    PLUGIN "joy2omni::Joy2Omni"
    EXECUTABLE ${TARGET}_node
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  urdf
  xacro
  stl
)