cmake_minimum_required(VERSION 3.8)
project(m20_joints_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_CXX_EXTENSIONS)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# drdds is the M20 robot's custom message package. It is only present on the real
# robot (sourced from /opt/robot) or when built from sdk_deploy. Use QUIET so the
# build does not hard-fail on sim/dev machines; the real-robot node is only built
# when drdds is available.
find_package(drdds QUIET)

if(NOT drdds_FOUND)
  message(STATUS "drdds not found; m20_joints_adapter will not build the real-robot node. This is expected on sim/dev machines.")
else()
  add_executable(joints_adapter_node src/joints_adapter_node.cpp)
  ament_target_dependencies(joints_adapter_node
    rclcpp sensor_msgs drdds)
  install(TARGETS joints_adapter_node DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
