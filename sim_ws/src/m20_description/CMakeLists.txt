cmake_minimum_required(VERSION 3.8)
project(m20_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(CONTROL_YAML_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/gazebo_ros2_control.yaml")
configure_file(urdf/m20.urdf ${CMAKE_CURRENT_BINARY_DIR}/m20.urdf @ONLY)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/m20.urdf
  DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY meshes config worlds launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/adapt_urdf.py scripts/stand_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
