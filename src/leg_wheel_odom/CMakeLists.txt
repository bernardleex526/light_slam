cmake_minimum_required(VERSION 3.8)
project(leg_wheel_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

add_library(leg_wheel_odom_core INTERFACE)
target_include_directories(leg_wheel_odom_core INTERFACE include)
target_link_libraries(leg_wheel_odom_core INTERFACE Eigen3::Eigen)

ament_target_dependencies(leg_wheel_odom_core INTERFACE
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

if(BUILD_TESTING)
  # ament_copyright has no template for the Proprietary license declared in
  # package.xml; exclude it so the rest of the auto-lint set can run.
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics leg_wheel_odom_core)
endif()

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

add_executable(leg_wheel_odom_node src/leg_wheel_odom_node.cpp)
target_include_directories(leg_wheel_odom_node PRIVATE include)
target_link_libraries(leg_wheel_odom_node leg_wheel_odom_core)
ament_target_dependencies(leg_wheel_odom_node
  rclcpp sensor_msgs nav_msgs geometry_msgs)
install(TARGETS leg_wheel_odom_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
