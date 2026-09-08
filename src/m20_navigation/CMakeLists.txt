cmake_minimum_required(VERSION 3.16)
project(m20_navigation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration io features search surface kdtree octree)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# Optional M20 vendor DrDDS SDK (present only on the real robot)
set(M20_VENDOR_DRDDS_CMAKE_DIR "/usr/local/lib/cmake/drdds" CACHE PATH
  "Directory containing the robot vendor drddsConfig.cmake")
if(EXISTS "${M20_VENDOR_DRDDS_CMAKE_DIR}/drddsConfig.cmake")
  set(drdds_DIR "${M20_VENDOR_DRDDS_CMAKE_DIR}" CACHE PATH "" FORCE)
endif()
find_package(drdds QUIET CONFIG)

# Determine if DrDDS SDK is actually usable
set(M20_HAS_DRDDS FALSE)
if(drdds_FOUND)
  # Check that the required IDL-generated headers exist
  foreach(required_header
    dridl/sensor_msgs/msg/PointCloud2PubSubTypes.h
    dridl/dr_msgs/msg/MotionInfoPubSubTypes.h
    dridl/dr_msgs/msg/NavCmdPubSubTypes.h
    dridl/dr_msgs/msg/PlannerStatusPubSubTypes.h
    dridl/dr_msgs/msg/StdMsgInt32PubSubTypes.h
    dridl/dr_srvs/srv/PoseStampedToInt32PubSubTypes.h
    dridl/dr_srvs/srv/StdSrvInt32PubSubTypes.h
    dridl/dr_srvs/srv/set_paramPubSubTypes.h)
    if(NOT EXISTS "${drdds_INCLUDE_DIR}/${required_header}")
      set(M20_HAS_DRDDS FALSE)
      break()
    endif()
    set(M20_HAS_DRDDS TRUE)
  endforeach()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Core navigation library (common, terrain, planning, control)
# ---------------------------------------------------------------------------
set(M20_NAV_SOURCES
  src/common/math_utils.cpp
  src/common/params.cpp
  src/terrain/elevation_grid.cpp
  src/terrain/slope_analyzer.cpp
  src/terrain/roughness_analyzer.cpp
  src/terrain/step_detector.cpp
  src/terrain/traversability_map.cpp
  src/planning/hybrid_astar.cpp
  src/planning/motion_primitives.cpp
  src/planning/spline_optimizer.cpp
  src/planning/global_planner_node.cpp
  src/control/dwa_planner.cpp
  src/control/line_planner.cpp
  src/control/local_controller_node.cpp
)

set(M20_NAV_DEPENDENCIES
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  std_srvs
  std_msgs
  builtin_interfaces
  visualization_msgs
  pcl_conversions
)

add_library(m20_navigation_core SHARED ${M20_NAV_SOURCES})
ament_target_dependencies(m20_navigation_core ${M20_NAV_DEPENDENCIES})
target_link_libraries(m20_navigation_core
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)
target_include_directories(m20_navigation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------------------
# Navigation procedure node (component)
# ---------------------------------------------------------------------------
add_library(navigation_node_component SHARED src/nodes/navigation_node.cpp)
ament_target_dependencies(navigation_node_component ${M20_NAV_DEPENDENCIES})
target_link_libraries(navigation_node_component
  m20_navigation_core
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)
target_include_directories(navigation_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_node(navigation_node_component
  PLUGIN "m20::nodes::NavigationNode"
  EXECUTABLE navigation_node
)

# ---------------------------------------------------------------------------
# Optional native DrDDS bridge (only on the real robot)
# ---------------------------------------------------------------------------
add_library(m20_native_navigation_bridge SHARED src/ros/native_navigation_bridge.cpp)
target_include_directories(m20_native_navigation_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(m20_native_navigation_bridge
  m20_navigation_core
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)
if(M20_HAS_DRDDS)
  target_compile_definitions(m20_native_navigation_bridge PRIVATE M20_HAS_DRDDS=1)
  target_include_directories(m20_native_navigation_bridge SYSTEM PRIVATE
    ${drdds_INCLUDE_DIRS})
  if(TARGET drdds::drdds)
    target_link_libraries(m20_native_navigation_bridge drdds::drdds)
  else()
    target_link_directories(m20_native_navigation_bridge PRIVATE /usr/local/lib)
    target_link_libraries(m20_native_navigation_bridge ${drdds_LIBRARIES})
  endif()
endif()
target_link_libraries(navigation_node_component m20_native_navigation_bridge)
if(M20_HAS_DRDDS)
  target_compile_definitions(navigation_node_component PRIVATE M20_HAS_DRDDS=1)
endif()

# ---------------------------------------------------------------------------
# OccGrid bridge: converts lightning-lm occ_grid output to /GRID_MAP topic
# ---------------------------------------------------------------------------
add_executable(occ_grid_bridge src/occ_grid_bridge.cpp)
ament_target_dependencies(occ_grid_bridge rclcpp nav_msgs nav2_map_server)
target_link_libraries(occ_grid_bridge Eigen3::Eigen m20_navigation_core ${PCL_LIBRARIES})
target_include_directories(occ_grid_bridge PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS
  m20_navigation_core
  m20_native_navigation_bridge
  navigation_node_component
  occ_grid_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_native_navigation_contract test/test_native_navigation_contract.cpp)
  if(TARGET test_native_navigation_contract)
    target_include_directories(test_native_navigation_contract PRIVATE
      include ${PCL_INCLUDE_DIRS})
    ament_target_dependencies(test_native_navigation_contract rclcpp)
    target_link_libraries(test_native_navigation_contract
      m20_navigation_core ${PCL_LIBRARIES})
  endif()
endif()

ament_package()