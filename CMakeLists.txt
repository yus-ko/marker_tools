cmake_minimum_required(VERSION 3.5)
project(marker_tools)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(potbot_ros REQUIRED)

include_directories(include)

add_library(marker_tools
  src/trajectory_recoder.cpp
  src/waypoint_editor.cpp
  src/tree_map.cpp
  src/graph_map.cpp
)

ament_target_dependencies(marker_tools
  rclcpp
  potbot_ros
)

target_include_directories(marker_tools PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS marker_tools
  EXPORT export_marker_tools
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

add_executable(base_marker src/node/base_marker.cpp)
ament_target_dependencies(base_marker rclcpp nav2_util potbot_ros)

add_executable(trajectory_recoder src/node/trajectory_recoder.cpp)
ament_target_dependencies(trajectory_recoder rclcpp potbot_ros)
target_link_libraries(trajectory_recoder marker_tools)

add_executable(waypoint_editor src/node/waypoint_editor.cpp)
ament_target_dependencies(waypoint_editor rclcpp potbot_ros)
target_link_libraries(waypoint_editor marker_tools)

add_executable(tree_map src/node/tree_map.cpp)
ament_target_dependencies(tree_map rclcpp potbot_ros)
target_link_libraries(tree_map marker_tools)

add_executable(graph_map src/node/graph_map.cpp)
ament_target_dependencies(graph_map rclcpp potbot_ros)
target_link_libraries(graph_map marker_tools)

install(TARGETS
  base_marker
  trajectory_recoder
  waypoint_editor
  tree_map
  graph_map
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch params rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_marker_tools)
ament_export_dependencies(
  rclcpp
  nav2_util
  potbot_ros
)
ament_export_include_directories(include)

ament_package()
