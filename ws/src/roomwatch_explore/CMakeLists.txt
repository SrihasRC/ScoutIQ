cmake_minimum_required(VERSION 3.8)
project(roomwatch_explore)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(map_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

# Core library
add_library(roomwatch_explore_lib SHARED
  src/costmap_client.cpp
  src/frontier_search.cpp
  src/explore.cpp
)

ament_target_dependencies(roomwatch_explore_lib
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  nav2_costmap_2d
  nav2_map_server
  map_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Node executable
add_executable(explore
  src/explore_node.cpp
)
target_link_libraries(explore
  roomwatch_explore_lib
)
ament_target_dependencies(explore
  rclcpp
)

# Install libraries and executables
install(TARGETS roomwatch_explore_lib explore
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  scripts/savemap.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frontier_search
    test/test_frontier_search.cpp
  )
  target_link_libraries(test_frontier_search
    roomwatch_explore_lib
  )
  ament_target_dependencies(test_frontier_search
    rclcpp
    nav2_costmap_2d
    geometry_msgs
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(roomwatch_explore_lib)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  nav2_costmap_2d
  nav2_map_server
  map_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_package()
