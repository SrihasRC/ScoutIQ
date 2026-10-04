cmake_minimum_required(VERSION 3.8)
project(roomwatch_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/run_pipeline
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
