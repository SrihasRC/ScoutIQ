cmake_minimum_required(VERSION 3.8)
project(roomwatch_gz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/tuck_arm
  scripts/set_head
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
