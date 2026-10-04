cmake_minimum_required(VERSION 3.8)
project(scoutiq_world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install world files, models, and launch scripts
install(
  DIRECTORY worlds models launch
  DESTINATION share/${PROJECT_NAME}
)

# Install environment hooks so IGN_GAZEBO_RESOURCE_PATH is automatically set
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/scoutiq_world.sh.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/scoutiq_world.dsv.in"
)

ament_package()
