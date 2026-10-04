cmake_minimum_required(VERSION 3.8)
project(scoutiq_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in"
)

ament_package()
