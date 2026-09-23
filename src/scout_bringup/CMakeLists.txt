cmake_minimum_required(VERSION 3.8)
project(scout_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/save_ld19_map
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
