cmake_minimum_required(VERSION 3.8)
project(scout_slam_toolbox)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
