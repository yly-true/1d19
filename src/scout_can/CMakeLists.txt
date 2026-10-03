cmake_minimum_required(VERSION 3.8)
project(scout_can)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(scout_can_node src/scout_can_node.cpp)
target_compile_features(scout_can_node PRIVATE cxx_std_17)
target_include_directories(scout_can_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(
  scout_can_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS scout_can_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(planar_odometry_test test/planar_odometry_test.cpp)
  target_compile_features(planar_odometry_test PRIVATE cxx_std_17)
  target_compile_options(planar_odometry_test PRIVATE -UNDEBUG)
  target_include_directories(planar_odometry_test PRIVATE include)
  add_test(NAME planar_odometry COMMAND planar_odometry_test)
endif()

ament_package()
