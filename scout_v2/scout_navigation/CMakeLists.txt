cmake_minimum_required(VERSION 3.5)
project(scout_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps rviz slam_params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
