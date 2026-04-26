cmake_minimum_required(VERSION 3.8)
project(my_robot_rviz)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
