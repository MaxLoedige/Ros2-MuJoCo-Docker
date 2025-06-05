cmake_minimum_required(VERSION 3.10.2)
project(google_robot_description)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY meshes urdf launch rviz DESTINATION share/${PROJECT_NAME})
