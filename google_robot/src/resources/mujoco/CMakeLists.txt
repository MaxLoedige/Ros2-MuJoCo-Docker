cmake_minimum_required(VERSION 3.10.2)
project(google_robot_mujoco)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY google_robot DESTINATION share/${PROJECT_NAME})
