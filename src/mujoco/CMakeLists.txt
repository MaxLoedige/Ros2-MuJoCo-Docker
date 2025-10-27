cmake_minimum_required(VERSION 3.10.2)
project(mujoco_world)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
