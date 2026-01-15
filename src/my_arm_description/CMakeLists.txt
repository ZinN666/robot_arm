cmake_minimum_required(VERSION 3.5)
project(my_arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY my_arm/meshes my_arm/srdf my_arm/urdf my_arm/rviz
  DESTINATION share/${PROJECT_NAME}/my_arm
)

ament_package()


