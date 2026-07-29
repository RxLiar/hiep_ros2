cmake_minimum_required(VERSION 3.8)

project(mec_mobile_description1)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    urdf
    meshes
    launch
    #rviz
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()