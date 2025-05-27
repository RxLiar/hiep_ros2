cmake_minimum_required(VERSION 3.8)
project(hiep_robot2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python module
ament_python_install_package(${PROJECT_NAME})

# Install scripts
install(
  PROGRAMS
  serial_comm_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
