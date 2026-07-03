cmake_minimum_required(VERSION 3.8)
project(mec_mobile_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # Them dong nay
find_package(rclpy REQUIRED)               # Them dong nay

# Cai dat Python package
ament_python_install_package(${PROJECT_NAME})

# Dang ky Python executable
install(PROGRAMS
  mec_mobile_navigation/laser_merger_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch config rviz maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()