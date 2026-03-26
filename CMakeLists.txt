cmake_minimum_required(VERSION 3.5)
project(isaac_ros_perceptor_realsense_d456)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
