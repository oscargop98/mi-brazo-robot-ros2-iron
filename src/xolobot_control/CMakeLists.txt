cmake_minimum_required(VERSION 3.8)
project(xolobot_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)


install(
  DIRECTORY 
    config 
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
