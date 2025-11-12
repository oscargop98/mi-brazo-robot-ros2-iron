cmake_minimum_required(VERSION 3.8)
project(xolobot_arm_server)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED) # Para mensajes y logs
find_package(gazebo_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(linkattacher_msgs REQUIRED)



include_directories(include)

add_executable(xolobot_arm_server
  src/startServerNode.cpp
  src/SimulationController.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${gazebo_msgs_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
)

install(TARGETS xolobot_arm_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
) 

ament_target_dependencies(xolobot_arm_server
  rclcpp
  fmt
  gazebo_msgs
  trajectory_msgs
  std_msgs
  gazebo_ros
  linkattacher_msgs
)

ament_package()
