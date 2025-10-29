cmake_minimum_required(VERSION 3.5)
project(turtlesim_automata)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(turtlesim_automata_node src/turtlesim_automata_node.cpp)
ament_target_dependencies(turtlesim_automata_node rclcpp turtlesim std_msgs geometry_msgs)

install(TARGETS
  turtlesim_automata_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

