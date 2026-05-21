cmake_minimum_required(VERSION 3.22)
project(panthera_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hightorque_robot REQUIRED)

add_library(
  panthera_moveit_hardware
  SHARED
  src/panthera_hardware_interface.cpp
)

target_include_directories(
  panthera_moveit_hardware
  PRIVATE
  include
)

ament_target_dependencies(
  panthera_moveit_hardware
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

target_link_libraries(
  panthera_moveit_hardware
  hightorque_robot::hightorque_robot
)

pluginlib_export_plugin_description_file(hardware_interface panthera_moveit_hardware.xml)

install(
  TARGETS panthera_moveit_hardware
  DESTINATION lib
)

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
    urdf
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
