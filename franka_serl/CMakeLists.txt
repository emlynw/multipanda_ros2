cmake_minimum_required(VERSION 3.5)
project(franka_serl)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Add library (C++ Controllers)
add_library(${PROJECT_NAME} SHARED
  src/motion_generator.cpp
  src/cartesian_impedance_controller.cpp
  src/move_to_start_controller.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  controller_interface
  pluginlib
  franka_msgs
  std_msgs
  std_srvs
  rclcpp_lifecycle
  hardware_interface
  franka_semantic_components
)

# Add executable (C++ Gripper Server)
add_executable(franka_gripper_server_node src/franka_gripper_server_node.cpp)
target_include_directories(franka_gripper_server_node
  PUBLIC include
)
ament_target_dependencies(franka_gripper_server_node
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  franka_msgs
)

# Install C++ Targets
install(
  TARGETS
    franka_gripper_server_node
    ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install Python Executable
install(PROGRAMS
  scripts/franka_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

# Pluginlib Export
pluginlib_export_plugin_description_file(controller_interface franka_controllers.xml)

# Export Package
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_action
  controller_interface
  pluginlib
  franka_msgs
  std_msgs
  std_srvs
  Eigen3
)

ament_package()