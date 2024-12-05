cmake_minimum_required(VERSION 3.8)
project(ros2_kdl_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(controller_manager REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Define include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Add nodes
add_executable(ros2_kdl_node 
  src/ros2_kdl_node.cpp 
  src/kdl_robot.cpp 
  src/kdl_planner.cpp 
  src/kdl_control.cpp
)

ament_target_dependencies(ros2_kdl_node 
  orocos_kdl
  rclcpp
  kdl_parser
  sensor_msgs
  geometry_msgs
  urdf
)

add_executable(ros2_kdl_vision_dynamic
  src/ros2_kdl_vision_dynamic.cpp 
  src/kdl_robot.cpp 
  src/kdl_planner.cpp 
  src/kdl_control.cpp
)

ament_target_dependencies(ros2_kdl_vision_dynamic
  orocos_kdl
  rclcpp
  kdl_parser
  sensor_msgs
  geometry_msgs
  urdf
  tf2
  tf2_ros
)


add_executable(ros2_kdl_vision_control
  src/ros2_kdl_vision_control.cpp 
  src/kdl_robot.cpp 
  src/kdl_planner.cpp 
  src/kdl_control.cpp
)

ament_target_dependencies(ros2_kdl_vision_control
  orocos_kdl
  rclcpp
  kdl_parser
  sensor_msgs
  geometry_msgs
  urdf
  tf2
  tf2_ros
)

# Install executables
install(TARGETS 
  ros2_kdl_node
  ros2_kdl_vision_dynamic
  ros2_kdl_vision_control

  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
