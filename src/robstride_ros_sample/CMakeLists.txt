cmake_minimum_required(VERSION 3.8)
project(rs_motor_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/MotorFeedback.msg"
#   DEPENDENCIES std_msgs
# )

# Create a ament index resource to make the package findable
ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(
  include
)

# Declare a cpp library with a different name to avoid conflict
add_library(${PROJECT_NAME}_lib
  src/motor_cfg.cpp
)

# Rename the output to avoid confusion
set_target_properties(${PROJECT_NAME}_lib PROPERTIES
  OUTPUT_NAME ${PROJECT_NAME}
)

# Use the generated interfaces

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  sensor_msgs
)

# rosidl_target_interfaces(${PROJECT_NAME}_lib 
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_library(c620_lib
  src/c620_motor.cpp
)

ament_target_dependencies(c620_lib
  rclcpp
)

# RobStride leg1
add_executable(rs_motor_ros2
  src/main.cpp
)

add_executable(c620_ros2
  src/c620_main.cpp
)

add_executable(leg1_dual_pose
  src/leg1_dual_pose.cpp
)

add_executable(leg1_dual_ros2
  src/leg1_dual_main.cpp
)

add_executable(quad_rs_ros2
  src/quad_rs_main.cpp
)

add_executable(c620_quad_ros2
  src/c620_quad_main.cpp
)

add_executable(quad_teleop_ros2
  src/quad_teleop_main.cpp
)

add_executable(quad_odom_ros2
  src/quad_odom_main.cpp
)

ament_target_dependencies(rs_motor_ros2
  rclcpp
  std_msgs
)

ament_target_dependencies(c620_ros2
  rclcpp
  std_msgs
  sensor_msgs
)

target_link_libraries(rs_motor_ros2
  ${PROJECT_NAME}_lib
)

target_link_libraries(c620_ros2
  c620_lib
)

target_link_libraries(leg1_dual_pose
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(leg1_dual_ros2
  rclcpp
  std_msgs
  sensor_msgs
)

target_link_libraries(leg1_dual_ros2
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(quad_rs_ros2
  rclcpp
  std_msgs
  sensor_msgs
)

target_link_libraries(quad_rs_ros2
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(c620_quad_ros2
  rclcpp
  std_msgs
  sensor_msgs
)

target_link_libraries(c620_quad_ros2
  c620_lib
)

ament_target_dependencies(quad_teleop_ros2
  rclcpp
  std_msgs
  geometry_msgs
)

ament_target_dependencies(quad_odom_ros2
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}_lib rs_motor_ros2 c620_lib c620_ros2 c620_quad_ros2 quad_teleop_ros2 quad_odom_ros2 leg1_dual_pose leg1_dual_ros2 quad_rs_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
