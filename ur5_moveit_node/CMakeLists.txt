cmake_minimum_required(VERSION 3.8)
project(ur5_moveit_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(move_trajectory src/move_trajectory.cpp)
target_compile_features(move_trajectory PUBLIC cxx_std_17)
ament_target_dependencies(move_trajectory
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

install(TARGETS move_trajectory
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
