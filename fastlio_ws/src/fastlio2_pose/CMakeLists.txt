cmake_minimum_required(VERSION 3.8)
project(fastlio2_pose)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(fastlio2_pose_node
  src/fastlio2_pose_node.cpp
)

ament_target_dependencies(fastlio2_pose_node
  rclcpp
  nav_msgs
  geometry_msgs
)

target_link_libraries(fastlio2_pose_node
  Eigen3::Eigen
)

install(
  TARGETS fastlio2_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()