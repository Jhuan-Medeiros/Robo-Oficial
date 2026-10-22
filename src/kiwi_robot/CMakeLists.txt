cmake_minimum_required(VERSION 3.8)
project(kiwi_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(kiwi_node src/kiwi_node.cpp)
ament_target_dependencies(kiwi_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(oms_node src/oms_node.cpp)
ament_target_dependencies(oms_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_executable(controle_node src/controle_node.cpp)
ament_target_dependencies(controle_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  kiwi_node
  oms_node
  controle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
