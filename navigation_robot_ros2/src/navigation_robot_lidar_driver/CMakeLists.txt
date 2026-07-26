cmake_minimum_required(VERSION 3.8)
project(navigation_robot_lidar_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(lidar_node src/lidar_node.cpp)
ament_target_dependencies(lidar_node diagnostic_msgs rclcpp sensor_msgs)

install(TARGETS lidar_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch scripts DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
