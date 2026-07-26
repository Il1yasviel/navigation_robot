cmake_minimum_required(VERSION 3.8)
project(navigation_robot_base_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME}_protocol src/protocol.cpp)
target_include_directories(${PROJECT_NAME}_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(base_driver_node src/base_driver_node.cpp)
target_link_libraries(base_driver_node ${PROJECT_NAME}_protocol)
ament_target_dependencies(base_driver_node
  diagnostic_msgs geometry_msgs nav_msgs rclcpp rclcpp_lifecycle sensor_msgs std_srvs)

install(TARGETS ${PROJECT_NAME}_protocol base_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch scripts DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol ${PROJECT_NAME}_protocol)
  target_include_directories(test_protocol PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_protocol)
ament_package()
