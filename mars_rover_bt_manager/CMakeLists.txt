cmake_minimum_required(VERSION 3.8)
project(mars_rover_bt_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mars_rover_srvs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

add_executable(mars_rover_bt_manager_node
  src/mars_rover_bt_manager_node.cpp
  src/action_handlers.cpp
  src/action_publisher.cpp
  src/condition_handlers.cpp
  src/condition_manager.cpp
)

target_include_directories(mars_rover_bt_manager_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mars_rover_bt_manager_node rclcpp std_srvs std_msgs geometry_msgs  mars_rover_srvs behaviortree_cpp_v3 nlohmann_json)

install(
  TARGETS mars_rover_bt_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
