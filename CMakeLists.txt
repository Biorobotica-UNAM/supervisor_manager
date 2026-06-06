cmake_minimum_required(VERSION 3.8)
project(supervisor_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(executor_interfaces REQUIRED)
find_package(supervisor_interfaces REQUIRED)
find_package(planning_tools REQUIRED)
find_package(clips_interfaces REQUIRED)
find_package(known_locations_interfaces REQUIRED)
find_package(listener_interfaces REQUIRED)
find_package(talker_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(supervisor_manager_node src/supervisor_manager_node.cpp)

target_include_directories(supervisor_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(supervisor_manager_node
  rclcpp
  std_msgs
  std_srvs
  supervisor_interfaces
  clips_interfaces
  planning_tools
  ament_index_cpp
  known_locations_interfaces
  listener_interfaces
  talker_interfaces
  nlohmann_json
)

target_link_libraries(supervisor_manager_node
  yaml-cpp
)

install(TARGETS
  supervisor_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  actionlib_msgs
  executor_interfaces
  supervisor_interfaces
  clips_interfaces
  planning_tools
  ament_index_cpp
  known_locations_interfaces
  listener_interfaces
  talker_interfaces
  nlohmann_json
)

ament_package()
