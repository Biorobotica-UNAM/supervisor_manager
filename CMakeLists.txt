cmake_minimum_required(VERSION 3.8)
project(supervisor_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(supervisor_interfaces REQUIRED)
find_package(planning_tools REQUIRED)
find_package(clips_interfaces REQUIRED)
find_package(known_locations_interfaces REQUIRED)
find_package(listener_interfaces REQUIRED)
find_package(talker_interfaces REQUIRED)

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
  known_locations_interfaces
  listener_interfaces
  talker_interfaces
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
  std_msgs
  std_srvs
  supervisor_interfaces
  clips_interfaces
  planning_tools
  known_locations_interfaces
  listener_interfaces
  talker_interfaces
)

ament_package()
