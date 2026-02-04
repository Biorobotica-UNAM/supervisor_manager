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
find_package(supervisor_interfaces)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(supervisor_manager_node src/supervisor_manager_node.cpp)

target_include_directories(supervisor_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_link_libraries(supervisor_manager_node
  ${rclcpp_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(supervisor_manager_node
  rclcpp
  std_msgs
  std_srvs
  supervisor_interfaces
)

# Install executables
install(TARGETS
  supervisor_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
#install(DIRECTORY include/
#  DESTINATION include
#)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(supervisor_plan_node)
ament_export_dependencies(rclcpp std_msgs)


ament_package()
