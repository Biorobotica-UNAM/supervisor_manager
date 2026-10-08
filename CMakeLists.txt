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
find_package(action_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(executor_interfaces REQUIRED)
find_package(supervisor_interfaces REQUIRED)
find_package(planning_tools REQUIRED)
find_package(clips_interfaces REQUIRED)
find_package(head_interfaces REQUIRED)
find_package(known_locations_interfaces REQUIRED)
find_package(listener_interfaces REQUIRED)
find_package(semantic_parser_interfaces REQUIRED)
find_package(talker_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(supervisor_recovery_modules STATIC
  src/navigation_recovery.cpp src/manipulation_recovery.cpp src/pnp_recovery.cpp
  src/recovery/door_sequence.cpp
  src/recovery/follow_sequence.cpp
  src/recovery/interaction_recovery.cpp
  src/recovery/manipulation_recovery_module.cpp
  src/recovery/navigation_recovery_module.cpp
  src/recovery/perception_recovery.cpp
  src/recovery/person_recovery.cpp
  src/recovery/recovery_coordinator.cpp
  src/recovery/recovery_support.cpp
  src/recovery/task_acquisition.cpp
)
target_compile_features(supervisor_recovery_modules PUBLIC cxx_std_17)
target_include_directories(supervisor_recovery_modules PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(supervisor_recovery_modules rclcpp action_msgs executor_interfaces std_msgs std_srvs
  supervisor_interfaces planning_tools clips_interfaces ament_index_cpp head_interfaces
  known_locations_interfaces listener_interfaces semantic_parser_interfaces talker_interfaces nlohmann_json tf2 tf2_ros)
target_link_libraries(supervisor_recovery_modules yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_recovery_modules test/test_recovery_modules.cpp)
  target_include_directories(test_recovery_modules PRIVATE include)
  target_compile_definitions(test_recovery_modules PRIVATE
    MODULE_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/recovery_modules.yaml"
    MANIPULATION_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/manipulation_recovery.yaml")
  ament_target_dependencies(test_recovery_modules rclcpp std_msgs supervisor_interfaces
    planning_tools clips_interfaces ament_index_cpp head_interfaces known_locations_interfaces
    listener_interfaces semantic_parser_interfaces talker_interfaces nlohmann_json tf2 tf2_ros)
  target_link_libraries(test_recovery_modules supervisor_recovery_modules yaml-cpp)

  ament_add_gtest(test_manipulation_recovery
    test/test_manipulation_recovery.cpp src/manipulation_recovery.cpp)
  target_include_directories(test_manipulation_recovery PRIVATE include)
  target_compile_definitions(test_manipulation_recovery PRIVATE
    MANIPULATION_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/manipulation_recovery.yaml")
  target_link_libraries(test_manipulation_recovery yaml-cpp)

  ament_add_gtest(test_navigation_recovery
    test/test_navigation_recovery.cpp src/navigation_recovery.cpp)
  target_include_directories(test_navigation_recovery PRIVATE include)
  target_compile_definitions(test_navigation_recovery PRIVATE
    NAVIGATION_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/navigation_recovery.yaml")
  target_link_libraries(test_navigation_recovery yaml-cpp)

  ament_add_gtest(test_pnp_recovery
    test/test_pnp_recovery.cpp
    src/pnp_recovery.cpp
  )
  target_include_directories(test_pnp_recovery PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_pnp_recovery
    rclcpp
  )
endif()

add_executable(supervisor_manager_node
  src/supervisor_manager_node.cpp
)

target_include_directories(supervisor_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(supervisor_manager_node
  rclcpp
  action_msgs
  std_msgs
  std_srvs
  supervisor_interfaces
  clips_interfaces
  planning_tools
  ament_index_cpp
  head_interfaces
  known_locations_interfaces
  listener_interfaces
  semantic_parser_interfaces
  talker_interfaces
  nlohmann_json
  tf2
  tf2_ros
)

target_link_libraries(supervisor_manager_node
  supervisor_recovery_modules yaml-cpp
)

install(TARGETS supervisor_recovery_modules ARCHIVE DESTINATION lib)
ament_export_libraries(supervisor_recovery_modules)

install(TARGETS
  supervisor_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config docs
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

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
  head_interfaces
  known_locations_interfaces
  listener_interfaces
  semantic_parser_interfaces
  talker_interfaces
  nlohmann_json
  tf2
  tf2_ros
)

ament_package()
