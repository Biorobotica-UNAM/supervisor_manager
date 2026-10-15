#pragma once
#include <rclcpp/rclcpp.hpp>

#include <supervisor_interfaces/msg/system_status.hpp>
#include <supervisor_interfaces/msg/task_execution_report.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <head_interfaces/srv/move_head.hpp>
#include <known_locations_interfaces/srv/locations_get.hpp>
#include <listener_interfaces/srv/listen.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <semantic_parser_interfaces/srv/extract_entity.hpp>
#include <talker_interfaces/srv/talk.hpp>
#include <planning_tools/Planning.hpp>

#include "supervisor_manager/pnp_recovery.hpp"
#include "supervisor_manager/navigation_recovery.hpp"
#include "supervisor_manager/manipulation_recovery.hpp"
#include <clips_interfaces/srv/string.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>




#include <functional>
namespace supervisor_manager {
using Report = supervisor_interfaces::msg::TaskExecutionReport;
// Services and plan proposals shared by modules. Only the manager publishes plan commands.
struct RecoveryContext {
  YAML::Node config;
  rclcpp::Node * node{};
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  std::function<rclcpp::Parameter(const std::string &)> parameter;
  tf2_ros::Buffer * tf{};
  std::shared_ptr<Planning> planning;
  rclcpp::Client<talker_interfaces::srv::Talk>::SharedPtr talker;
  rclcpp::Client<listener_interfaces::srv::Listen>::SharedPtr listener;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr door_check;
  rclcpp::Client<semantic_parser_interfaces::srv::ExtractEntity>::SharedPtr entities;
  rclcpp::Client<known_locations_interfaces::srv::LocationsGet>::SharedPtr go_to, locations;
  rclcpp::Client<head_interfaces::srv::MoveHead>::SharedPtr head;
  bool natural_language{false}, ask_confirmation{false}, at_instruction_point{false};
  std::string map_frame{"map"}, base_frame{"base_link"};
  double arrival_tolerance{0.75};
  PnpRecoveryManager pnp;
  std::function<bool()> allow_next, continue_on_failure, has_report;
  std::function<void()> retry, rewind, stop, clear_plan_ready;
  std::function<void(int)> skip;
  std::function<void(const std::string &)> insert;
  std::function<void(const std::string &, const std::string &)> update_arrival;
  std::function<void(bool)> finish_failed;
  std::function<std::string()> current_instruction;
  std::function<bool(bool &)> plan_active;
};
}  // namespace supervisor_manager
