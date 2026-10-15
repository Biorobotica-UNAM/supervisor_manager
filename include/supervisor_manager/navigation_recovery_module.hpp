#pragma once
#include "supervisor_manager/recovery_support.hpp"
#include "supervisor_manager/retry_rules.hpp"
namespace supervisor_manager {
class NavigationRecoveryModule : public RecoverySupport {

  enum class NavigationIoState {IDLE, START_SPEAK, WAIT_SPEAK, START_LISTEN, WAIT_LISTEN, START_EXTRACT, WAIT_EXTRACT, START_VALIDATE, WAIT_VALIDATE};
  struct GoToArrivalVerification
  {
    bool applicable{false};
    bool verified{false};
    bool within_tolerance{true};
    double distance_m{0.0};
    std::string goal_frame;
    std::string error_message;
  };

public:
  explicit NavigationRecoveryModule(RecoveryContext & context) : RecoverySupport(context), general_rules_(context.config["navigation"]["cases"]) {navigation_recovery_ = std::make_unique<NavigationRecovery>(get_parameter("navigation_recovery_config").as_string());}
  bool active() const {return navigation_io_state_ != NavigationIoState::IDLE;}
  bool handleFeedback(const Report & report);
  bool tick() {return tickNavigationRecovery();}
  void reset() {
    general_rules_.reset(); navigation_recovery_->reset();
    if (navigation_entity_future_ && context_.entities) {context_.entities->prune_pending_requests();}
    navigation_entity_future_.reset(); navigation_speech_.clear();
    arrival_original_.clear(); arrival_corrected_.clear();
    navigation_io_state_ = NavigationIoState::IDLE;
  }

private:
  GoToArrivalVerification verifySuccessfulGoToArrival(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
  void updateInstructionPointStateFromGoTo(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
  bool handleNavigationFeedback(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback);
  const char * navigationBehaviorName(supervisor_manager::NavigationRecovery::Behavior behavior) const;
  void setNavigationState(NavigationIoState next);
  void rejectNavigationDestination(const std::string & reason);
  bool tickNavigationRecovery();
private:
  RetryRules general_rules_;
  NavigationIoState navigation_io_state_{NavigationIoState::IDLE};
  std::unique_ptr<supervisor_manager::NavigationRecovery> navigation_recovery_;
  supervisor_manager::NavigationRecovery::Decision navigation_decision_;
  std::string navigation_error_, navigation_argument_, navigation_prompt_, navigation_candidate_;
  std::string arrival_original_, arrival_corrected_;
  std::optional<rclcpp::Client<known_locations_interfaces::srv::LocationsGet>::SharedFuture> locations_validation_future_;
  std::chrono::steady_clock::time_point navigation_deadline_;
  std::optional<rclcpp::Client<talker_interfaces::srv::Talk>::SharedFuture> navigation_talk_future_;
  std::optional<rclcpp::Client<listener_interfaces::srv::Listen>::SharedFuture> navigation_listen_future_;
  std::optional<rclcpp::Client<semantic_parser_interfaces::srv::ExtractEntity>::SharedFuture> navigation_entity_future_;
  std::string navigation_speech_;

};
}  // namespace supervisor_manager
