#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "actionlib_msgs/msg/goal_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_interfaces/msg/task_result.hpp"

using namespace std::chrono_literals;

class SupervisorGoToTestNode : public rclcpp::Node
{
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTask>;

  SupervisorGoToTestNode()
  : Node("supervisor_goto_test_node")
  {
    this->declare_parameter<std::string>("location", "fake_place");
    this->declare_parameter<std::string>("terminal_location", "__nav_test__");
    this->declare_parameter<std::string>("scenario", "single");
    this->declare_parameter<std::string>("fake_status", "ABORTED");
    this->declare_parameter<std::string>("fake_status_text", "Injected navigation test failure");
    this->declare_parameter<std::string>("action", "go_to");
    this->declare_parameter<std::string>("agent", "Justina");
    this->declare_parameter<int>("step_id", 999);
    this->declare_parameter<double>("server_wait_seconds", 5.0);
    this->declare_parameter<double>("fake_status_delay_seconds", 1.0);
    this->declare_parameter<double>("next_test_delay_seconds", 1.0);
    this->declare_parameter<bool>("stop_after_fake_status", true);

    this->get_parameter("location", location_);
    this->get_parameter("terminal_location", terminal_location_);
    this->get_parameter("scenario", scenario_);
    this->get_parameter("fake_status", fake_status_);
    this->get_parameter("fake_status_text", fake_status_text_);
    this->get_parameter("action", action_);
    this->get_parameter("agent", agent_);
    this->get_parameter("step_id", step_id_);
    this->get_parameter("server_wait_seconds", server_wait_seconds_);
    this->get_parameter("fake_status_delay_seconds", fake_status_delay_seconds_);
    this->get_parameter("next_test_delay_seconds", next_test_delay_seconds_);
    this->get_parameter("stop_after_fake_status", stop_after_fake_status_);

    action_client_ = rclcpp_action::create_client<ExecuteTask>(
      this,
      "/executor/execute_task");
    fake_status_pub_ = this->create_publisher<actionlib_msgs::msg::GoalStatus>(
      "/navigation/status",
      rclcpp::QoS(10));
    stop_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/navigation/navigation_ctrl/stop",
      rclcpp::QoS(10));

    start_timer_ = this->create_wall_timer(
      200ms,
      std::bind(&SupervisorGoToTestNode::start_tests, this));
  }

private:
  struct TestCase
  {
    std::string name;
    std::string location;
    bool publish_fake_status{false};
    int fake_status{actionlib_msgs::msg::GoalStatus::ABORTED};
    std::string fake_status_text;
  };

  static const char * task_result_name(uint8_t status)
  {
    switch (status) {
      case executor_interfaces::msg::TaskResult::SUCCESS:
        return "SUCCESS";
      case executor_interfaces::msg::TaskResult::FAILURE:
        return "FAILURE";
      case executor_interfaces::msg::TaskResult::RUNNING:
        return "RUNNING";
      case executor_interfaces::msg::TaskResult::ABORTED:
        return "ABORTED";
      default:
        return "UNKNOWN";
    }
  }

  static const char * goal_status_name(int status)
  {
    switch (status) {
      case actionlib_msgs::msg::GoalStatus::PENDING:
        return "PENDING";
      case actionlib_msgs::msg::GoalStatus::ACTIVE:
        return "ACTIVE";
      case actionlib_msgs::msg::GoalStatus::PREEMPTED:
        return "PREEMPTED";
      case actionlib_msgs::msg::GoalStatus::SUCCEEDED:
        return "SUCCEEDED";
      case actionlib_msgs::msg::GoalStatus::ABORTED:
        return "ABORTED";
      case actionlib_msgs::msg::GoalStatus::REJECTED:
        return "REJECTED";
      case actionlib_msgs::msg::GoalStatus::PREEMPTING:
        return "PREEMPTING";
      case actionlib_msgs::msg::GoalStatus::RECALLING:
        return "RECALLING";
      case actionlib_msgs::msg::GoalStatus::RECALLED:
        return "RECALLED";
      case actionlib_msgs::msg::GoalStatus::LOST:
        return "LOST";
      default:
        return "UNKNOWN";
    }
  }

  static int parse_goal_status(const std::string & name)
  {
    if (name == "PENDING") return actionlib_msgs::msg::GoalStatus::PENDING;
    if (name == "ACTIVE") return actionlib_msgs::msg::GoalStatus::ACTIVE;
    if (name == "PREEMPTED") return actionlib_msgs::msg::GoalStatus::PREEMPTED;
    if (name == "SUCCEEDED") return actionlib_msgs::msg::GoalStatus::SUCCEEDED;
    if (name == "ABORTED") return actionlib_msgs::msg::GoalStatus::ABORTED;
    if (name == "REJECTED") return actionlib_msgs::msg::GoalStatus::REJECTED;
    if (name == "PREEMPTING") return actionlib_msgs::msg::GoalStatus::PREEMPTING;
    if (name == "RECALLING") return actionlib_msgs::msg::GoalStatus::RECALLING;
    if (name == "RECALLED") return actionlib_msgs::msg::GoalStatus::RECALLED;
    if (name == "LOST") return actionlib_msgs::msg::GoalStatus::LOST;
    return actionlib_msgs::msg::GoalStatus::ABORTED;
  }

  void start_tests()
  {
    start_timer_->cancel();

    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for /executor/execute_task action server...");

    const auto wait = std::chrono::duration<double>(server_wait_seconds_);
    if (!action_client_->wait_for_action_server(
        std::chrono::duration_cast<std::chrono::nanoseconds>(wait)))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Action server /executor/execute_task not available after %.2f s",
        server_wait_seconds_);
      rclcpp::shutdown();
      return;
    }

    build_test_cases();
    if (tests_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No tests configured for scenario='%s'", scenario_.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Configured scenario='%s' with %zu test(s). Fake terminal-status tests publish on /navigation/status.",
      scenario_.c_str(),
      tests_.size());

    send_next_goal();
  }

  void build_test_cases()
  {
    tests_.clear();

    if (scenario_ == "missing_location") {
      tests_.push_back({"missing_location", "fake_place", false, 0, ""});
      return;
    }

    if (scenario_ == "empty_location") {
      tests_.push_back({"empty_location", "", false, 0, ""});
      return;
    }

    if (scenario_ == "timeout") {
      tests_.push_back({"timeout", location_, false, 0, ""});
      return;
    }

    if (scenario_ == "terminal_status") {
      const int status = parse_goal_status(fake_status_);
      tests_.push_back({
        "terminal_" + std::string(goal_status_name(status)),
        terminal_location_,
        true,
        status,
        fake_status_text_});
      return;
    }

    if (scenario_ == "terminal_suite") {
      add_terminal_case(actionlib_msgs::msg::GoalStatus::ABORTED);
      add_terminal_case(actionlib_msgs::msg::GoalStatus::REJECTED);
      add_terminal_case(actionlib_msgs::msg::GoalStatus::PREEMPTED);
      add_terminal_case(actionlib_msgs::msg::GoalStatus::RECALLED);
      add_terminal_case(actionlib_msgs::msg::GoalStatus::LOST);
      return;
    }

    if (scenario_ == "nav_controller_error_suite") {
      // End-to-end error propagation suite for executor_tools::GoTo.
      //
      // empty_location and missing_location are generated before a navigation
      // goal is published. The terminal_* cases need nav_controller_node to
      // reach its "waiting for /navigation/status" state. For deterministic
      // terminal-status tests, start nav_controller_node with:
      //
      //   -p enable_test_location_bypass:=true
      //   -p test_location_name:=__nav_test__
      //   -p test_location_publish_goal:=false
      //
      // That prevents the real planner from publishing a natural ABORTED or
      // SUCCEEDED before this node injects the requested terminal status.
      tests_.push_back({"empty_location", "", false, 0, ""});
      tests_.push_back({"missing_location", "fake_place", false, 0, ""});
      add_terminal_case(actionlib_msgs::msg::GoalStatus::ABORTED);
      add_terminal_case(actionlib_msgs::msg::GoalStatus::REJECTED);
      add_terminal_case(actionlib_msgs::msg::GoalStatus::PREEMPTED);
      add_terminal_case(actionlib_msgs::msg::GoalStatus::RECALLED);
      add_terminal_case(actionlib_msgs::msg::GoalStatus::LOST);
      return;
    }

    tests_.push_back({"single", location_, false, 0, ""});
  }

  void add_terminal_case(int status)
  {
    const std::string status_name = goal_status_name(status);
    tests_.push_back({
      "terminal_" + status_name,
      terminal_location_,
      true,
      status,
      "Injected " + status_name + " from supervisor_goto_test_node"});
  }

  void send_next_goal()
  {
    if (current_test_index_ >= tests_.size()) {
      RCLCPP_INFO(this->get_logger(), "All GoTo error-message tests finished");
      rclcpp::shutdown();
      return;
    }

    const auto & test = tests_[current_test_index_];

    ExecuteTask::Goal goal;
    goal.task.step_id = static_cast<uint32_t>(step_id_ + static_cast<int>(current_test_index_));
    goal.task.agent = agent_;
    goal.task.action = action_;
    goal.task.args = std::vector<std::string>{test.location};

    rclcpp_action::Client<ExecuteTask>::SendGoalOptions options;
    options.goal_response_callback =
      std::bind(&SupervisorGoToTestNode::goal_response_cb, this, std::placeholders::_1);
    options.feedback_callback =
      std::bind(
        &SupervisorGoToTestNode::feedback_cb,
        this,
        std::placeholders::_1,
        std::placeholders::_2);
    options.result_callback =
      std::bind(&SupervisorGoToTestNode::result_cb, this, std::placeholders::_1);

    RCLCPP_INFO(
      this->get_logger(),
      "[%zu/%zu] Sending test '%s': step_id=%u action='%s' location='%s'",
      current_test_index_ + 1,
      tests_.size(),
      test.name.c_str(),
      goal.task.step_id,
      action_.c_str(),
      test.location.c_str());

    if (test.publish_fake_status) {
      RCLCPP_WARN(
        this->get_logger(),
        "This test will inject navigation status %s after %.2f s. Use with care while the robot is moving.",
        goal_status_name(test.fake_status),
        fake_status_delay_seconds_);
    }

    action_client_->async_send_goal(goal, options);
  }

  void goal_response_cb(const GoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Executor rejected test goal");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Executor accepted test goal");

    const auto & test = tests_[current_test_index_];
    if (test.publish_fake_status) {
      const auto delay = std::chrono::duration<double>(fake_status_delay_seconds_);
      fake_status_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(delay),
        std::bind(&SupervisorGoToTestNode::publish_fake_status_once, this));
    }
  }

  void feedback_cb(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const ExecuteTask::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Feedback: step_id=%u message='%s'",
      feedback->feedback.step_id,
      feedback->feedback.message.c_str());
  }

  void result_cb(const GoalHandle::WrappedResult & result)
  {
    if (fake_status_timer_) {
      fake_status_timer_->cancel();
    }

    const auto & task_result = result.result->result;

    RCLCPP_INFO(
      this->get_logger(),
      "Action code=%d result_status=%s message='%s'",
      static_cast<int>(result.code),
      task_result_name(task_result.status),
      task_result.message.c_str());

    ++current_test_index_;
    const auto delay = std::chrono::duration<double>(next_test_delay_seconds_);
    next_test_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(delay),
      std::bind(&SupervisorGoToTestNode::send_next_goal_from_timer, this));
  }

  void send_next_goal_from_timer()
  {
    next_test_timer_->cancel();
    send_next_goal();
  }

  void publish_fake_status_once()
  {
    fake_status_timer_->cancel();

    const auto & test = tests_[current_test_index_];

    actionlib_msgs::msg::GoalStatus status;
    status.goal_id.id = "supervisor_goto_test_" + test.name;
    status.goal_id.stamp = this->now();
    status.status = static_cast<int8_t>(test.fake_status);
    status.text = test.fake_status_text;
    fake_status_pub_->publish(status);

    RCLCPP_WARN(
      this->get_logger(),
      "Injected /navigation/status %s: '%s'",
      goal_status_name(test.fake_status),
      test.fake_status_text.c_str());

    if (stop_after_fake_status_) {
      std_msgs::msg::Bool stop;
      stop.data = true;
      stop_pub_->publish(stop);
      RCLCPP_WARN(this->get_logger(), "Published /navigation/navigation_ctrl/stop=true");
    }
  }

  rclcpp_action::Client<ExecuteTask>::SharedPtr action_client_;
  rclcpp::Publisher<actionlib_msgs::msg::GoalStatus>::SharedPtr fake_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::TimerBase::SharedPtr fake_status_timer_;
  rclcpp::TimerBase::SharedPtr next_test_timer_;

  std::vector<TestCase> tests_;
  size_t current_test_index_{0};

  std::string location_;
  std::string terminal_location_;
  std::string scenario_;
  std::string fake_status_;
  std::string fake_status_text_;
  std::string action_;
  std::string agent_;
  int step_id_{999};
  double server_wait_seconds_{5.0};
  double fake_status_delay_seconds_{1.0};
  double next_test_delay_seconds_{1.0};
  bool stop_after_fake_status_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SupervisorGoToTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
