#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_interfaces/msg/task_result.hpp"

using namespace std::chrono_literals;

class SupervisorTestNode : public rclcpp::Node
{
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTask>;

  SupervisorTestNode()
  : Node("supervisor_test_node")
  {
    this->declare_parameter<std::string>("tool", "check");
    this->declare_parameter<std::string>("scenario", "single");
    this->declare_parameter<std::string>("arg", "");
    this->declare_parameter<std::string>("agent", "Justina");
    this->declare_parameter<int>("step_id", 2000);
    this->declare_parameter<double>("server_wait_seconds", 5.0);
    this->declare_parameter<double>("next_test_delay_seconds", 1.0);

    this->get_parameter("tool", tool_);
    this->get_parameter("scenario", scenario_);
    this->get_parameter("arg", arg_);
    this->get_parameter("agent", agent_);
    this->get_parameter("step_id", step_id_);
    this->get_parameter("server_wait_seconds", server_wait_seconds_);
    this->get_parameter("next_test_delay_seconds", next_test_delay_seconds_);

    action_client_ = rclcpp_action::create_client<ExecuteTask>(
      this,
      "/executor/execute_task");

    start_timer_ = this->create_wall_timer(
      200ms,
      std::bind(&SupervisorTestNode::start_tests, this));
  }

private:
  struct TestCase
  {
    std::string name;
    std::string action;
    std::vector<std::string> args;
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
      RCLCPP_ERROR(
        this->get_logger(),
        "No tests configured for tool='%s' scenario='%s'",
        tool_.c_str(),
        scenario_.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Configured tool='%s' scenario='%s' with %zu test(s). This node reports the current real executor feedback/results.",
      tool_.c_str(),
      scenario_.c_str(),
      tests_.size());

    send_next_goal();
  }

  void build_test_cases()
  {
    tests_.clear();

    if (tool_ == "all") {
      add_basic_failure_suite();
      return;
    }

    if (scenario_ == "single") {
      add_single_case(tool_, default_arg_for(tool_));
      return;
    }

    if (tool_ == "check") {
      build_check_cases();
      return;
    }

    if (tool_ == "point") {
      build_point_cases();
      return;
    }

    if (tool_ == "give") {
      build_give_cases();
      return;
    }

    if (tool_ == "receive") {
      build_receive_cases();
      return;
    }

    if (tool_ == "find_object") {
      build_find_object_cases();
      return;
    }

    if (tool_ == "find_person") {
      build_find_person_cases();
      return;
    }

    if (tool_ == "go_to") {
      build_go_to_cases();
      return;
    }
  }

  std::string default_arg_for(const std::string & tool) const
  {
    if (!arg_.empty()) {
      return arg_;
    }

    if (tool == "check") return "door";
    if (tool == "point") return "invalid_argument";
    if (tool == "give") return "object";
    if (tool == "receive") return "object";
    if (tool == "find_object") return "fake_object";
    if (tool == "find_person") return "fake_person";
    if (tool == "go_to") return "fake_place";
    return "";
  }

  void add_single_case(const std::string & action, const std::string & arg)
  {
    tests_.push_back({"single_" + action, action, std::vector<std::string>{arg}});
  }

  void add_case(const std::string & name, const std::string & action, const std::string & arg)
  {
    tests_.push_back({name, action, std::vector<std::string>{arg}});
  }

  void build_check_cases()
  {
    if (scenario_ == "suite" || scenario_ == "basic_failure_suite") {
      add_case("check_door", "check", default_arg_for("check"));
      return;
    }

    if (scenario_ == "door") {
      add_case("check_door", "check", default_arg_for("check"));
    }
  }

  void build_point_cases()
  {
    if (scenario_ == "suite") {
      add_case("point_unknown_argument", "point", "invalid_argument");
      add_case("point_enable", "point", "enable");
      add_case("point_disable", "point", "disable");
      return;
    }

    if (scenario_ == "unknown_argument") {
      add_case("point_unknown_argument", "point", arg_.empty() ? "invalid_argument" : arg_);
      return;
    }

    if (scenario_ == "enable") {
      add_case("point_enable", "point", "enable");
      return;
    }

    if (scenario_ == "disable") {
      add_case("point_disable", "point", "disable");
    }
  }

  void build_give_cases()
  {
    if (scenario_ == "suite") {
      add_case("give_object", "give", default_arg_for("give"));
      add_case("give_empty_arg", "give", "");
      return;
    }

    if (scenario_ == "empty_arg") {
      add_case("give_empty_arg", "give", "");
    }
  }

  void build_receive_cases()
  {
    if (scenario_ == "suite") {
      add_case("receive_object", "receive", default_arg_for("receive"));
      add_case("receive_empty_arg", "receive", "");
      return;
    }

    if (scenario_ == "empty_arg") {
      add_case("receive_empty_arg", "receive", "");
    }
  }

  void build_find_object_cases()
  {
    if (scenario_ == "suite") {
      add_case("find_object_not_found", "find_object", default_arg_for("find_object"));
      add_case("find_empty_chair", "find_object", "empty_chair");
      return;
    }

    if (scenario_ == "not_found") {
      add_case("find_object_not_found", "find_object", default_arg_for("find_object"));
      return;
    }

    if (scenario_ == "empty_chair") {
      add_case("find_empty_chair", "find_object", "empty_chair");
    }
  }

  void build_find_person_cases()
  {
    if (scenario_ == "suite") {
      add_case("find_person_not_found", "find_person", default_arg_for("find_person"));
      add_case("find_anybody", "find_person", "anybody");
      return;
    }

    if (scenario_ == "not_found") {
      add_case("find_person_not_found", "find_person", default_arg_for("find_person"));
      return;
    }

    if (scenario_ == "anybody") {
      add_case("find_anybody", "find_person", "anybody");
    }
  }

  void build_go_to_cases()
  {
    if (scenario_ == "missing_location") {
      add_case("go_to_missing_location", "go_to", default_arg_for("go_to"));
    }
  }

  void add_basic_failure_suite()
  {
    add_case("check_door", "check", default_arg_for("check"));
    add_case("point_unknown_argument", "point", "invalid_argument");
    add_case("find_object_not_found", "find_object", default_arg_for("find_object"));
    add_case("find_person_not_found", "find_person", default_arg_for("find_person"));
    add_case("go_to_missing_location", "go_to", default_arg_for("go_to"));
  }

  void send_next_goal()
  {
    if (current_test_index_ >= tests_.size()) {
      RCLCPP_INFO(this->get_logger(), "All supervisor tool tests finished");
      rclcpp::shutdown();
      return;
    }

    const auto & test = tests_[current_test_index_];

    ExecuteTask::Goal goal;
    goal.task.step_id = static_cast<uint32_t>(step_id_ + static_cast<int>(current_test_index_));
    goal.task.agent = agent_;
    goal.task.action = test.action;
    goal.task.args = test.args;

    rclcpp_action::Client<ExecuteTask>::SendGoalOptions options;
    options.goal_response_callback =
      std::bind(&SupervisorTestNode::goal_response_cb, this, std::placeholders::_1);
    options.feedback_callback =
      std::bind(
        &SupervisorTestNode::feedback_cb,
        this,
        std::placeholders::_1,
        std::placeholders::_2);
    options.result_callback =
      std::bind(&SupervisorTestNode::result_cb, this, std::placeholders::_1);

    RCLCPP_INFO(
      this->get_logger(),
      "[%zu/%zu] Sending test '%s': step_id=%u action='%s' args=[%s]",
      current_test_index_ + 1,
      tests_.size(),
      test.name.c_str(),
      goal.task.step_id,
      test.action.c_str(),
      join_args(test.args).c_str());

    action_client_->async_send_goal(goal, options);
  }

  std::string join_args(const std::vector<std::string> & args) const
  {
    std::string joined;
    for (size_t i = 0; i < args.size(); ++i) {
      if (i > 0) joined += ", ";
      joined += "'";
      joined += args[i];
      joined += "'";
    }
    return joined;
  }

  void goal_response_cb(const GoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Executor rejected test goal");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Executor accepted test goal");
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
    const auto & task_result = result.result->result;

    RCLCPP_INFO(
      this->get_logger(),
      "Action code=%d result_step_id=%u result_status=%s message='%s'",
      static_cast<int>(result.code),
      task_result.step_id,
      task_result_name(task_result.status),
      task_result.message.c_str());

    ++current_test_index_;
    const auto delay = std::chrono::duration<double>(next_test_delay_seconds_);
    next_test_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(delay),
      std::bind(&SupervisorTestNode::send_next_goal_from_timer, this));
  }

  void send_next_goal_from_timer()
  {
    next_test_timer_->cancel();
    send_next_goal();
  }

  rclcpp_action::Client<ExecuteTask>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::TimerBase::SharedPtr next_test_timer_;

  std::vector<TestCase> tests_;
  size_t current_test_index_{0};

  std::string tool_;
  std::string scenario_;
  std::string arg_;
  std::string agent_;
  int step_id_{2000};
  double server_wait_seconds_{5.0};
  double next_test_delay_seconds_{1.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SupervisorTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
