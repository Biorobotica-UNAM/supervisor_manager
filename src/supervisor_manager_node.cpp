#include <rclcpp/rclcpp.hpp>

#include <supervisor_interfaces/msg/system_status.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mutex>
#include <optional>
#include <string>

#include <planning_tools/Planning.hpp>

using namespace std::chrono_literals;

class SupervisorManagerNode : public rclcpp::Node
{
public:
  SupervisorManagerNode() : Node("supervisor_manager_node")
  {
    RCLCPP_INFO(this->get_logger(), "supervisor_manager_node: started");

    // --- Subscribers ---
    sub_status_ = this->create_subscription<supervisor_interfaces::msg::SystemStatus>(
      "/system/status",
      rclcpp::QoS(10),
      std::bind(&SupervisorManagerNode::statusCallback, this, std::placeholders::_1)
    );

    execution_feedback_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/supervisor/supervisor_execution/result",
      rclcpp::QoS(10),
      std::bind(&SupervisorManagerNode::executionFeedbackCallback, this, std::placeholders::_1)
    );
    plan_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/supervisor/supervisor_plan/active",
      rclcpp::QoS(10),
      std::bind(&SupervisorManagerNode::planActiveCallback, this, std::placeholders::_1)
    );

    // --- Publishers ---
    allow_next_instruction_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/supervisor/supervisor_manager/allow_next_instruction", 10);
    allow_plan_instruction_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/supervisor/supervisor_manager/allow_plan_instruction", 10);
    instruction_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "/supervisor/supervisor_manager/instruction_command", 10);

    // --- Clients ---
    start_planning_client_ =
      this->create_client<std_srvs::srv::Trigger>("/planning/clips/start_planning");

    // --- FSM Timer (10 Hz) ---
    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&SupervisorManagerNode::tick, this)
    );

    // --- One-shot init for Planning tool ---
    init_timer_ = this->create_wall_timer(
      0ms,
      std::bind(&SupervisorManagerNode::initPlanningOnce, this)
    );
  }

private:
  enum class State : uint8_t
  {
    SM_INIT,
    SM_VERIFY_SYS,

    SM_CREATE_PLAN,          // parse + primitiveToClips
    SM_SEND_START_PLANNING,  // send /planning/clips/start_planning (once)
    SM_WAIT_PLAN_RESPONSE,   // wait for response (non-blocking)
    SM_POST_PLAN_TRIGGER,      // allow execution to request first instruction
    SM_WAIT_EXECUTION_RESULT   // manager waits execution feedback and decides next move
  };

  // ===== One-shot init =====
  void initPlanningOnce()
  {
    init_timer_->cancel();
    planning_ = std::make_shared<Planning>(this->shared_from_this());
    RCLCPP_INFO(this->get_logger(), "Planning tool initialized");
  }

  // ===== Callbacks =====
  void statusCallback(const supervisor_interfaces::msg::SystemStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    last_status_ = *msg;
    got_status_ = true;
  }

  void executionFeedbackCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    last_execution_feedback_ = msg->data;
    got_execution_feedback_ = true;
  }

  void planActiveCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    plan_active_ = msg->data;
    got_plan_active_ = true;
  }

  // ===== Helpers =====
  bool getLatestStatus(supervisor_interfaces::msg::SystemStatus & out)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!got_status_ || !last_status_.has_value()) return false;
    out = *last_status_;
    return true;
  }

  bool consumeExecutionFeedback(std::string & out)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!got_execution_feedback_ || !last_execution_feedback_.has_value()) {
      return false;
    }
    out = *last_execution_feedback_;
    got_execution_feedback_ = false;
    last_execution_feedback_.reset();
    return true;
  }

  void publishAllowNextCycle()
  {
    allow_plan_instruction_pub_->publish(std_msgs::msg::Empty{});
    allow_next_instruction_pub_->publish(std_msgs::msg::Empty{});
    RCLCPP_INFO(
      this->get_logger(),
      "Manager semaphore: allow one plan dispatch + allow execution trigger"
    );
  }

  bool getPlanActiveStatus(bool & out)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!got_plan_active_) {
      return false;
    }
    out = plan_active_;
    return true;
  }

  void publishRetryCommand()
  {
    std_msgs::msg::Int32 msg;
    msg.data = 1;
    instruction_cmd_pub_->publish(msg);
  }

  bool parseExecutionFeedback(
    const std::string & feedback,
    std::string & action,
    std::string & result)
  {
    const auto sep = feedback.find(':');
    if (sep == std::string::npos || sep == 0U || sep == feedback.size() - 1U) {
      return false;
    }
    action = feedback.substr(0, sep);
    result = feedback.substr(sep + 1);
    return true;
  }

  // ===== FSM Tick =====
  void tick()
  {
    switch (state_)
    {
      case State::SM_INIT:
      {
        RCLCPP_INFO(this->get_logger(), "SM_INIT: Init");
        RCLCPP_INFO(this->get_logger(), "Waiting messages on /system/status ...");
        state_ = State::SM_VERIFY_SYS;
        break;
      }

      case State::SM_VERIFY_SYS:
      {
        supervisor_interfaces::msg::SystemStatus st;
        if (!getLatestStatus(st))
        {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "SM_VERIFY_SYS: No message received yet on /system/status"
          );
          break;
        }

        if (st.hardware && st.hri && st.navigation)
        {
          RCLCPP_INFO(this->get_logger(),
                      "SM_VERIFY_SYS: hardware/hri/navigation OK -> SM_CREATE_PLAN");
          state_ = State::SM_CREATE_PLAN;
        }
        else
        {
          RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1500,
            "SM_VERIFY_SYS: waiting... hw=%s hri=%s nav=%s",
            st.hardware ? "true" : "false",
            st.hri ? "true" : "false",
            st.navigation ? "true" : "false"
          );
        }
        break;
      }

      case State::SM_CREATE_PLAN:
      {
        RCLCPP_INFO(this->get_logger(), "SM_CREATE_PLAN: parsing semantic command");

        const std::string cmd =
          "Go to the entrance and wait for the door to be oppened and navigate to living room "
          "and wait until you listen continue and navigate to the entrance";

        std::string plan = planning_->parseSemanticCommand(cmd);

        if (plan.empty())
        {
          RCLCPP_ERROR(this->get_logger(), "SM_CREATE_PLAN: semantic parser returned empty plan -> back to VERIFY");
          state_ = State::SM_VERIFY_SYS;
          break;
        }

        RCLCPP_INFO(this->get_logger(), "Plan semantic parsed: %s", plan.c_str());

        planning_->primitiveToClips(plan);
        RCLCPP_INFO(this->get_logger(), "Plan sent to CLIPS (primitive_to_clips)");

        state_ = State::SM_SEND_START_PLANNING;
        break;
      }

      case State::SM_SEND_START_PLANNING:
      {
        if (!start_planning_client_->service_is_ready())
        {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "SM_SEND_START_PLANNING: waiting for /planning/clips/start_planning..."
          );
          break;
        }

        RCLCPP_INFO(this->get_logger(), "SM_SEND_START_PLANNING: calling /planning/clips/start_planning");

        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future_and_id = start_planning_client_->async_send_request(req);
        start_planning_future_ = future_and_id.future.share();
        start_planning_sent_ = true;

        state_ = State::SM_WAIT_PLAN_RESPONSE;
        break;
      }

      case State::SM_WAIT_PLAN_RESPONSE:
      {
        if (!start_planning_sent_ || !start_planning_future_.valid())
        {
          RCLCPP_WARN(this->get_logger(),
                      "SM_WAIT_PLAN_RESPONSE: no future (start_planning not sent). Returning to SEND_START_PLANNING.");
          state_ = State::SM_SEND_START_PLANNING;
          break;
        }

        if (start_planning_future_.wait_for(0s) == std::future_status::ready)
        {
          auto resp = start_planning_future_.get();
          start_planning_sent_ = false;

          if (resp->success)
          {
            RCLCPP_INFO(this->get_logger(),
                        "SM_WAIT_PLAN_RESPONSE: plan created OK: %s", resp->message.c_str());
            state_ = State::SM_POST_PLAN_TRIGGER;
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(),
                        "SM_WAIT_PLAN_RESPONSE: planning failed: %s", resp->message.c_str());
            state_ = State::SM_VERIFY_SYS;
          }
        }
        else
        {
          RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "SM_WAIT_PLAN_RESPONSE: waiting response from /planning/clips/start_planning..."
          );
        }
        break;
      }

      case State::SM_POST_PLAN_TRIGGER:
      {
        RCLCPP_INFO(
          this->get_logger(),
          "SM_POST_PLAN_TRIGGER: plan ready -> allow execution to request first instruction"
        );
        publishAllowNextCycle();
        state_ = State::SM_WAIT_EXECUTION_RESULT;
        break;
      }

      case State::SM_WAIT_EXECUTION_RESULT:
      {
        std::string feedback;
        if (!consumeExecutionFeedback(feedback))
        {
          RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "SM_WAIT_EXECUTION_RESULT: waiting execution feedback..."
          );
          break;
        }

        std::string action;
        std::string result;
        if (!parseExecutionFeedback(feedback, action, result))
        {
          RCLCPP_WARN(
            this->get_logger(),
            "SM_WAIT_EXECUTION_RESULT: malformed feedback='%s'",
            feedback.c_str()
          );
          break;
        }

        RCLCPP_INFO(
          this->get_logger(),
          "SM_WAIT_EXECUTION_RESULT: feedback action='%s' result='%s'",
          action.c_str(), result.c_str()
        );

        if (action == "Check" && result == "failure")
        {
          RCLCPP_WARN(
            this->get_logger(),
            "SM_WAIT_EXECUTION_RESULT: Check failure -> retry current instruction"
          );
          publishRetryCommand();
        }

        if (result == "success" || result == "failure")
        {
          bool active_plan = false;
          if (getPlanActiveStatus(active_plan))
          {
            RCLCPP_INFO(
              this->get_logger(),
              "SM_WAIT_EXECUTION_RESULT: plan_active=%s before next cycle",
              active_plan ? "true" : "false"
            );
          }
          else
          {
            RCLCPP_WARN(
              this->get_logger(),
              "SM_WAIT_EXECUTION_RESULT: plan_active status not received yet"
            );
          }

          publishAllowNextCycle();
          break;
        }

        RCLCPP_WARN(
          this->get_logger(),
          "SM_WAIT_EXECUTION_RESULT: result='%s' not actionable",
          result.c_str()
        );
        break;
      }

      default:
      {
        RCLCPP_ERROR(this->get_logger(), "Unknown state, resetting FSM");
        state_ = State::SM_INIT;
        break;
      }
    }
  }

  // ===== ROS =====
  rclcpp::Subscription<supervisor_interfaces::msg::SystemStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr execution_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr plan_active_sub_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr allow_next_instruction_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr allow_plan_instruction_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr instruction_cmd_pub_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_planning_client_;
  std::shared_future<std::shared_ptr<std_srvs::srv::Trigger::Response>> start_planning_future_;
  bool start_planning_sent_{false};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  // Planning tool
  std::shared_ptr<Planning> planning_;

  // FSM
  State state_{State::SM_INIT};

  // Shared state
  std::mutex mtx_;
  std::optional<supervisor_interfaces::msg::SystemStatus> last_status_;
  bool got_status_{false};

  std::optional<std::string> last_execution_feedback_;
  bool got_execution_feedback_{false};
  bool plan_active_{false};
  bool got_plan_active_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SupervisorManagerNode>();

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(),
    4
  );

  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
