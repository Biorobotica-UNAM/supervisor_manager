#include <rclcpp/rclcpp.hpp>

#include <supervisor_interfaces/msg/system_status.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mutex>
#include <optional>
#include <string>

#include <planning_tools/Planning.hpp>
#include "supervisor_executor_interfaces/srv/authorize_task.hpp"

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

    // --- Publishers ---
    input_trigger_pub_   = this->create_publisher<std_msgs::msg::Empty>("/input_trigger", 10);
    instruction_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "/supervisor/supervisor_manager/instruction_command", 10);

    // --- Clients ---
    start_planning_client_ =
      this->create_client<std_srvs::srv::Trigger>("/planning/clips/start_planning");

    // --- Service server ---
    authorize_task_srv_ = this->create_service<supervisor_executor_interfaces::srv::AuthorizeTask>(
      "/supervisor/supervisor_executor/authorize",
      std::bind(
        &SupervisorManagerNode::authorizeTaskCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );

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
    SM_POST_PLAN_TRIGGER,    // publish /input_trigger (one-shot)

    SM_WAIT_AUTHORIZE        // wait for authorize service call
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

  void authorizeTaskCallback(
    const supervisor_executor_interfaces::srv::AuthorizeTask::Request::SharedPtr req,
    supervisor_executor_interfaces::srv::AuthorizeTask::Response::SharedPtr res)
  {
    // Default: no bloquear por ahora
    res->approved = true;
    res->message  = "ok";

    // Guardamos "evento" para FSM, NO publicamos triggers aquí (evita encadenamientos raros)
    std::lock_guard<std::mutex> lock(mtx_);
    last_authorize_req_ = *req;
    got_authorize_ = true;
  }

  // ===== Helpers =====
  bool getLatestStatus(supervisor_interfaces::msg::SystemStatus & out)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!got_status_ || !last_status_.has_value()) return false;
    out = *last_status_;
    return true;
  }

  bool consumeAuthorize(supervisor_executor_interfaces::srv::AuthorizeTask::Request & out)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!got_authorize_ || !last_authorize_req_.has_value()) return false;
    out = *last_authorize_req_;
    got_authorize_ = false;
    last_authorize_req_.reset();
    return true;
  }

  void publishInputTriggerOnce()
  {
    input_trigger_pub_->publish(std_msgs::msg::Empty{});
  }

  void publishRetryCommand()
  {
    std_msgs::msg::Int32 msg;
    msg.data = 1;
    instruction_cmd_pub_->publish(msg);
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
        start_planning_future_ = start_planning_client_->async_send_request(req);
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
        RCLCPP_INFO(this->get_logger(), "SM_POST_PLAN_TRIGGER: publishing /input_trigger (one-shot) -> WAIT_AUTHORIZE");
        publishInputTriggerOnce();
        RCLCPP_INFO(this->get_logger(), "aquiiii");
        state_ = State::SM_WAIT_AUTHORIZE;
        break;
      }

      case State::SM_WAIT_AUTHORIZE:
      {
        RCLCPP_INFO(this->get_logger(), "SM_WAIT_AUTHORIZE: waiting for authorize_task...");
        supervisor_executor_interfaces::srv::AuthorizeTask::Request req;
        if (!consumeAuthorize(req))
        {
          RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "SM_WAIT_AUTHORIZE: waiting for author111."
          );
          break;
        }

        // --- Aquí sí va tu lógica de Check ---
        if (req.action == "Check")
        {
          if (req.worker_result == "success")
          {
            RCLCPP_INFO(this->get_logger(), "authorize(Check): success -> publish /input_trigger and keep waiting");
            publishInputTriggerOnce();          // ✅ 1 vez por evento
            state_ = State::SM_WAIT_AUTHORIZE;  // sigue esperando el siguiente authorize
            break;
          }

          if (req.worker_result == "failure")
          {
            RCLCPP_WARN(this->get_logger(), "authorize(Check): failure -> publish retry=1 and /input_trigger");
            publishRetryCommand();
            publishInputTriggerOnce();          // ✅ 1 vez por evento
            state_ = State::SM_WAIT_AUTHORIZE;
            break;
          }

          RCLCPP_WARN(this->get_logger(),
                      "authorize(Check): unknown worker_result='%s' (no trigger)",
                      req.worker_result.c_str());
          state_ = State::SM_WAIT_AUTHORIZE;
          break;
        }

        // no es Check
        RCLCPP_DEBUG(this->get_logger(),
                     "authorize: action='%s' (no special logic, no trigger)",
                     req.action.c_str());
        state_ = State::SM_WAIT_AUTHORIZE;
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
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr input_trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr instruction_cmd_pub_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_planning_client_;
  std::shared_future<std::shared_ptr<std_srvs::srv::Trigger::Response>> start_planning_future_;
  bool start_planning_sent_{false};

  rclcpp::Service<supervisor_executor_interfaces::srv::AuthorizeTask>::SharedPtr authorize_task_srv_;

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

  // Authorize event (store last request)
  std::optional<supervisor_executor_interfaces::srv::AuthorizeTask::Request> last_authorize_req_;
  bool got_authorize_{false};
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