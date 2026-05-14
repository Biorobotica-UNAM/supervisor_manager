#include <rclcpp/rclcpp.hpp>

#include <supervisor_interfaces/msg/system_status.hpp>
#include <supervisor_interfaces/msg/task_execution_report.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <known_locations_interfaces/srv/locations_get.hpp>
#include <listener_interfaces/srv/listen.hpp>
#include <talker_interfaces/srv/talk.hpp>
#include <planning_tools/Planning.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class SupervisorManagerNode : public rclcpp::Node
{
public:
  explicit SupervisorManagerNode(bool natural_language_planning)
  : Node("supervisor_manager_node"),
    natural_language_planning_(natural_language_planning)
  {
    RCLCPP_INFO(this->get_logger(), "supervisor_manager_node: started");
    RCLCPP_INFO(
      this->get_logger(),
      "natural_language_planning=%s",
      natural_language_planning_ ? "true" : "false");

    // --- Subscribers ---
    sub_status_ = this->create_subscription<supervisor_interfaces::msg::SystemStatus>(
      "/system/status",
      rclcpp::QoS(10),
      std::bind(&SupervisorManagerNode::statusCallback, this, std::placeholders::_1)
    );

    execution_feedback_sub_ =
      this->create_subscription<supervisor_interfaces::msg::TaskExecutionReport>(
      "/supervisor/supervisor_execution/result",
      rclcpp::QoS(10),
      std::bind(&SupervisorManagerNode::executionFeedbackCallback, this, std::placeholders::_1)
    );

    plan_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/supervisor/supervisor_plan/active",
      rclcpp::QoS(10),
      std::bind(&SupervisorManagerNode::planActiveCallback, this, std::placeholders::_1)
    );

    plan_ready_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/supervisor/supervisor_plan/plan_ready",
      rclcpp::QoS(10),
      std::bind(&SupervisorManagerNode::planReadyCallback, this, std::placeholders::_1)
    );

    // --- Publishers ---
	rewind_pub_ = create_publisher<std_msgs::msg::Empty>("/supervisor/supervisor_plan/rewind_one", 10);

    allow_next_instruction_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/supervisor/supervisor_manager/allow_next_instruction", 10);
    allow_plan_instruction_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/supervisor/supervisor_manager/allow_plan_instruction", 10);
    instruction_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "/supervisor/supervisor_manager/instruction_command", 10);
	corrective_instruction_pub_ = this->create_publisher<std_msgs::msg::String>(
	  "/supervisor/supervisor_manager/instruction", 10);

    hri_client_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    talker_client_ = this->create_client<talker_interfaces::srv::Talk>(
      "/hri/talker/talk",
      rmw_qos_profile_services_default,
      hri_client_group_);
    listener_client_ = this->create_client<listener_interfaces::srv::Listen>(
      "/hri/listener/listen",
      rmw_qos_profile_services_default,
      hri_client_group_);
    goto_client_ = this->create_client<known_locations_interfaces::srv::LocationsGet>(
      "/nav_controller/go_to_location",
      rmw_qos_profile_services_default,
      hri_client_group_);

    // --- FSM Timer (10 Hz) ---
    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&SupervisorManagerNode::tick, this)
    );

    // --- One-shot init for tools needing shared_from_this() ---
    init_timer_ = this->create_wall_timer(
      0ms,
      std::bind(&SupervisorManagerNode::initToolsOnce, this)
    );
  }

private:
  enum class State : uint8_t
  {
    SM_INIT,
    SM_VERIFY_SYS,

    SM_CREATE_PLAN,
    SM_WAIT_PLAN_READY,
    SM_POST_PLAN_TRIGGER,
    SM_WAIT_EXECUTION_RESULT,

    SM_FINISH
  };

  // ===== One-shot init =====
  void initToolsOnce()
  {
    init_timer_->cancel();

    auto node_ptr = this->shared_from_this();
    planning_ = std::make_shared<Planning>(node_ptr);

    RCLCPP_INFO(this->get_logger(), "Planning tool initialized");
  }

  // ===== Pipeline helpers =====
  bool goToLocation(const std::string & location_name)
  {
    if (!goto_client_) {
      RCLCPP_ERROR(this->get_logger(), "[Pipeline][Goto] GoTo client not initialized");
      return false;
    }

    if (!goto_client_->wait_for_service(500ms)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[Pipeline][Goto] Service /nav_controller/go_to_location unavailable"
      );
      return false;
    }

    auto request = std::make_shared<known_locations_interfaces::srv::LocationsGet::Request>();
    request->location_name = location_name;

    auto future = goto_client_->async_send_request(request);
    const auto status = future.wait_for(120s);
    if (status != std::future_status::ready) {
      RCLCPP_WARN(
        this->get_logger(),
        "[Pipeline][Goto] Timeout waiting for '%s'",
        location_name.c_str()
      );
      return false;
    }

    const auto response = future.get();
    if (!response->success) {
      RCLCPP_WARN(
        this->get_logger(),
        "[Pipeline][Goto] Navigation to '%s' failed: %s",
        location_name.c_str(),
        response->message.c_str()
      );
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[Pipeline][Goto] Arrived at '%s': %s",
      location_name.c_str(),
      response->message.c_str()
    );
    return true;
  }

  bool goToInstructionPoint()
  {
    RCLCPP_INFO(this->get_logger(), "[Pipeline][Step -1] Navigating to instruction_point...");
    return goToLocation("instruction_point");
  }

  bool sayText(const std::string & text)
  {
    if (!talker_client_) {
      RCLCPP_ERROR(this->get_logger(), "[Pipeline][Say] Talk client not initialized");
      return false;
    }

    if (!talker_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "[Pipeline][Say] Talk service unavailable");
      return false;
    }

    auto request = std::make_shared<talker_interfaces::srv::Talk::Request>();
    request->text = text;

    auto future = talker_client_->async_send_request(request);
    const auto status = future.wait_for(10s);
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "[Pipeline][Say] Talk service timed out");
      return false;
    }

    const auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[Pipeline][Say] TTS error: %s",
        response->message.c_str()
      );
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "[Pipeline][Say] TTS OK: %s", response->message.c_str());
    return true;
  }

  bool promptForCommand()
  {
    RCLCPP_INFO(this->get_logger(), "[Pipeline][Step 0] Prompting user for a command...");
    return sayText("Please tell me your command.");
  }

  std::string stepListen()
  {
    RCLCPP_INFO(this->get_logger(), "[Pipeline][Step 1] Listening...");
    if (!listener_client_) {
      RCLCPP_ERROR(this->get_logger(), "[Pipeline][Step 1] Listen client not initialized");
      return "";
    }

    if (!listener_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "[Pipeline][Step 1] Listen service unavailable");
      return "";
    }

    auto request = std::make_shared<listener_interfaces::srv::Listen::Request>();
    request->enable = true;
    request->grammar = "";

    auto future = listener_client_->async_send_request(request);
    const auto status = future.wait_for(10s);
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "[Pipeline][Step 1] Listen timeout");
      return "";
    }

    const auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(this->get_logger(), "[Pipeline][Step 1] Listen failed");
      return "";
    }

    const std::string text = response->recognized_speech;

    if (text.empty()) {
      RCLCPP_ERROR(this->get_logger(), "[Pipeline][Step 1] Listen returned empty");
      return "";
    }

    RCLCPP_INFO(this->get_logger(), "[Pipeline][Step 1] Recognised: \"%s\"", text.c_str());
    return text;
  }

  std::string stepParse(const std::string & text)
  {
    RCLCPP_INFO(this->get_logger(), "[Pipeline][Step 2] Parsing: \"%s\"", text.c_str());
    const std::string semantic = planning_->parseSemanticCommand(text);

    if (semantic.empty()) {
      RCLCPP_ERROR(this->get_logger(), "[Pipeline][Step 2] Semantic parser returned empty");
      return "";
    }

    RCLCPP_INFO(this->get_logger(), "[Pipeline][Step 2] Semantic CD: \"%s\"", semantic.c_str());
    return semantic;
  }

  std::string stepToClips(const std::string & semantic)
  {
    RCLCPP_INFO(this->get_logger(), "[Pipeline][Step 3] Converting to CLIPS: \"%s\"", semantic.c_str());
    const std::string clips = planning_->primitiveToClips(semantic);

    if (clips.empty()) {
      RCLCPP_ERROR(this->get_logger(), "[Pipeline][Step 3] primitiveToClips returned empty");
      return "";
    }

    RCLCPP_INFO(this->get_logger(), "[Pipeline][Step 3] CLIPS facts: \"%s\"", clips.c_str());
    return clips;
  }

  bool runNaturalLanguagePipeline()
  {
    if (!planning_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "runNaturalLanguagePipeline: planning tool not initialized yet");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "──────────────────────────────────────");
    RCLCPP_INFO(this->get_logger(), "Natural language pipeline started");

    sayText("Im ready for GPSR task, ill navigate to the intruction point, please open the door.");

    if (!goToInstructionPoint()) {
      RCLCPP_ERROR(this->get_logger(), "Pipeline failed at step -1: goToInstructionPoint failed");
      return false;
    }

    if (!promptForCommand()) {
      RCLCPP_ERROR(this->get_logger(), "Pipeline failed at step 0: promptForCommand failed");
      return false;
    }

    const std::string text = stepListen();
    if (text.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Pipeline failed at step 1: Listen returned empty");
      return false;
    }

    const std::string semantic = stepParse(text);
    if (semantic.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Pipeline failed at step 2: parseSemanticCommand returned empty");
      return false;
    }

    clearPlanReadySignal();

    const std::string clips = stepToClips(semantic);
    if (clips.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Pipeline failed at step 3: primitiveToClips returned empty");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Natural language pipeline completed successfully");
    RCLCPP_INFO(this->get_logger(), "──────────────────────────────────────");
    return true;
  }

  // ===== Callbacks =====
  void statusCallback(const supervisor_interfaces::msg::SystemStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    last_status_ = *msg;
    got_status_ = true;
  }

  void executionFeedbackCallback(
    const supervisor_interfaces::msg::TaskExecutionReport::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    last_execution_feedback_ = *msg;
    got_execution_feedback_ = true;
  }

  void planActiveCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    plan_active_ = msg->data;
    got_plan_active_ = true;
  }

  void planReadyCallback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    (void)msg;
    std::lock_guard<std::mutex> lock(mtx_);
    plan_ready_signal_ = true;
  }

  // ===== Helpers =====
  bool getLatestStatus(supervisor_interfaces::msg::SystemStatus & out)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!got_status_ || !last_status_.has_value()) return false;
    out = *last_status_;
    return true;
  }

  bool consumeExecutionFeedback(supervisor_interfaces::msg::TaskExecutionReport & out)
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
    {
      std::lock_guard<std::mutex> lock(mtx_);
      got_plan_active_ = false;
    }
    allow_plan_instruction_pub_->publish(std_msgs::msg::Empty{});
    allow_next_instruction_pub_->publish(std_msgs::msg::Empty{});
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

  void clearPlanReadySignal()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    plan_ready_signal_ = false;
  }

  bool consumePlanReadySignal()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!plan_ready_signal_) {
      return false;
    }
    plan_ready_signal_ = false;
    return true;
  }

  void publishRetryCommand()
  {
    std_msgs::msg::Int32 msg;
    msg.data = 1;
    instruction_cmd_pub_->publish(msg);
  }

  bool needsApprovalRetry(const std::string & action) const
  {
    std::string normalized = action;
    std::transform(
      normalized.begin(),
      normalized.end(),
      normalized.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});

    return (normalized == "check" ||
            normalized == "wait_for" ||
            normalized == "listen");
  }

  // AGREGAR junto a needsApprovalRetry():
  std::string getCorrectiveInstruction(
    const std::string & action,
    const std::string & argument) const
  {
    if (action == "listen" || action == "listen_for") {
      if (argument == "name")     return "Justina say repeat_name";
      if (argument == "drink")    return "Justina say repeat_drink";
      return "Justina say please_repeat";   // fallback genérico
    }
    return "";
  }

  // ===== FSM Tick =====
  void tick()
  {
    switch (state_)
    {
      case State::SM_INIT:
      {
        RCLCPP_INFO(this->get_logger(), "SM_INIT: Init");
        RCLCPP_INFO(this->get_logger(), "SM_INIT: Waiting messages on /system/status ...");
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
        if (!planning_) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "SM_CREATE_PLAN: Planning tool not initialized yet");
          break;
        }

        if (natural_language_planning_)
        {
          RCLCPP_INFO(this->get_logger(), "SM_CREATE_PLAN: running natural language pipeline");

          if (!runNaturalLanguagePipeline())
          {
            RCLCPP_ERROR(
              this->get_logger(),
              "SM_CREATE_PLAN: natural language pipeline failed -> back to VERIFY"
            );
            state_ = State::SM_VERIFY_SYS;
            break;
          }
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "SM_CREATE_PLAN: starting planning directly (startPlanning)");

          clearPlanReadySignal();

          if (!planning_->startPlanning())
          {
            RCLCPP_ERROR(
              this->get_logger(),
              "SM_CREATE_PLAN: startPlanning failed -> back to VERIFY"
            );
            state_ = State::SM_VERIFY_SYS;
            break;
          }

          RCLCPP_INFO(this->get_logger(), "SM_CREATE_PLAN: startPlanning request sent successfully");
        }

        state_ = State::SM_WAIT_PLAN_READY;
        break;
      }

      case State::SM_WAIT_PLAN_READY:
      {
        if (!consumePlanReadySignal())
        {
          RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "SM_WAIT_PLAN_READY: waiting for /supervisor/supervisor_plan/plan_ready..."
          );
          break;
        }

        RCLCPP_INFO(
          this->get_logger(),
          "SM_WAIT_PLAN_READY: supervisor_plan confirmed plan is ready"
        );
        state_ = State::SM_POST_PLAN_TRIGGER;
        break;
      }

      case State::SM_POST_PLAN_TRIGGER:
      {
        RCLCPP_INFO(this->get_logger(),
                    "SM_POST_PLAN_TRIGGER: plan ready.");
        RCLCPP_INFO(this->get_logger(),
                    "SM_POST_PLAN_TRIGGER: Manager semaphore -> allow one plan dispatch + allow execution trigger");
        publishAllowNextCycle();
        state_ = State::SM_WAIT_EXECUTION_RESULT;
        break;
      }

      case State::SM_WAIT_EXECUTION_RESULT:
      {
        supervisor_interfaces::msg::TaskExecutionReport feedback;
        if (!consumeExecutionFeedback(feedback))
        {
          RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "SM_WAIT_EXECUTION_RESULT: waiting execution feedback..."
          );
          break;
        }

        RCLCPP_INFO(
          this->get_logger(),
          "SM_WAIT_EXECUTION_RESULT: feedback action='%s' arg='%s' worker='%s' approved=%s status=%u reason='%s'",
          feedback.action.c_str(),
          feedback.argument.c_str(),
          feedback.worker_result.c_str(),
          feedback.approved ? "true" : "false",
          static_cast<unsigned int>(feedback.status),
          feedback.failure_reason.c_str()
        );

        if (feedback.action != "listen") {
			corrective_pending_ = false;
		}

        // En el bloque de retry:
		if (needsApprovalRetry(feedback.action) && !feedback.approved)
		{
		  if (!corrective_pending_) {
			  const std::string corrective =
				getCorrectiveInstruction(feedback.action, feedback.argument);
			  if (!corrective.empty()) {
				// Rewind para que el listen se reintente después del say
				rewind_pub_->publish(std_msgs::msg::Empty{});
				
				std_msgs::msg::String inj;
				inj.data = corrective;
				corrective_instruction_pub_->publish(inj);
				corrective_pending_ = true;
			  }
			}
			publishAllowNextCycle();
			break;
		}

        if (feedback.status == supervisor_interfaces::msg::TaskExecutionReport::SUCCESS)
        {
          bool active_plan = false;
          if (getPlanActiveStatus(active_plan))
          {
            RCLCPP_INFO(
              this->get_logger(),
              "SM_WAIT_EXECUTION_RESULT: plan_active=%s before next decision",
              active_plan ? "true" : "false"
            );

            // Si ya no hay plan activo, reinicia el ciclo completo
            // para volver a pedir una nueva tarea/instrucción.
            if (!active_plan)
            {
              RCLCPP_INFO(
                this->get_logger(),
                "SM_WAIT_EXECUTION_RESULT: plan finished -> restarting task request pipeline"
              );

              if(!natural_language_planning_)
              {
                state_ = State::SM_FINISH;
                break;
              }

              state_ = State::SM_CREATE_PLAN;
              break;
            }
          }
          else
          {
            RCLCPP_WARN(
              this->get_logger(),
              "SM_WAIT_EXECUTION_RESULT: plan_active status not received yet; waiting before dispatching next cycle"
            );
            break;
          }

          RCLCPP_INFO(this->get_logger(),
                      "SM_WAIT_EXECUTION_RESULT: Manager semaphore -> allow one plan dispatch + allow execution trigger");
          publishAllowNextCycle();
          break;
        }

        if (feedback.status == supervisor_interfaces::msg::TaskExecutionReport::FAILURE ||
            feedback.status == supervisor_interfaces::msg::TaskExecutionReport::TIMEOUT ||
            feedback.status == supervisor_interfaces::msg::TaskExecutionReport::REJECTED)
        {
          const std::string status_name =
            feedback.status == supervisor_interfaces::msg::TaskExecutionReport::FAILURE ? "FAILURE" :
            feedback.status == supervisor_interfaces::msg::TaskExecutionReport::TIMEOUT ? "TIMEOUT" :
            "REJECTED";

          const std::string reason = feedback.failure_reason.empty()
            ? status_name
            : status_name + " - " + feedback.failure_reason;

          std::string corrective_instruction =
            "Justina say action " + feedback.action + " failed because " + reason;

          std_msgs::msg::String inj;
          inj.data = corrective_instruction;
          corrective_instruction_pub_->publish(inj);

          RCLCPP_ERROR(
            this->get_logger(),
            "SM_WAIT_EXECUTION_RESULT: execution failed (status=%u), injected corrective say and continuing to next instruction",
            static_cast<unsigned int>(feedback.status)
          );
          RCLCPP_INFO(
            this->get_logger(),
            "SM_WAIT_EXECUTION_RESULT: injected corrective instruction='%s'",
            corrective_instruction.c_str()
          );

          publishAllowNextCycle();
          break;
        }

        RCLCPP_WARN(
          this->get_logger(),
          "SM_WAIT_EXECUTION_RESULT: status='%u' not actionable",
          static_cast<unsigned int>(feedback.status)
        );
        RCLCPP_INFO(this->get_logger(),
                    "SM_WAIT_EXECUTION_RESULT: Manager semaphore -> allow one plan dispatch + allow execution trigger");
        publishAllowNextCycle();
        break;
      }

      case State::SM_FINISH:
      {
        RCLCPP_INFO(this->get_logger(),
                    "SM_FINISH: Test finished.");
        rclcpp::shutdown();
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
  rclcpp::Subscription<supervisor_interfaces::msg::TaskExecutionReport>::SharedPtr
    execution_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr plan_active_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr plan_ready_sub_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr rewind_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr allow_next_instruction_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr allow_plan_instruction_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr instruction_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr corrective_instruction_pub_;
  rclcpp::CallbackGroup::SharedPtr hri_client_group_;
  rclcpp::Client<known_locations_interfaces::srv::LocationsGet>::SharedPtr goto_client_;
  rclcpp::Client<talker_interfaces::srv::Talk>::SharedPtr talker_client_;
  rclcpp::Client<listener_interfaces::srv::Listen>::SharedPtr listener_client_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  // Tools
  std::shared_ptr<Planning> planning_;
  bool natural_language_planning_{false};

  // FSM
  State state_{State::SM_INIT};

  // Shared state
  std::mutex mtx_;
  std::optional<supervisor_interfaces::msg::SystemStatus> last_status_;
  bool got_status_{false};
  bool corrective_pending_ = false;

  std::optional<supervisor_interfaces::msg::TaskExecutionReport> last_execution_feedback_;
  bool got_execution_feedback_{false};

  bool plan_active_{false};
  bool got_plan_active_{false};
  bool plan_ready_signal_{false};
};

namespace
{
bool parseBooleanValue(const std::string & value, bool & out)
{
  std::string normalized = value;
  std::transform(
    normalized.begin(),
    normalized.end(),
    normalized.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});

  if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on") {
    out = true;
    return true;
  }

  if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off") {
    out = false;
    return true;
  }

  return false;
}

bool getNaturalLanguagePlanningFromArgv(int argc, char ** argv)
{
  bool value = false;
  bool found = false;

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg(argv[i]);
    const std::string key = "natural_language_planning";

    if (arg.rfind(key + "=", 0) == 0 || arg.rfind(key + ":=", 0) == 0)
    {
      const std::size_t delim_pos = arg.find('=');
      bool parsed = false;
      if (delim_pos != std::string::npos && delim_pos + 1 < arg.size()) {
        parsed = parseBooleanValue(arg.substr(delim_pos + 1), value);
      }
      if (!parsed) {
        RCLCPP_WARN(
          rclcpp::get_logger("supervisor_manager_node"),
          "Invalid value for natural_language_planning: '%s'. Using default false.",
          arg.c_str());
      } else {
        found = true;
      }
      continue;
    }

    if (arg == key && (i + 2) < argc && std::strcmp(argv[i + 1], "=") == 0)
    {
      if (!parseBooleanValue(argv[i + 2], value)) {
        RCLCPP_WARN(
          rclcpp::get_logger("supervisor_manager_node"),
          "Invalid value for natural_language_planning: '%s'. Using default false.",
          argv[i + 2]);
      } else {
        found = true;
      }
      i += 2;
    }
  }

  return found ? value : false;
}
}  // namespace

int main(int argc, char ** argv)
{
  const bool natural_language_planning = getNaturalLanguagePlanningFromArgv(argc, argv);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SupervisorManagerNode>(natural_language_planning);

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(),
    4
  );

  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
