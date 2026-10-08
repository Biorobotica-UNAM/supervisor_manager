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
#include <talker_interfaces/srv/talk.hpp>
#include <planning_tools/Planning.hpp>

#include "supervisor_manager/recovery_coordinator.hpp"
#include "supervisor_manager/plan_sequence_tools.hpp"
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

using namespace std::chrono_literals;

class SupervisorManagerNode : public rclcpp::Node
{
public:
  explicit SupervisorManagerNode(
    bool natural_language_planning,
    std::optional<bool> ask_data_confirmation_override = std::nullopt)
  : Node("supervisor_manager_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    natural_language_planning_(natural_language_planning)
  {
    RCLCPP_INFO(this->get_logger(), "supervisor_manager_node: started");
    RCLCPP_INFO(
      this->get_logger(),
      "natural_language_planning=%s",
      natural_language_planning_ ? "true" : "false");

    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<double>("go_to_success_distance_tolerance_m", 0.75);
    this->declare_parameter<bool>("ask_data_confirmation_enabled", false);
    this->declare_parameter<bool>("continue_on_recovery_failure", true);
    RCLCPP_INFO(get_logger(), "continue_on_recovery_failure=%s",
      get_parameter("continue_on_recovery_failure").as_bool() ? "true" : "false");
    this->declare_parameter<double>("pnp_time_limit_sec", 420.0);
    this->declare_parameter<double>("pnp_retry_min_remaining_sec", 60.0);
    this->declare_parameter<double>("pnp_safety_margin_sec", 25.0);

    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter(
      "go_to_success_distance_tolerance_m",
      go_to_success_distance_tolerance_m_);
    this->get_parameter(
      "ask_data_confirmation_enabled",
      ask_data_confirmation_enabled_);
    this->get_parameter("pnp_time_limit_sec", pnp_time_limit_sec_);
    this->get_parameter(
      "pnp_retry_min_remaining_sec",
      pnp_retry_min_remaining_sec_);
    this->get_parameter("pnp_safety_margin_sec", pnp_safety_margin_sec_);


    if (ask_data_confirmation_override.has_value()) {
      ask_data_confirmation_enabled_ = *ask_data_confirmation_override;
      this->set_parameter(
        rclcpp::Parameter(
          "ask_data_confirmation_enabled",
          ask_data_confirmation_enabled_));
    }

    RCLCPP_INFO(
      this->get_logger(),
      "ask_data_confirmation_enabled=%s",
      ask_data_confirmation_enabled_ ? "true" : "false");
    RCLCPP_INFO(
      this->get_logger(),
      "pnp_time_limit_sec=%.1f retry_min_remaining_sec=%.1f safety_margin_sec=%.1f",
      pnp_time_limit_sec_,
      pnp_retry_min_remaining_sec_,
      pnp_safety_margin_sec_);

    this->declare_parameter<std::string>("navigation_recovery_config",
      ament_index_cpp::get_package_share_directory("supervisor_manager") +
      "/config/navigation_recovery.yaml");
    this->declare_parameter<std::string>("navigation_locations_service", "/locations_manager/get");

    this->declare_parameter<std::string>("manipulation_recovery_config",
      ament_index_cpp::get_package_share_directory("supervisor_manager") +
      "/config/manipulation_recovery.yaml");

    // --- Entradas ROS ---
    // /system/status indica si los modulos minimos estan disponibles.
    sub_status_ = this->create_subscription<supervisor_interfaces::msg::SystemStatus>(
      "/system/status",
      rclcpp::QoS(10),
      std::bind(&SupervisorManagerNode::statusCallback, this, std::placeholders::_1)
    );

    // Resultado validado por supervisor_execution despues de que executor_worker
    // termina una accion.
    execution_feedback_sub_ =
      this->create_subscription<supervisor_interfaces::msg::TaskExecutionReport>(
      "/supervisor/supervisor_execution/result",
      rclcpp::QoS(10),
      std::bind(&SupervisorManagerNode::executionFeedbackCallback, this, std::placeholders::_1)
    );

    // Estado del plan: true si aun quedan instrucciones por ejecutar.
    plan_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/supervisor/supervisor_plan/active",
      rclcpp::QoS(10),
      std::bind(&SupervisorManagerNode::planActiveCallback, this, std::placeholders::_1)
    );

    // Pulso enviado por supervisor_plan cuando ya parseo el plan publicado por CLIPS.
    plan_ready_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/supervisor/supervisor_plan/plan_ready",
      rclcpp::QoS(10),
      std::bind(&SupervisorManagerNode::planReadyCallback, this, std::placeholders::_1)
    );

    // Solo se usa para observabilidad: registra cual instruccion despacho el plan.
    current_instruction_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/instruction_msg",
      rclcpp::QoS(10),
      std::bind(&SupervisorManagerNode::currentInstructionCallback, this, std::placeholders::_1)
    );

    // --- Salidas ROS ---
    // El manager prepara el plan por servicio y libera la ejecucion por pulso.
    allow_next_instruction_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/supervisor/supervisor_manager/allow_next_instruction", 10);
    // Clientes usados por el pipeline opcional de lenguaje natural y por los
    // mensajes hablados de error.
    hri_client_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    prepare_cycle_client_ = this->create_client<clips_interfaces::srv::String>(
      "/supervisor/supervisor_plan/prepare_cycle",
      rmw_qos_profile_services_default, hri_client_group_);
    talker_client_ = this->create_client<talker_interfaces::srv::Talk>(
      "/hri/talker/talk",
      rmw_qos_profile_services_default,
      hri_client_group_);
    listener_client_ = this->create_client<listener_interfaces::srv::Listen>(
      "/hri/listener/listen",
      rmw_qos_profile_services_default,
      hri_client_group_);
    locations_validation_client_ = this->create_client<known_locations_interfaces::srv::LocationsGet>(
      get_parameter("navigation_locations_service").as_string(),
      rmw_qos_profile_services_default, hri_client_group_);
    RCLCPP_INFO(get_logger(), "Navigation recovery validates destinations using '%s'",
      get_parameter("navigation_locations_service").as_string().c_str());
    goto_client_ = this->create_client<known_locations_interfaces::srv::LocationsGet>(
      "/nav_controller/go_to_location",
      rmw_qos_profile_services_default,
      hri_client_group_);
    head_client_ = this->create_client<head_interfaces::srv::MoveHead>(
      "/hardware/head/move_head",
      rmw_qos_profile_services_default,
      hri_client_group_);

    this->declare_parameter<std::string>("recovery_modules_config",
      ament_index_cpp::get_package_share_directory("supervisor_manager") + "/config/recovery_modules.yaml");
    recovery_context_ = std::make_unique<supervisor_manager::RecoveryContext>();
    auto & context = *recovery_context_;
    context.config = YAML::LoadFile(get_parameter("recovery_modules_config").as_string());
    context.node = this; context.tf = &tf_buffer_;
    context.talker = talker_client_; context.listener = listener_client_;
    context.entities = this->create_client<semantic_parser_interfaces::srv::ExtractEntity>(
      "/planning/semantic_parser/extract_entity", rmw_qos_profile_services_default,
      hri_client_group_);
    context.go_to = goto_client_; context.locations = locations_validation_client_; context.head = head_client_;
    context.natural_language = natural_language_planning_;
    context.ask_confirmation = ask_data_confirmation_enabled_;
    context.map_frame = map_frame_; context.base_frame = base_frame_;
    context.arrival_tolerance = go_to_success_distance_tolerance_m_;
    context.pnp.configure(pnp_time_limit_sec_, pnp_retry_min_remaining_sec_, pnp_safety_margin_sec_);
    context.allow_next = [this]() {return publishAllowNextCycle();};
    context.retry = [this]() {publishRetryCommand();};
    context.skip = [this](int count) {publishSkipCommand(count);};
    context.insert = [this](const std::string & instruction) {publishOverrideInstruction(instruction);};
    context.rewind = [this]() {plan_tools_.rewind();};
    context.stop = [this]() {state_ = State::SM_FINISH;};
    context.finish_failed = [this](bool can_continue) {finishFailedStep(can_continue);};
    context.continue_on_failure = [this]() {return continueAfterRecoveryFailure();};
    context.current_instruction = [this]() {return getCurrentInstruction();};
    context.plan_active = [this](bool & active) {return getPlanActiveStatus(active);};
    context.clear_plan_ready = [this]() {clearPlanReadySignal();};
    context.has_report = [this]() {std::lock_guard<std::mutex> lock(mtx_); return got_execution_feedback_;};
    recovery_ = std::make_unique<supervisor_manager::RecoveryCoordinator>(context, hri_client_group_);
    RCLCPP_INFO(get_logger(), "Recovery modules loaded from '%s'", get_parameter("recovery_modules_config").as_string().c_str());

    // --- Timer principal del FSM (10 Hz) ---
    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&SupervisorManagerNode::tick, this)
    );

    // --- Inicializacion diferida ---
    // Planning necesita shared_from_this(), por eso se crea despues del constructor.
    init_timer_ = this->create_wall_timer(
      0ms,
      std::bind(&SupervisorManagerNode::initToolsOnce, this)
    );
  }
private:
  enum class State : uint8_t
  {
    // Arranque y verificacion de modulos minimos.
    SM_INIT,
    SM_VERIFY_SYS,

    // Creacion/recepcion del plan antes de ejecutar instrucciones.
    SM_CREATE_PLAN,
    SM_WAIT_PLAN_READY,
    SM_POST_PLAN_TRIGGER,

    // Ciclo de ejecucion: espera resultado y decide si avanza, reintenta o termina.
    SM_WAIT_EXECUTION_RESULT,

    SM_FINISH
  };


  void initToolsOnce()
  {
    init_timer_->cancel();

    auto node_ptr = this->shared_from_this();
    planning_ = std::make_shared<Planning>(node_ptr);

    recovery_context_->planning = planning_;
    RCLCPP_INFO(this->get_logger(), "Planning tool initialized");
  }
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
  void currentInstructionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    current_instruction_ = msg->data;
    got_current_instruction_ = true;
    recovery_->observeInstruction(current_instruction_);

    // /instruction_msg is published by supervisor_plan exactly when the next
    // raw instruction is dispatched to executor_manager.
    RCLCPP_INFO(
      this->get_logger(),
      "CURRENT_INSTRUCTION: '%s'",
      current_instruction_.c_str());
  }
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
    if (!got_plan_active_ || !got_execution_feedback_ || !last_execution_feedback_.has_value()) {
      return false;
    }
    out = *last_execution_feedback_;
    got_execution_feedback_ = false;
    last_execution_feedback_.reset();
    return true;
  }
  bool publishAllowNextCycle()
  {
    // Accept the decision now. tick retries delivery until dependencies exist.
    if (state_ == State::SM_WAIT_EXECUTION_RESULT && plan_tools_.empty()) {
      bool active = true;
      if (getPlanActiveStatus(active) && !active) {
        if (natural_language_planning_) {state_ = State::SM_CREATE_PLAN;}
        else if (!recovery_->planFinished()) {state_ = State::SM_FINISH;}
        return true;
      }
    }
    cycle_pending_ = true;
    return true;
  }
  bool processPendingCycle()
  {
    if (!cycle_pending_) {return false;}
    if (execution_release_pending_) {
      if (allow_next_instruction_pub_->get_subscription_count() == 0) {return true;}
      allow_next_instruction_pub_->publish(std_msgs::msg::Empty{});
      execution_release_pending_ = false;
      cycle_pending_ = false;
      plan_tools_.clear();
      return true;
    }
    if (!prepare_cycle_future_) {
      if (!prepare_cycle_client_->service_is_ready() ||
          allow_next_instruction_pub_->get_subscription_count() == 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Pending dispatch: waiting for plan prepare service and execution subscriber");
        return true;
      }
      auto request = std::make_shared<clips_interfaces::srv::String::Request>();
      request->data = plan_tools_.operations().dump();
      RCLCPP_INFO(get_logger(), "[PlanDispatch] preparing operations=%s", request->data.c_str());
      {
        std::lock_guard<std::mutex> lock(mtx_);
        got_plan_active_ = false;
      }
      prepare_cycle_future_ = prepare_cycle_client_->async_send_request(request).share();
      prepare_cycle_deadline_ = std::chrono::steady_clock::now() + 10s;
      return true;
    }
    if (prepare_cycle_future_->wait_for(0s) != std::future_status::ready) {
      if (std::chrono::steady_clock::now() >= prepare_cycle_deadline_) {
        // Do not resend mutations after an ambiguous timeout (skip/rewind).
        prepare_cycle_client_->prune_pending_requests();
        prepare_cycle_future_.reset();
        cycle_pending_ = false;
        RCLCPP_ERROR(get_logger(), "Plan preparation timed out; stopping task");
        state_ = State::SM_FINISH;
      }
      return true;
    }
    const auto response = prepare_cycle_future_->get();
    prepare_cycle_future_.reset();
    if (response->data != "ready") {
      RCLCPP_ERROR(get_logger(), "Plan rejected cycle: %s", response->data.c_str());
      cycle_pending_ = false;
      state_ = State::SM_FINISH;
      return true;
    }
    RCLCPP_INFO(get_logger(), "[PlanDispatch] plan acknowledged operations; waiting execution release");
    // The plan has applied all mutations and installed its token.
    if (allow_next_instruction_pub_->get_subscription_count() == 0) {
      execution_release_pending_ = true;
      return true;
    }
    allow_next_instruction_pub_->publish(std_msgs::msg::Empty{});
    plan_tools_.clear();
    cycle_pending_ = false;
    return true;
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
  std::string getCurrentInstruction()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return got_current_instruction_ ? current_instruction_ : "";
  }
  void publishRetryCommand()
  {
    // supervisor_plan interpreta command=1 como "reenviar la ultima instruccion".
    std_msgs::msg::Int32 msg;
    msg.data = 1;
    RCLCPP_INFO(get_logger(), "[PlanMutation] queued command=%d current_instruction='%s'",
      msg.data, getCurrentInstruction().c_str());
    plan_tools_.retry();
  }
  void publishSkipCommand(int count)
  {
    if (count <= 0) {
      return;
    }

    // supervisor_plan interpreta valores negativos como "saltar N pasos
    // normales" antes de entregar la siguiente instruccion ejecutable.
    std_msgs::msg::Int32 msg;
    msg.data = -count;
    RCLCPP_INFO(get_logger(), "[PlanMutation] queued command=%d current_instruction='%s'",
      msg.data, getCurrentInstruction().c_str());
    plan_tools_.skip(count);
  }
  void publishOverrideInstruction(const std::string & instruction)
  {
    // Inyecta una instruccion temporal en supervisor_plan. Esa instruccion se
    // ejecuta antes del siguiente paso normal del plan.
    std_msgs::msg::String inj;
    inj.data = instruction;
    RCLCPP_INFO(get_logger(), "[PlanMutation] inserted instruction='%s'", inj.data.c_str());
    plan_tools_.insert(instruction);
  }
  bool continueAfterRecoveryFailure() const
  {
    return get_parameter("continue_on_recovery_failure").as_bool();
  }
  void finishFailedStep(bool can_continue)
  {
    bool active = false;
    const bool known = getPlanActiveStatus(active);
    if (!can_continue || !known || !active) {
      RCLCPP_WARN(get_logger(),
        "[RecoveryOutcome] STOP can_continue=%s plan_active_known=%s plan_active=%s",
        can_continue ? "true" : "false", known ? "true" : "false", active ? "true" : "false");
      state_ = State::SM_FINISH;
      return;
    }
    RCLCPP_WARN(get_logger(), "[RecoveryOutcome] CONTINUE: failed step skipped; resuming original plan");
    publishAllowNextCycle();
  }
  void tick()
  {
    if (processPendingCycle()) {return;}
    if (state_ != State::SM_FINISH && recovery_->tick()) {return;}
    switch (state_)
    {
      case State::SM_INIT:
      {
        // Primer estado: solo anuncia arranque y pasa a esperar /system/status.
        RCLCPP_INFO(this->get_logger(), "SM_INIT: Init");
        RCLCPP_INFO(this->get_logger(), "SM_INIT: Waiting messages on /system/status ...");
        state_ = State::SM_VERIFY_SYS;
        break;
      }

      case State::SM_VERIFY_SYS:
      {
        // No se crea plan hasta que supervisor_system reporte modulos minimos OK.
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
        // Crea el plan. En modo directo pide a planning iniciar; en modo
        // lenguaje natural primero obtiene y convierte una instruccion hablada.
        if (!planning_) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "SM_CREATE_PLAN: Planning tool not initialized yet");
          break;
        }

        if (natural_language_planning_)
        {
          RCLCPP_INFO(this->get_logger(), "SM_CREATE_PLAN: running natural language pipeline");

          if (!recovery_->acquireTask())
          {
            RCLCPP_ERROR(
              this->get_logger(),
              "SM_CREATE_PLAN: natural language pipeline failed -> back to VERIFY"
            );
            state_ = continueAfterRecoveryFailure() ? State::SM_VERIFY_SYS : State::SM_FINISH;
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
        // Espera confirmacion explicita de supervisor_plan para evitar pedir
        // una instruccion antes de que el plan haya sido parseado.
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
        recovery_->reset();
        state_ = State::SM_POST_PLAN_TRIGGER;
        break;
      }

      case State::SM_POST_PLAN_TRIGGER:
      {
        RCLCPP_INFO(this->get_logger(),
                    "SM_POST_PLAN_TRIGGER: plan ready.");
        RCLCPP_INFO(this->get_logger(),
                    "SM_POST_PLAN_TRIGGER: Manager semaphore -> allow one plan dispatch + allow execution trigger");
        // La decision queda pendiente; processPendingCycle espera dependencias
        // y confirmacion del plan antes de liberar la ejecucion.
        if (!publishAllowNextCycle()) {
          break;
        }
        state_ = State::SM_WAIT_EXECUTION_RESULT;
        break;
      }

      case State::SM_WAIT_EXECUTION_RESULT:
      {
        // Consume one result; the coordinator owns all domain decisions.
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

        if (recovery_->handleFeedback(feedback)) {break;}
        publishAllowNextCycle();
        break;
      }

      case State::SM_FINISH:
      {
        RCLCPP_INFO(this->get_logger(),
                    "SM_FINISH: Supervisor stopped. No further plan instructions will be dispatched.");
        rclcpp::shutdown();
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
  std::unique_ptr<supervisor_manager::RecoveryContext> recovery_context_;
  std::unique_ptr<supervisor_manager::RecoveryCoordinator> recovery_;
  supervisor_manager::PlanSequenceTools plan_tools_;
  std::chrono::steady_clock::time_point prepare_cycle_deadline_;
  rclcpp::Client<clips_interfaces::srv::String>::SharedPtr prepare_cycle_client_;
  std::optional<rclcpp::Client<clips_interfaces::srv::String>::SharedFuture> prepare_cycle_future_;
  bool cycle_pending_{false}, execution_release_pending_{false};
  rclcpp::Subscription<supervisor_interfaces::msg::SystemStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<supervisor_interfaces::msg::TaskExecutionReport>::SharedPtr execution_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr plan_active_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr plan_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_instruction_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr allow_next_instruction_pub_;
  rclcpp::CallbackGroup::SharedPtr hri_client_group_;
  rclcpp::Client<known_locations_interfaces::srv::LocationsGet>::SharedPtr goto_client_, locations_validation_client_;
  rclcpp::Client<head_interfaces::srv::MoveHead>::SharedPtr head_client_;
  rclcpp::Client<talker_interfaces::srv::Talk>::SharedPtr talker_client_;
  rclcpp::Client<listener_interfaces::srv::Listen>::SharedPtr listener_client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_, init_timer_;
  std::shared_ptr<Planning> planning_;
  bool natural_language_planning_{false}, ask_data_confirmation_enabled_{false};
  std::string map_frame_{"map"}, base_frame_{"base_link"};
  double go_to_success_distance_tolerance_m_{0.75};
  double pnp_time_limit_sec_{420.0}, pnp_retry_min_remaining_sec_{60.0}, pnp_safety_margin_sec_{25.0};
  State state_{State::SM_INIT};
  std::mutex mtx_;
  std::optional<supervisor_interfaces::msg::SystemStatus> last_status_;
  bool got_status_{false};
  std::optional<supervisor_interfaces::msg::TaskExecutionReport> last_execution_feedback_;
  bool got_execution_feedback_{false};
  bool plan_active_{false}, got_plan_active_{false}, plan_ready_signal_{false};
  std::string current_instruction_;
  bool got_current_instruction_{false};
};

namespace
{
bool parseBooleanValue(const std::string & value, bool & out)
{
  // Parser tolerante para argumentos booleanos de CLI/launch.
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

std::optional<bool> getBooleanArgFromArgv(
  int argc,
  char ** argv,
  const std::string & key)
{
  // Se parsea manualmente porque este nodo puede correrse con ros2 run o desde
  // launch; soporta key=true y key:=true.
  bool value = false;
  bool found = false;

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg(argv[i]);

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
          "Invalid value for %s: '%s'. Ignoring command-line override.",
          key.c_str(),
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
          "Invalid value for %s: '%s'. Ignoring command-line override.",
          key.c_str(),
          argv[i + 2]);
      } else {
        found = true;
      }
      i += 2;
    }
  }

  if (!found) {
    return std::nullopt;
  }

  return value;
}

bool getNaturalLanguagePlanningFromArgv(int argc, char ** argv)
{
  const std::optional<bool> value =
    getBooleanArgFromArgv(argc, argv, "natural_language_planning");
  return value.value_or(false);
}
}  // namespace

int main(int argc, char ** argv)
{
  const bool natural_language_planning = getNaturalLanguagePlanningFromArgv(argc, argv);
  const std::optional<bool> ask_data_confirmation_override =
    getBooleanArgFromArgv(argc, argv, "ask_data_confirmation_enabled");
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SupervisorManagerNode>(
    natural_language_planning,
    ask_data_confirmation_override);

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(),
    4
  );

  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
