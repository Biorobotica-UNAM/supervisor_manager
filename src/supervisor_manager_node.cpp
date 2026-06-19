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
  explicit SupervisorManagerNode(bool natural_language_planning)
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

    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter(
      "go_to_success_distance_tolerance_m",
      go_to_success_distance_tolerance_m_);

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
    // Rewind permite pedirle a supervisor_plan que repita la instruccion anterior.
	rewind_pub_ = create_publisher<std_msgs::msg::Empty>("/supervisor/supervisor_plan/rewind_one", 10);

    // Semaforos de un solo disparo: uno habilita al plan a entregar una
    // instruccion y el otro dispara la ejecucion.
    allow_next_instruction_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/supervisor/supervisor_manager/allow_next_instruction", 10);
    allow_plan_instruction_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/supervisor/supervisor_manager/allow_plan_instruction", 10);

    // Comandos auxiliares para supervisor_plan: por ahora 1 significa retry.
    instruction_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "/supervisor/supervisor_manager/instruction_command", 10);

    // Inyeccion de una instruccion correctiva, por ejemplo pedir repetir un dato.
	corrective_instruction_pub_ = this->create_publisher<std_msgs::msg::String>(
	  "/supervisor/supervisor_manager/instruction", 10);

    // Clientes usados por el pipeline opcional de lenguaje natural y por los
    // mensajes hablados de error.
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
    head_client_ = this->create_client<head_interfaces::srv::MoveHead>(
      "/hardware/head/move_head",
      rmw_qos_profile_services_default,
      hri_client_group_);

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

  enum class FollowGiftState : uint8_t
  {
    // Sub-FSM local para el caso say instruct_follow_person
    // sin consumir pasos fijos del plan CLIPS.
    IDLE,
    WAIT_FOLLOW_ENABLE,
    WAIT_FOLLOW_DISABLE,
    WAIT_DROP_GIFT
  };

  enum class TakeApproachRetryState : uint8_t
  {
    IDLE,
    WAIT_APPROACH,
    WAIT_TAKE_RETRY
  };

  enum class PersonApproachRecoveryState : uint8_t
  {
    IDLE,
    WAIT_PARTIAL_APPROACH,
    WAIT_PREVENTIVE_FIND_POSE,
    WAIT_FIND_POSE,
    WAIT_APPROACH
  };

  struct SemanticValidationResult
  {
    bool valid{true};
    bool unknown_destination{false};
    bool command_aborted{false};
    std::string abort_reason;
    std::vector<std::string> invalid_destinations;
  };

  struct GoToArrivalVerification
  {
    bool applicable{false};
    bool verified{false};
    bool within_tolerance{true};
    double distance_m{0.0};
    std::string goal_frame;
    std::string error_message;
  };

  // ===== One-shot init =====
  void initToolsOnce()
  {
    init_timer_->cancel();

    auto node_ptr = this->shared_from_this();
    planning_ = std::make_shared<Planning>(node_ptr);

    RCLCPP_INFO(this->get_logger(), "Planning tool initialized");
  }

  // ===== Helpers del pipeline opcional de lenguaje natural =====
  // Este bloque solo se usa cuando natural_language_planning_ es true.
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
    const bool arrived = goToLocation("instruction_point");
    if (arrived) {
      at_instruction_point_ = true;
    }
    return arrived;
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

  bool askUserToRepeatCommand(int attempt)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "[Pipeline][Step 0] Asking user to repeat command, retry %d/%d...",
      attempt,
      MAX_NATURAL_LANGUAGE_LISTEN_RETRIES);
    return sayText("I did not understand your command. Please repeat the instruction.");
  }

  bool askUserToRepeatInvalidDestinationCommand(
    const std::string & destination,
    int attempt)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "[Pipeline][Step 0] Asking user to repeat command with valid destination, retry %d/%d...",
      attempt,
      MAX_NATURAL_LANGUAGE_LISTEN_RETRIES);
    return sayText(
      "I understood the command, but " + makeSpeechFriendly(destination) +
      " is not a known destination. Please repeat the command.");
  }

  bool askUserForDifferentCommand(
    const std::string & destination,
    int attempt)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "[Pipeline][Step 0] Asking user for a different command, retry %d/%d...",
      attempt,
      MAX_NATURAL_LANGUAGE_LISTEN_RETRIES);
    return sayText(
      "That command cannot be performed because " + makeSpeechFriendly(destination) +
      " is not a known destination. Please give me a different command.");
  }

  void announceNaturalLanguageSequenceRestart()
  {
    RCLCPP_WARN(
      this->get_logger(),
      "[Pipeline][Step 1] Listen retries exhausted; restarting full instruction sequence");
    sayText(
      "I still could not understand your command. "
      "I will restart the sequence from the beginning and navigate to the instruction point again.");
    at_instruction_point_ = false;
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

  std::string normalizeKnowledgeToken(std::string value) const
  {
    std::transform(
      value.begin(),
      value.end(),
      value.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});

    const auto first = value.find_first_not_of(" \t\n\r");
    if (first == std::string::npos) {
      return "";
    }

    const auto last = value.find_last_not_of(" \t\n\r");
    value = value.substr(first, last - first + 1);

    std::replace(value.begin(), value.end(), ' ', '_');
    std::replace(value.begin(), value.end(), '-', '_');
    return value;
  }

  bool loadKnownLocations()
  {
    if (known_locations_loaded_) {
      return !known_locations_.empty();
    }

    try {
      const std::string bank_path =
        ament_index_cpp::get_package_share_directory("knowledge_bank");
      const std::string locations_path = bank_path + "/domains/locations.yaml";
      const YAML::Node root = YAML::LoadFile(locations_path);

      if (!root.IsMap()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[Pipeline][Step 2] locations.yaml is not a map: %s",
          locations_path.c_str());
        known_locations_loaded_ = true;
        return false;
      }

      for (const auto & category : root) {
        if (category.first.IsScalar()) {
          const std::string location = normalizeKnowledgeToken(category.first.as<std::string>());
          if (!location.empty()) {
            known_locations_.insert(location);
          }
        }

        if (!category.second.IsSequence()) {
          continue;
        }

        for (const auto & item : category.second) {
          if (!item.IsScalar()) {
            continue;
          }

          const std::string location = normalizeKnowledgeToken(item.as<std::string>());
          if (!location.empty()) {
            known_locations_.insert(location);
          }
        }
      }

      known_locations_loaded_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "[Pipeline][Step 2] Loaded %zu known destinations from %s",
        known_locations_.size(),
        locations_path.c_str());
      return !known_locations_.empty();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[Pipeline][Step 2] Could not load knowledge_bank locations: %s",
        ex.what());
      known_locations_loaded_ = true;
      return false;
    }
  }

  bool isSpecialSemanticDestination(const std::string & destination) const
  {
    return destination == "unknown" ||
           destination == "user" ||
           destination == "person_recipient";
  }

  bool textMentionsUnknownDestination(const std::string & text) const
  {
    const std::string normalized = normalizeKnowledgeToken(text);
    return normalized.find("destination") != std::string::npos ||
           normalized.find("location") != std::string::npos;
  }

  bool semanticLooksLikeAbortedCommand(const std::string & semantic) const
  {
    const std::string normalized = normalizeKnowledgeToken(semantic);
    return normalized.rfind("command_aborted", 0U) == 0U;
  }

  std::string commandAbortReasonFromText(const std::string & semantic) const
  {
    const std::size_t separator = semantic.find(':');
    if (separator == std::string::npos || separator + 1 >= semantic.size()) {
      return semantic;
    }

    const std::string reason = semantic.substr(separator + 1);
    const auto first = reason.find_first_not_of(" \t\n\r");
    if (first == std::string::npos) {
      return "";
    }

    const auto last = reason.find_last_not_of(" \t\n\r");
    return reason.substr(first, last - first + 1);
  }

  void markSemanticAbort(
    SemanticValidationResult & result,
    const std::string & reason) const
  {
    result.valid = false;
    result.command_aborted = true;
    result.abort_reason = reason;
    if (textMentionsUnknownDestination(reason)) {
      result.unknown_destination = true;
    }
  }

  bool applySemanticAbortIfPresent(
    const nlohmann::json & value,
    SemanticValidationResult & result) const
  {
    if (!value.is_object()) {
      return false;
    }

    const auto status = value.find("status");
    if (status == value.end() || !status->is_string() ||
        normalizeKnowledgeToken(status->get<std::string>()) != "aborted")
    {
      return false;
    }

    const auto reason = value.find("reason");
    if (reason != value.end() && reason->is_string()) {
      markSemanticAbort(result, reason->get<std::string>());
    } else {
      markSemanticAbort(result, "semantic parser aborted the command");
    }

    return true;
  }

  void validateJsonDestinations(
    const nlohmann::json & value,
    SemanticValidationResult & result)
  {
    if (value.is_object()) {
      const auto destination = value.find("destination");
      if (destination != value.end() && destination->is_string()) {
        const std::string raw_destination = destination->get<std::string>();
        const std::string normalized_destination = normalizeKnowledgeToken(raw_destination);

        if (normalized_destination == "unknown") {
          result.valid = false;
          result.unknown_destination = true;
        } else if (!known_locations_.empty() &&
                   !isSpecialSemanticDestination(normalized_destination) &&
                   known_locations_.find(normalized_destination) == known_locations_.end())
        {
          result.valid = false;
          result.invalid_destinations.push_back(raw_destination);
        }
      }

      for (const auto & item : value.items()) {
        validateJsonDestinations(item.value(), result);
      }
      return;
    }

    if (value.is_array()) {
      for (const auto & item : value) {
        validateJsonDestinations(item, result);
      }
    }
  }

  SemanticValidationResult validateSemanticDestinations(const std::string & semantic)
  {
    SemanticValidationResult result;

    if (semanticLooksLikeAbortedCommand(semantic)) {
      markSemanticAbort(result, commandAbortReasonFromText(semantic));
      return result;
    }

    try {
      const auto parsed = nlohmann::json::parse(semantic);
      if (applySemanticAbortIfPresent(parsed, result)) {
        return result;
      }

      const bool can_validate_locations = loadKnownLocations();
      validateJsonDestinations(parsed, result);

      if (!can_validate_locations) {
        RCLCPP_WARN(
          this->get_logger(),
          "[Pipeline][Step 2] Destination domain unavailable; semantic destination validation is partial");
      }
    } catch (const nlohmann::json::exception & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "[Pipeline][Step 2] Could not parse semantic JSON for destination validation: %s",
        ex.what());
      markSemanticAbort(result, "semantic parser returned a non JSON response");
    }

    return result;
  }

  std::string firstInvalidDestination(const SemanticValidationResult & validation) const
  {
    if (validation.invalid_destinations.empty()) {
      return "the requested place";
    }

    return validation.invalid_destinations.front();
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
    // Pipeline GPSR interactivo:
    // 1) ir al punto de instrucciones,
    // 2) pedir comando,
    // 3) escuchar,
    // 4) parsear a representacion semantica,
    // 5) convertir a hechos CLIPS para que planning publique un plan.
    if (!planning_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "runNaturalLanguagePipeline: planning tool not initialized yet");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "──────────────────────────────────────");
    RCLCPP_INFO(this->get_logger(), "Natural language pipeline started");

    if (!at_instruction_point_) {
      sayText("Im ready for GPSR task, ill navigate to the instruction point, please open the door.");

      if (!goToInstructionPoint()) {
        RCLCPP_ERROR(this->get_logger(), "Pipeline failed at step -1: goToInstructionPoint failed");
        return false;
      }
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "[Pipeline][Step -1] Already at instruction_point; skipping intro and navigation");
    }

    if (!promptForCommand()) {
      RCLCPP_ERROR(this->get_logger(), "Pipeline failed at step 0: promptForCommand failed");
      return false;
    }

    std::string semantic;
    int invalid_destination_attempts = 0;
    for (int retry = 0; retry <= MAX_NATURAL_LANGUAGE_LISTEN_RETRIES; ++retry) {
      enum class RetryPrompt : uint8_t {RepeatCommand, RepeatInvalidDestination, DifferentCommand};
      RetryPrompt retry_prompt = RetryPrompt::RepeatCommand;
      std::string invalid_destination;

      const std::string text = stepListen();
      if (text.empty()) {
        RCLCPP_WARN(
          this->get_logger(),
          "Pipeline listen attempt %d/%d failed",
          retry + 1,
          MAX_NATURAL_LANGUAGE_LISTEN_RETRIES + 1);
      } else {
        semantic = stepParse(text);
        if (semantic.empty()) {
          RCLCPP_WARN(
            this->get_logger(),
            "Pipeline semantic parse attempt %d/%d failed",
            retry + 1,
            MAX_NATURAL_LANGUAGE_LISTEN_RETRIES + 1);
        } else {
          const SemanticValidationResult validation = validateSemanticDestinations(semantic);
          if (!validation.valid) {
            if (validation.command_aborted) {
              RCLCPP_WARN(
                this->get_logger(),
                "[Pipeline][Step 2] Semantic parser aborted command: %s",
                validation.abort_reason.empty()
                  ? "unknown validation error"
                  : validation.abort_reason.c_str());
            }

            if (validation.unknown_destination) {
              RCLCPP_WARN(
                this->get_logger(),
                "[Pipeline][Step 2] Semantic destination is unknown; asking for instruction again");
            }

            if (!validation.invalid_destinations.empty()) {
              ++invalid_destination_attempts;
              invalid_destination = firstInvalidDestination(validation);
              retry_prompt = invalid_destination_attempts >= 2
                ? RetryPrompt::DifferentCommand
                : RetryPrompt::RepeatInvalidDestination;
              RCLCPP_WARN(
                this->get_logger(),
                "[Pipeline][Step 2] Semantic destination '%s' is not in knowledge_bank/domains/locations.yaml; attempt %d",
                invalid_destination.c_str(),
                invalid_destination_attempts);
            }

            semantic.clear();
          }
        }
      }

      if (!semantic.empty()) {
        break;
      }

      if (retry < MAX_NATURAL_LANGUAGE_LISTEN_RETRIES) {
        bool prompt_ok = false;
        switch (retry_prompt) {
          case RetryPrompt::RepeatInvalidDestination:
            prompt_ok = askUserToRepeatInvalidDestinationCommand(
              invalid_destination,
              retry + 1);
            break;
          case RetryPrompt::DifferentCommand:
            prompt_ok = askUserForDifferentCommand(
              invalid_destination,
              retry + 1);
            break;
          case RetryPrompt::RepeatCommand:
          default:
            prompt_ok = askUserToRepeatCommand(retry + 1);
            break;
        }

        if (!prompt_ok) {
          RCLCPP_ERROR(this->get_logger(), "Pipeline failed at step 0: retry prompt failed");
          return false;
        }
      }
    }

    if (semantic.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Pipeline failed before step 3: no valid command after %d retries",
        MAX_NATURAL_LANGUAGE_LISTEN_RETRIES);
      announceNaturalLanguageSequenceRestart();
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

  // ===== Callbacks ROS =====
  // Los callbacks solo actualizan estado compartido; la logica de decision vive
  // en tick(), lo que mantiene el flujo del supervisor en un solo lugar.
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

    // /instruction_msg is published by supervisor_plan exactly when the next
    // raw instruction is dispatched to executor_manager.
    RCLCPP_INFO(
      this->get_logger(),
      "CURRENT_INSTRUCTION: '%s'",
      current_instruction_.c_str());
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

  bool publishAllowNextCycle()
  {
    // Evita perder el semaforo de un solo disparo si algun nodo aun no descubre
    // sus subscribers; en ese caso el FSM permanece en el estado actual y reintenta.
    const size_t plan_subscribers =
      allow_plan_instruction_pub_->get_subscription_count();
    const size_t execution_subscribers =
      allow_next_instruction_pub_->get_subscription_count();

    if (plan_subscribers == 0 || execution_subscribers == 0)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Manager semaphore not sent: waiting subscribers "
        "allow_plan_instruction=%zu allow_next_instruction=%zu",
        plan_subscribers,
        execution_subscribers);
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(mtx_);
      got_plan_active_ = false;
    }

    // Give supervisor_plan its dispatch token before triggering executor_manager
    // to request the next instruction. Publishing both topics back-to-back can
    // race: execution may call /request_instruction before the plan node has
    // processed allow_plan_instruction.
    allow_plan_instruction_pub_->publish(std_msgs::msg::Empty{});
    scheduleExecutionTrigger();
    return true;
  }

  void scheduleExecutionTrigger()
  {
    if (execution_trigger_timer_) {
      execution_trigger_timer_->cancel();
      execution_trigger_timer_.reset();
    }

    execution_trigger_timer_ = this->create_wall_timer(
      50ms,
      [this]()
      {
        allow_next_instruction_pub_->publish(std_msgs::msg::Empty{});
        if (execution_trigger_timer_) {
          execution_trigger_timer_->cancel();
          execution_trigger_timer_.reset();
        }
      });
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
    instruction_cmd_pub_->publish(msg);
  }

  void publishOverrideInstruction(const std::string & instruction)
  {
    // Inyecta una instruccion temporal en supervisor_plan. Esa instruccion se
    // ejecuta antes del siguiente paso normal del plan.
    std_msgs::msg::String inj;
    inj.data = instruction;
    corrective_instruction_pub_->publish(inj);
  }

  bool dispatchOverrideInstruction(const std::string & instruction)
  {
    // Publica el override y libera exactamente un ciclo plan->executor.
    publishOverrideInstruction(instruction);
    return publishAllowNextCycle();
  }

  bool isSuccessfulFeedback(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    return feedback.approved &&
           feedback.status == supervisor_interfaces::msg::TaskExecutionReport::SUCCESS;
  }

  bool feedbackMatches(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback,
    const std::string & action,
    const std::string & argument) const
  {
    return normalizeText(feedback.action) == action &&
           normalizeText(feedback.argument) == argument;
  }

  bool answerHasStop(const std::string & text) const
  {
    return normalizeText(text).find("stop") != std::string::npos;
  }

  bool answerHasFollowOrContinue(const std::string & text) const
  {
    // La primera respuesta acepta follow o continue como permiso para activar
    // seguimiento humano.
    const std::string answer = normalizeText(text);
    return answer.find("follow") != std::string::npos ||
           answer.find("continue") != std::string::npos;
  }

  std::string listenFollowGiftAnswer()
  {
    // Escucha directa del manager: no inyecta "Justina listen answer" al plan,
    // y evita depender de extractEntity(answer) en executor_tools/Listen.
    if (!listener_client_) {
      RCLCPP_ERROR(this->get_logger(), "[FollowGift] Listen client not initialized");
      return "";
    }

    if (!listener_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "[FollowGift] Listen service unavailable");
      return "";
    }

    auto request = std::make_shared<listener_interfaces::srv::Listen::Request>();
    request->enable = true;
    request->grammar = "";

    auto future = listener_client_->async_send_request(request);
    const auto status = future.wait_for(20s);
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "[FollowGift] Listen timeout");
      return "";
    }

    const auto response = future.get();
    if (!response->success) {
      RCLCPP_WARN(this->get_logger(), "[FollowGift] No response received");
      return "";
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[FollowGift] Heard response: '%s'",
      response->recognized_speech.c_str());
    return response->recognized_speech;
  }

  std::string waitForFollowGiftAnswer(bool allow_follow_or_continue)
  {
    for (int attempt = 1; attempt <= MAX_FOLLOW_GIFT_LISTEN_ATTEMPTS; ++attempt) {
      if (attempt > 1) {
        if (allow_follow_or_continue) {
          sayText("I could not understand. Please say follow, continue, or stop.");
        } else {
          sayText("I could not understand. Please say stop when you want me to stop following.");
        }
      }

      const std::string answer = listenFollowGiftAnswer();
      if (answerHasStop(answer)) {
        return "stop";
      }

      if (allow_follow_or_continue && answerHasFollowOrContinue(answer)) {
        return "follow";
      }

      RCLCPP_WARN(
        this->get_logger(),
        "[FollowGift] ignoring answer '%s' (attempt %d/%d)",
        answer.c_str(),
        attempt,
        MAX_FOLLOW_GIFT_LISTEN_ATTEMPTS);
    }

    return "";
  }

  bool dispatchFollowGiftInstruction(
    const std::string & instruction,
    FollowGiftState next_state)
  {
    follow_gift_state_ = next_state;
    RCLCPP_INFO(
      this->get_logger(),
      "[FollowGift] dispatching '%s'; next_state=%u",
      instruction.c_str(),
      static_cast<unsigned int>(follow_gift_state_));
    return dispatchOverrideInstruction(instruction);
  }

  bool handleFollowGiftSequence(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
  {
    // Punto de entrada: cuando termina "say instruct_follow_person", el plan deja
    // el control al manager para decidir dinamicamente segun lo que escuche.
    if (follow_gift_state_ == FollowGiftState::IDLE &&
        feedbackMatches(feedback, "say", "instruct_follow_person") &&
        isSuccessfulFeedback(feedback))
    {
      const std::string answer = waitForFollowGiftAnswer(true);
      if (answer == "stop") {
        sayText("Understood. I am ready to leave the gift.");
        return dispatchFollowGiftInstruction(
          "Justina drop gift",
          FollowGiftState::WAIT_DROP_GIFT);
      }

      if (answer.empty()) {
        sayText("I could not understand the instruction. I will start following you.");
      }

      if (answer == "follow") {
        sayText("Understood. I will start following you now.");
      }

      return dispatchFollowGiftInstruction(
        "Justina follow enable",
        FollowGiftState::WAIT_FOLLOW_ENABLE);
    }

    if (follow_gift_state_ == FollowGiftState::IDLE) {
      return false;
    }

    switch (follow_gift_state_)
    {
      case FollowGiftState::WAIT_FOLLOW_ENABLE:
      {
        // Cuando el seguimiento ya quedo activo, el manager escucha directo
        // hasta oir stop, sin inyectar listen al plan.
        if (!feedbackMatches(feedback, "follow", "enable")) {
          return false;
        }

        if (!isSuccessfulFeedback(feedback)) {
          follow_gift_state_ = FollowGiftState::IDLE;
          return false;
        }

        const std::string answer = waitForFollowGiftAnswer(false);
        if (answer.empty()) {
          sayText("I could not hear stop clearly. I will stop following now.");
        } else if (answer == "stop") {
          sayText("Understood. I am ready to leave the gift.");
        }

        return dispatchFollowGiftInstruction(
          "Justina follow disable",
          FollowGiftState::WAIT_FOLLOW_DISABLE);
      }

      case FollowGiftState::WAIT_FOLLOW_DISABLE:
      {
        // Al detener el seguimiento, se encadena la entrega del regalo.
        if (!feedbackMatches(feedback, "follow", "disable")) {
          return false;
        }

        if (!isSuccessfulFeedback(feedback)) {
          follow_gift_state_ = FollowGiftState::IDLE;
          return false;
        }

        return dispatchFollowGiftInstruction(
          "Justina drop gift",
          FollowGiftState::WAIT_DROP_GIFT);
      }

      case FollowGiftState::WAIT_DROP_GIFT:
      {
        // La secuencia dinamica termina despues de ejecutar drop gift; desde aqui
        // el manager vuelve al flujo normal del plan.
        if (!feedbackMatches(feedback, "drop", "gift")) {
          return false;
        }

        follow_gift_state_ = FollowGiftState::IDLE;
        return false;
      }

      case FollowGiftState::IDLE:
      default:
        return false;
    }
  }

  bool needsApprovalRetry(const std::string & action) const
  {
    // Acciones que pueden beneficiarse de reintento/correccion en vez de
    // simplemente continuar con el siguiente paso.
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

  bool isSayAskAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    // Primer paso del patron de confirmacion:
    // "Justina say ask_name", "Justina say ask_drink", etc.
    const std::string action = normalizeText(feedback.action);
    const std::string argument = normalizeText(feedback.argument);

    return action == "say" && argument.rfind("ask_", 0U) == 0U;
  }

  bool isListenAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    const std::string action = normalizeText(feedback.action);
    return action == "listen";
  }

  bool isFullListenArgument(const std::string & argument) const
  {
    // Listen en modo FULL guarda en world state y usa el formato:
    // <categoria>_<atributo>_<indice>, por ejemplo guest_name_1.
    const std::size_t last_sep = argument.rfind('_');
    if (last_sep == std::string::npos || last_sep == 0 ||
        last_sep + 1 >= argument.size())
    {
      return false;
    }

    const std::string prefix = argument.substr(0, last_sep);
    const std::size_t second_sep = prefix.rfind('_');
    return second_sep != std::string::npos &&
           second_sep > 0 &&
           second_sep + 1 < prefix.size();
  }

  std::string listenAttributeFromArgument(const std::string & argument) const
  {
    // De guest_name_1 extrae "name"; se usa para construir la frase
    // "I understood your name is ...".
    const std::size_t last_sep = argument.rfind('_');
    if (last_sep == std::string::npos || last_sep == 0) {
      return argument;
    }

    const std::string prefix = argument.substr(0, last_sep);
    const std::size_t second_sep = prefix.rfind('_');
    if (second_sep == std::string::npos || second_sep + 1 >= prefix.size()) {
      return argument;
    }

    return prefix.substr(second_sep + 1);
  }

  std::string extractStoredListenValue(const std::string & worker_result) const
  {
    // Listen devuelve el dato guardado como "people[1].name=sergio".
    // Aqui tomamos solo la parte despues del '='.
    const std::size_t eq = worker_result.find('=');
    if (eq == std::string::npos || eq + 1 >= worker_result.size()) {
      return "";
    }

    return worker_result.substr(eq + 1);
  }

  std::string confirmationPhrase(
    const std::string & listen_argument,
    const std::string & value) const
  {
    // Frase hablada para confirmar el dato antes de avanzar en el plan.
    const std::string attribute =
      makeSpeechFriendly(listenAttributeFromArgument(listen_argument));

    if (attribute == "name") {
      return "I understood your name is " + value + ". Is that correct?";
    }

    if (attribute == "drink") {
      return "I understood your drink is " + value + ". Is that correct?";
    }

    if (attribute == "order") {
      return "I understood your order is " + value + ". Is that correct?";
    }

    return "I understood " + attribute + " as " + value + ". Is that correct?";
  }

  std::string listenYesNo()
  {
    // Confirmacion fuera del plan: usa directamente HRI/listener para no
    // consumir otra instruccion del plan mientras se valida el dato.
    if (!listener_client_) {
      RCLCPP_ERROR(this->get_logger(), "[Confirmation] Listen client not initialized");
      return "";
    }

    if (!listener_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "[Confirmation] Listen service unavailable");
      return "";
    }

    auto request = std::make_shared<listener_interfaces::srv::Listen::Request>();
    request->enable = true;
    // test.json contiene yes/no; cambiar esta gramatica si se crea una dedicada.
    request->grammar = "test.json";

    auto future = listener_client_->async_send_request(request);
    const auto status = future.wait_for(8s);
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "[Confirmation] Listen yes/no timeout");
      return "";
    }

    const auto response = future.get();
    if (!response->success) {
      RCLCPP_WARN(this->get_logger(), "[Confirmation] No yes/no response received");
      return "";
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[Confirmation] Heard yes/no response: '%s'",
      response->recognized_speech.c_str());
    return response->recognized_speech;
  }

  bool isYesAnswer(const std::string & text) const
  {
    const std::string normalized = normalizeText(text);
    return normalized.find("yes") != std::string::npos ||
           normalized.find("yeah") != std::string::npos ||
           normalized.find("correct") != std::string::npos ||
           normalized.find("right") != std::string::npos ||
           normalized.find("continue") != std::string::npos;
  }

  bool isNoAnswer(const std::string & text) const
  {
    const std::string normalized = normalizeText(text);
    return normalized.find("no") != std::string::npos ||
           normalized.find("nope") != std::string::npos ||
           normalized.find("wrong") != std::string::npos ||
           normalized.find("incorrect") != std::string::npos;
  }

  void clearDataConfirmation()
  {
    pending_confirmation_ask_key_.clear();
    pending_confirmation_listen_argument_.clear();
    pending_confirmation_value_.clear();
    confirmation_attempts_ = 0;
  }

  void rememberAskPrompt(const std::string & ask_key)
  {
    if (pending_confirmation_ask_key_ != ask_key) {
      confirmation_attempts_ = 0;
    }

    pending_confirmation_ask_key_ = ask_key;
    pending_confirmation_listen_argument_.clear();
    pending_confirmation_value_.clear();

    RCLCPP_INFO(
      this->get_logger(),
      "[Confirmation] Tracking ask prompt '%s'",
      pending_confirmation_ask_key_.c_str());
  }

  bool retryDataCapture()
  {
    // Repite la pareja ask/listen usando las herramientas ya existentes:
    // 1) rewind_one hace que supervisor_plan repita el listen actual,
    // 2) corrective_instruction_pub_ inyecta antes el "Justina say ask_*".
    if (pending_confirmation_ask_key_.empty()) {
      return false;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "[Confirmation] Repeating data capture with '%s' (attempt %d/%d)",
      pending_confirmation_ask_key_.c_str(),
      confirmation_attempts_ + 1,
      MAX_CONFIRMATION_ATTEMPTS);

    rewind_pub_->publish(std_msgs::msg::Empty{});

    std_msgs::msg::String inj;
    inj.data = "Justina say " + pending_confirmation_ask_key_;
    corrective_instruction_pub_->publish(inj);

    publishAllowNextCycle();
    return true;
  }

  bool handleDataConfirmation(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
  {
    // Devuelve true cuando este helper ya disparo un reintento y el FSM debe
    // quedarse esperando el siguiente resultado sin avanzar el plan.
    // Detecta el primer paso del patron: "Justina say ask_*".
    if (isSayAskAction(feedback)) {
      rememberAskPrompt(feedback.argument);
      return false;
    }

    // Si habia una pregunta ask_* pendiente, la siguiente instruccion relevante
    // debe ser un listen FULL como guest_name_1. Otro tipo de accion cancela
    // la confirmacion para no mezclar datos entre pasos.
    if (!pending_confirmation_ask_key_.empty() && !isListenAction(feedback)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[Confirmation] Expected listen after '%s' but got action '%s'; clearing confirmation state",
        pending_confirmation_ask_key_.c_str(),
        feedback.action.c_str());
      clearDataConfirmation();
      return false;
    }

    if (pending_confirmation_ask_key_.empty() || !isListenAction(feedback)) {
      return false;
    }

    if (!isFullListenArgument(feedback.argument)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[Confirmation] Listen argument '%s' is not a stored-parameter listen; skipping confirmation",
        feedback.argument.c_str());
      clearDataConfirmation();
      return false;
    }

    const std::string value = extractStoredListenValue(feedback.worker_result);
    if (value.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "[Confirmation] Could not extract stored value from worker_result '%s'",
        feedback.worker_result.c_str());
      clearDataConfirmation();
      return false;
    }

    pending_confirmation_listen_argument_ = feedback.argument;
    pending_confirmation_value_ = value;
    ++confirmation_attempts_;

    const std::string phrase = confirmationPhrase(
      pending_confirmation_listen_argument_,
      pending_confirmation_value_);

    // Si el yes/no no se entiende, se repite solo la confirmacion. No volvemos
    // a pedir el dato completo a menos que el usuario responda explicitamente no.
    for (int yes_no_attempt = 1;
         yes_no_attempt <= MAX_CONFIRMATION_ATTEMPTS;
         ++yes_no_attempt)
    {
      if (yes_no_attempt > 1) {
        sayText("I could not understand if that was correct. " + phrase);
      } else {
        sayText(phrase);
      }

      const std::string answer = listenYesNo();
      if (isYesAnswer(answer)) {
        RCLCPP_INFO(
          this->get_logger(),
          "[Confirmation] User confirmed '%s'='%s'",
          pending_confirmation_listen_argument_.c_str(),
          pending_confirmation_value_.c_str());
        clearDataConfirmation();
        return false;
      }

      if (isNoAnswer(answer)) {
        if (confirmation_attempts_ < MAX_CONFIRMATION_ATTEMPTS) {
          return retryDataCapture();
        }

        break;
      }
    }

    bool active_plan = true;
    const bool knows_plan_state = getPlanActiveStatus(active_plan);
    sayText(
      (knows_plan_state && !active_plan)
        ? "I could not confirm the information. There are no more steps in the plan."
        : "I could not confirm the information. I will continue with the task.");
    RCLCPP_WARN(
      this->get_logger(),
      "[Confirmation] Max attempts reached for '%s'; continuing",
      pending_confirmation_listen_argument_.c_str());
    clearDataConfirmation();
    return false;
  }

  bool handleDataCaptureFailure(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
  {
    // Si falla el listen que venia despues de un ask_*, no avanzamos el plan:
    // repetimos la pregunta y el listen hasta MAX_CONFIRMATION_ATTEMPTS.
    if (pending_confirmation_ask_key_.empty() || !isListenAction(feedback)) {
      return false;
    }

    ++confirmation_attempts_;

    if (confirmation_attempts_ < MAX_CONFIRMATION_ATTEMPTS) {
      sayText("I could not hear the answer. I will ask again.");
      return retryDataCapture();
    }

    bool active_plan = true;
    const bool knows_plan_state = getPlanActiveStatus(active_plan);
    sayText(
      (knows_plan_state && !active_plan)
        ? "I could not get the information. There are no more steps in the plan."
        : "I could not get the information. I will continue with the task.");
    RCLCPP_WARN(
      this->get_logger(),
      "[Confirmation] Listen failed after %d attempts for '%s'; continuing",
      MAX_CONFIRMATION_ATTEMPTS,
      pending_confirmation_ask_key_.c_str());
    clearDataConfirmation();
    if (knows_plan_state && !active_plan) {
      RCLCPP_INFO(
        this->get_logger(),
        "[Confirmation] Listen failed on last instruction -> finish without dispatching another step");
      state_ = State::SM_FINISH;
      return true;
    }
    publishAllowNextCycle();
    return true;
  }

  bool isExecutionFailureStatus(uint8_t status) const
  {
    // approved=false y estos estados se tratan como fallo recuperable:
    // el robot anuncia el problema y continua con la siguiente instruccion.
    return status == supervisor_interfaces::msg::TaskExecutionReport::FAILURE ||
           status == supervisor_interfaces::msg::TaskExecutionReport::TIMEOUT ||
           status == supervisor_interfaces::msg::TaskExecutionReport::REJECTED;
  }

  std::string normalizeText(std::string value) const
  {
    std::transform(
      value.begin(),
      value.end(),
      value.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    return value;
  }

  bool isDoorClosedCheckFailure(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    const std::string action = normalizeText(feedback.action);
    const std::string reason = normalizeText(feedback.failure_reason);

    // Only a real closed-door reading starts the human prompt and retry loop.
    // check_service_failed and unsupported arguments fall through to the
    // generic failure path and continue.
    return (action == "check" || action == "wait_for") &&
           reason == supervisor_interfaces::msg::TaskExecutionReport::REASON_DOOR_CLOSED;
  }

  bool isDoorOpenCheckSuccess(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    const std::string action = normalizeText(feedback.action);
    const std::string worker_result = normalizeText(feedback.worker_result);

    return (action == "check" || action == "wait_for") &&
           feedback.status == supervisor_interfaces::msg::TaskExecutionReport::SUCCESS &&
           worker_result.rfind("door_open", 0U) == 0U;
  }

  bool isSuccessfulGoTo(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    const std::string action = normalizeText(feedback.action);
    return (action == "go_to" || action == "goto") &&
           feedback.status == supervisor_interfaces::msg::TaskExecutionReport::SUCCESS &&
           feedback.approved;
  }

  bool isNavigationFailure(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    const std::string action = normalizeText(feedback.action);
    const std::string reason = normalizeText(feedback.failure_reason);
    return (action == "go_to" || action == "goto") && reason == "navigation_failed";
  }

  bool isBumperFailure(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    const std::string worker = normalizeText(feedback.worker_result);
    const std::string reason = normalizeText(feedback.failure_reason);
    return worker.find("bumper") != std::string::npos ||
           reason.find("bumper") != std::string::npos;
  }

  bool isFindPosePersonNotFound(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    const std::string action = normalizeText(feedback.action);
    const std::string worker = normalizeText(feedback.worker_result);
    return action == "find_pose" &&
           worker.size() >= std::strlen("_pose_not_found") &&
           worker.rfind("_pose_not_found") == worker.size() - std::strlen("_pose_not_found");
  }

  bool isFindPersonAnybodyNotFound(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    const std::string action = normalizeText(feedback.action);
    const std::string argument = normalizeText(feedback.argument);
    const std::string worker = normalizeText(feedback.worker_result);
    return action == "find_person" &&
           argument.rfind("anybody", 0U) == 0U &&
           worker.rfind("person_not_found:", 0U) == 0U;
  }

  void resetFindPersonAnybodyRetryState()
  {
    find_person_anybody_retry_count_ = 0;
    find_person_anybody_retry_argument_.clear();
  }

  bool moveHeadForFindPersonRetryExhausted()
  {
    if (!head_client_) {
      RCLCPP_ERROR(this->get_logger(), "[FindPersonRetry] Head client not initialized");
      return false;
    }

    if (!head_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "[FindPersonRetry] Head service unavailable");
      return false;
    }

    auto request = std::make_shared<head_interfaces::srv::MoveHead::Request>();
    request->positions = {0.0, 0.2};

    auto future = head_client_->async_send_request(request);
    const auto status = future.wait_for(5s);
    if (status != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "[FindPersonRetry] Timeout moving head to [0.0, 0.2]");
      return false;
    }

    const auto response = future.get();
    if (!response->success) {
      RCLCPP_WARN(
        this->get_logger(),
        "[FindPersonRetry] Head move to [0.0, 0.2] failed: %s",
        response->message.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "[FindPersonRetry] Head moved to [0.0, 0.2]");
    return true;
  }

  bool lookupFramePositionInMap(
    const std::string & source_frame,
    double & x,
    double & y,
    std::string & error_message)
  {
    try {
      const auto tf = tf_buffer_.lookupTransform(
        map_frame_,
        source_frame,
        tf2::TimePointZero,
        500ms);

      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      error_message.clear();
      return true;
    } catch (const tf2::TransformException & ex) {
      error_message =
        "cannot lookup TF " + map_frame_ + " -> " + source_frame + ": " + ex.what();
      return false;
    }
  }

  GoToArrivalVerification verifySuccessfulGoToArrival(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
  {
    GoToArrivalVerification result;
    if (!isSuccessfulGoTo(feedback)) {
      return result;
    }

    result.applicable = true;
    result.goal_frame = normalizeKnowledgeToken(feedback.argument);
    if (result.goal_frame.empty()) {
      result.error_message = "go_to destination is empty after normalization";
      return result;
    }

    double robot_x = 0.0;
    double robot_y = 0.0;
    if (!lookupFramePositionInMap(base_frame_, robot_x, robot_y, result.error_message)) {
      return result;
    }

    double goal_x = 0.0;
    double goal_y = 0.0;
    if (!lookupFramePositionInMap(result.goal_frame, goal_x, goal_y, result.error_message)) {
      return result;
    }

    const double dx = goal_x - robot_x;
    const double dy = goal_y - robot_y;

    result.distance_m = std::sqrt(dx * dx + dy * dy);
    result.verified = true;
    result.within_tolerance = result.distance_m <= go_to_success_distance_tolerance_m_;
    return result;
  }

  void updateInstructionPointStateFromGoTo(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
  {
    if (!natural_language_planning_ || !isSuccessfulGoTo(feedback)) {
      return;
    }

    const std::string destination = normalizeText(feedback.argument);
    if (destination.empty()) {
      return;
    }

    at_instruction_point_ = (destination == "instruction_point");
    RCLCPP_INFO(
      this->get_logger(),
      "SM_WAIT_EXECUTION_RESULT: go_to succeeded to '%s'; at_instruction_point=%s",
      feedback.argument.c_str(),
      at_instruction_point_ ? "true" : "false");
  }

  void scheduleDoorCheckRetry()
  {
    // Give the human a short window to open the door before dispatching the
    // same check instruction again.
    door_retry_pending_ = true;
    door_retry_time_ = this->now() + rclcpp::Duration::from_seconds(5.0);
  }

  void handleDoorClosedCheck(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
  {
    // Caso especial: una puerta cerrada no cancela todo el plan.
    // Se pide ayuda, se espera un poco y se reintenta el mismo check.
    RCLCPP_WARN(
      this->get_logger(),
      "SM_WAIT_EXECUTION_RESULT: door is closed; requesting human assistance and retrying. worker='%s'",
      feedback.worker_result.c_str());

    if (door_open_request_count_ >= MAX_DOOR_OPEN_REQUESTS) {
      RCLCPP_WARN(
        this->get_logger(),
        "SM_WAIT_EXECUTION_RESULT: door stayed closed after %d requests; continuing with next instruction",
        MAX_DOOR_OPEN_REQUESTS);

      sayText(
        "The door is still closed, so I cannot continue through it. "
        "I will move on to the next instruction.");

      door_waiting_active_ = false;
      door_retry_pending_ = false;
      door_open_request_count_ = 0;
      publishAllowNextCycle();
      return;
    }

    const char * prompt = door_waiting_active_
      ? "I am waiting for the door to be opened."
      : "Please open the door so I can continue.";

    sayText(prompt);

    ++door_open_request_count_;

    // Stay in the execution-result state; the timer will retry this check when
    // door_retry_time_ is reached.
    door_waiting_active_ = true;
    scheduleDoorCheckRetry();
  }

  std::string executionStatusName(uint8_t status) const
  {
    switch (status) {
      case supervisor_interfaces::msg::TaskExecutionReport::SUCCESS:
        return "SUCCESS";
      case supervisor_interfaces::msg::TaskExecutionReport::FAILURE:
        return "FAILURE";
      case supervisor_interfaces::msg::TaskExecutionReport::TIMEOUT:
        return "TIMEOUT";
      case supervisor_interfaces::msg::TaskExecutionReport::REJECTED:
        return "REJECTED";
      default:
        return "UNKNOWN";
    }
  }

  std::string makeSpeechFriendly(std::string text) const
  {
    std::replace(text.begin(), text.end(), '_', ' ');
    return text;
  }

  std::string failureReasonForSpeech(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback,
    const std::string & fallback_reason) const
  {
    // Para el habla usamos primero el mensaje del worker: normalmente contiene
    // el detalle util para el usuario, mientras failure_reason es una etiqueta interna.
    if (!feedback.worker_result.empty()) {
      return makeSpeechFriendly(feedback.worker_result);
    }

    if (!feedback.failure_reason.empty() &&
        feedback.failure_reason != supervisor_interfaces::msg::TaskExecutionReport::REASON_NONE)
    {
      return makeSpeechFriendly(feedback.failure_reason);
    }

    return makeSpeechFriendly(fallback_reason);
  }

  bool isTakeAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    return normalizeText(feedback.action) == "take";
  }

  bool isApproachAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    return normalizeText(feedback.action) == "approach";
  }

  bool isApproachPartialAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    return normalizeText(feedback.action) == "approach_partial";
  }

  bool isFindPoseAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    return normalizeText(feedback.action) == "find_pose";
  }

  bool customerIndexFromApproachArgument(
    const std::string & argument,
    std::string & index) const
  {
    const std::string normalized = normalizeKnowledgeToken(argument);
    constexpr const char * prefix = "customer_";
    if (normalized.rfind(prefix, 0U) != 0U ||
        normalized.size() <= std::strlen(prefix))
    {
      return false;
    }

    index = normalized.substr(std::strlen(prefix));
    return std::all_of(
      index.begin(),
      index.end(),
      [](unsigned char c) {return std::isdigit(c) != 0;});
  }

  bool isCustomerApproachAction(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback,
    std::string & customer_index) const
  {
    return isApproachAction(feedback) &&
           customerIndexFromApproachArgument(feedback.argument, customer_index);
  }

  std::string findPoseArgumentForCustomerIndex(
    const std::string & customer_index) const
  {
    return "waving_" + customer_index;
  }

  bool customerIndexFromFindPoseArgument(
    const std::string & argument,
    std::string & index) const
  {
    const std::string normalized = normalizeKnowledgeToken(argument);
    constexpr const char * prefix = "waving_";
    if (normalized.rfind(prefix, 0U) != 0U ||
        normalized.size() <= std::strlen(prefix))
    {
      return false;
    }

    index = normalized.substr(std::strlen(prefix));
    return std::all_of(
      index.begin(),
      index.end(),
      [](unsigned char c) {return std::isdigit(c) != 0;});
  }

  std::string customerApproachArgumentForIndex(
    const std::string & customer_index) const
  {
    return "customer_" + customer_index;
  }

  std::string findPoseDescription(const std::string & argument) const
  {
    std::string normalized = normalizeKnowledgeToken(argument);
    const std::size_t sep = normalized.rfind('_');
    if (sep != std::string::npos && sep + 1 < normalized.size()) {
      const std::string suffix = normalized.substr(sep + 1);
      const bool numeric_suffix = std::all_of(
        suffix.begin(),
        suffix.end(),
        [](unsigned char c) {return std::isdigit(c) != 0;});
      if (numeric_suffix) {
        normalized = normalized.substr(0, sep);
      }
    }

    std::replace(normalized.begin(), normalized.end(), '_', ' ');
    return normalized.empty() ? "with the requested pose" : normalized;
  }

  bool promptFindPoseTarget(const std::string & find_pose_argument)
  {
    const std::string pose = findPoseDescription(find_pose_argument);
    return sayText(
      "I need to confirm your position. I am looking for the person " +
      pose + ". Please raise your hand and keep it raised.");
  }

  bool isTakePoseOutOfRangeFailure(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback) const
  {
    return isTakeAction(feedback) &&
           normalizeText(feedback.worker_result) == "pose_out_of_range";
  }

  bool isJustinaTakeInstruction(const std::string & instruction) const
  {
    const std::string normalized = normalizeText(instruction);
    return normalized.rfind("justina take ", 0U) == 0U;
  }

  std::string takeRetryInstructionKey(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
  {
    const std::string current_instruction = getCurrentInstruction();
    if (isJustinaTakeInstruction(current_instruction)) {
      return current_instruction;
    }

    if (isTakeAction(feedback) && !feedback.argument.empty()) {
      return "Justina take " + feedback.argument;
    }

    return "";
  }

  bool handleTakeFailureRetry(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
  {
    if (!isTakeAction(feedback)) {
      return false;
    }

    const std::string instruction_key = takeRetryInstructionKey(feedback);
    if (instruction_key.empty()) {
      return false;
    }

    const std::string normalized_key = normalizeText(instruction_key);
    if (take_retry_pending_ && normalizeText(take_retry_instruction_) == normalized_key) {
      RCLCPP_WARN(
        this->get_logger(),
        "[TakeRetry] Retry also failed for '%s'; continuing with normal failure path",
        instruction_key.c_str());
      take_retry_pending_ = false;
      take_retry_instruction_.clear();
      return false;
    }

    take_retry_pending_ = true;
    take_retry_instruction_ = instruction_key;

    if (isTakePoseOutOfRangeFailure(feedback)) {
      take_approach_retry_state_ = TakeApproachRetryState::WAIT_APPROACH;
      take_approach_retry_argument_ = feedback.argument;

      const std::string approach_instruction =
        "Justina approach " + feedback.argument;

      RCLCPP_WARN(
        this->get_logger(),
        "[TakeRetry] Take target is out of range; injecting approach before retry: '%s'",
        approach_instruction.c_str());

      if (!sayText("I need to get closer first.")) {
        RCLCPP_WARN(
          this->get_logger(),
          "[TakeRetry] could not announce approach recovery through TTS");
      }

      return dispatchOverrideInstruction(approach_instruction);
    }

    RCLCPP_WARN(
      this->get_logger(),
      "[TakeRetry] Retrying failed take instruction once: '%s'",
      take_retry_instruction_.c_str());

    if (!sayText("I will try once more.")) {
      RCLCPP_WARN(
        this->get_logger(),
        "[TakeRetry] could not announce take retry through TTS");
    }

    publishRetryCommand();
    publishAllowNextCycle();
    return true;
  }

  bool handleTakeApproachRecovery(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
  {
    if (take_approach_retry_state_ == TakeApproachRetryState::IDLE) {
      return false;
    }

    const std::string retry_argument = normalizeText(take_approach_retry_argument_);

    if (take_approach_retry_state_ == TakeApproachRetryState::WAIT_APPROACH) {
      if (!isApproachAction(feedback) ||
          normalizeText(feedback.argument) != retry_argument)
      {
        return false;
      }

      if (!isSuccessfulFeedback(feedback)) {
        RCLCPP_WARN(
          this->get_logger(),
          "[TakeRetry] Approach recovery failed for '%s'; continuing with normal failure path",
          take_approach_retry_argument_.c_str());
        take_approach_retry_state_ = TakeApproachRetryState::IDLE;
        take_approach_retry_argument_.clear();
        take_retry_pending_ = false;
        take_retry_instruction_.clear();
        return false;
      }

      take_approach_retry_state_ = TakeApproachRetryState::WAIT_TAKE_RETRY;
      RCLCPP_INFO(
        this->get_logger(),
        "[TakeRetry] Approach completed; retrying original take: '%s'",
        take_retry_instruction_.c_str());
      return dispatchOverrideInstruction(take_retry_instruction_);
    }

    if (take_approach_retry_state_ == TakeApproachRetryState::WAIT_TAKE_RETRY) {
      if (!isTakeAction(feedback) ||
          normalizeText(feedback.argument) != retry_argument)
      {
        return false;
      }

      take_approach_retry_state_ = TakeApproachRetryState::IDLE;
      take_approach_retry_argument_.clear();
      return false;
    }

    return false;
  }

  void resetPersonApproachRecovery()
  {
    person_approach_recovery_state_ = PersonApproachRecoveryState::IDLE;
    person_approach_argument_.clear();
    person_find_pose_argument_.clear();
    person_find_pose_retry_count_ = 0;
  }

  bool handlePersonApproachRecovery(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback)
  {
    if (person_approach_recovery_state_ ==
        PersonApproachRecoveryState::WAIT_PARTIAL_APPROACH)
    {
      if (!isApproachPartialAction(feedback) ||
          normalizeKnowledgeToken(feedback.argument) !=
            normalizeKnowledgeToken(person_approach_argument_))
      {
        return false;
      }

      if (!isSuccessfulFeedback(feedback)) {
        RCLCPP_WARN(
          this->get_logger(),
          "[PersonApproachRecovery] preventive partial approach failed for '%s'; continuing with normal failure path",
          person_approach_argument_.c_str());
        resetPersonApproachRecovery();
        return false;
      }

      person_approach_recovery_state_ =
        PersonApproachRecoveryState::WAIT_PREVENTIVE_FIND_POSE;
      person_find_pose_retry_count_ = 0;

      RCLCPP_INFO(
        this->get_logger(),
        "[PersonApproachRecovery] partial approach completed; refreshing pose with find_pose '%s'",
        person_find_pose_argument_.c_str());

      if (!promptFindPoseTarget(person_find_pose_argument_)) {
        RCLCPP_WARN(
          this->get_logger(),
          "[PersonApproachRecovery] could not announce preventive find_pose prompt");
      }

      const std::string find_pose_instruction =
        "Justina find_pose " + person_find_pose_argument_;
      return dispatchOverrideInstruction(find_pose_instruction);
    }

    if (person_approach_recovery_state_ ==
        PersonApproachRecoveryState::WAIT_PREVENTIVE_FIND_POSE)
    {
      if (!isFindPoseAction(feedback) ||
          normalizeKnowledgeToken(feedback.argument) !=
            normalizeKnowledgeToken(person_find_pose_argument_))
      {
        return false;
      }

      if (!isSuccessfulFeedback(feedback)) {
        ++person_find_pose_retry_count_;
        RCLCPP_WARN(
          this->get_logger(),
          "[PersonApproachRecovery] preventive find_pose '%s' failed (retry #%d).",
          person_find_pose_argument_.c_str(),
          person_find_pose_retry_count_);

        if (person_find_pose_retry_count_ == 1 ||
            person_find_pose_retry_count_ %
              FIND_POSE_REPROMPT_EVERY_RETRIES == 0)
        {
          if (!promptFindPoseTarget(person_find_pose_argument_)) {
            RCLCPP_WARN(
              this->get_logger(),
              "[PersonApproachRecovery] could not repeat preventive find_pose prompt");
          }
        }

        publishRetryCommand();
        publishAllowNextCycle();
        return true;
      }

      RCLCPP_INFO(
        this->get_logger(),
        "[PersonApproachRecovery] preventive pose refresh completed; continuing normal plan");
      resetPersonApproachRecovery();
      find_pose_retry_count_ = 0;
      publishAllowNextCycle();
      return true;
    }

    if (person_approach_recovery_state_ ==
        PersonApproachRecoveryState::WAIT_FIND_POSE)
    {
      if (!isFindPoseAction(feedback) ||
          normalizeKnowledgeToken(feedback.argument) !=
            normalizeKnowledgeToken(person_find_pose_argument_))
      {
        return false;
      }

      if (!isSuccessfulFeedback(feedback)) {
        ++person_find_pose_retry_count_;
        RCLCPP_WARN(
          this->get_logger(),
          "[PersonApproachRecovery] find_pose '%s' did not confirm the person (retry #%d).",
          person_find_pose_argument_.c_str(),
          person_find_pose_retry_count_);

        if (person_find_pose_retry_count_ == 1 ||
            person_find_pose_retry_count_ %
              FIND_POSE_REPROMPT_EVERY_RETRIES == 0)
        {
          if (!promptFindPoseTarget(person_find_pose_argument_)) {
            RCLCPP_WARN(
              this->get_logger(),
              "[PersonApproachRecovery] could not repeat find_pose prompt");
          }
        }

        publishRetryCommand();
        publishAllowNextCycle();
        return true;
      }

      person_approach_recovery_state_ =
        PersonApproachRecoveryState::WAIT_APPROACH;
      person_find_pose_retry_count_ = 0;
      find_pose_retry_count_ = 0;

      const std::string approach_instruction =
        "Justina approach " + person_approach_argument_;

      RCLCPP_INFO(
        this->get_logger(),
        "[PersonApproachRecovery] find_pose confirmed target; refining approach with '%s'",
        approach_instruction.c_str());

      return dispatchOverrideInstruction(approach_instruction);
    }

    if (person_approach_recovery_state_ ==
        PersonApproachRecoveryState::WAIT_APPROACH)
    {
      if (!isApproachAction(feedback) ||
          normalizeKnowledgeToken(feedback.argument) !=
            normalizeKnowledgeToken(person_approach_argument_))
      {
        return false;
      }

      if (!isSuccessfulFeedback(feedback)) {
        RCLCPP_WARN(
          this->get_logger(),
          "[PersonApproachRecovery] refined approach failed for '%s'; continuing with normal failure path",
          person_approach_argument_.c_str());
        resetPersonApproachRecovery();
        return false;
      }

      RCLCPP_INFO(
        this->get_logger(),
        "[PersonApproachRecovery] refined approach completed for '%s'",
        person_approach_argument_.c_str());
      resetPersonApproachRecovery();
      nav_retry_count_ = 0;
      publishAllowNextCycle();
      return true;
    }

    if (!isSuccessfulFeedback(feedback)) {
      return false;
    }

    if (isFindPoseAction(feedback)) {
      std::string customer_index;
      if (!customerIndexFromFindPoseArgument(feedback.argument, customer_index)) {
        return false;
      }

      person_approach_argument_ =
        customerApproachArgumentForIndex(customer_index);
      person_find_pose_argument_ =
        findPoseArgumentForCustomerIndex(customer_index);
      person_find_pose_retry_count_ = 0;
      person_approach_recovery_state_ =
        PersonApproachRecoveryState::WAIT_PARTIAL_APPROACH;

      const std::string partial_instruction =
        "Justina approach_partial " + person_approach_argument_;

      RCLCPP_INFO(
        this->get_logger(),
        "[PersonApproachRecovery] find_pose '%s' succeeded; inserting preventive partial approach '%s'",
        feedback.argument.c_str(),
        partial_instruction.c_str());

      return dispatchOverrideInstruction(partial_instruction);
    }

    std::string customer_index;
    if (!isCustomerApproachAction(feedback, customer_index)) {
      return false;
    }

    person_approach_argument_ = normalizeKnowledgeToken(feedback.argument);
    person_find_pose_argument_ =
      findPoseArgumentForCustomerIndex(customer_index);
    person_find_pose_retry_count_ = 0;
    nav_retry_count_ = 0;
    person_approach_recovery_state_ =
      PersonApproachRecoveryState::WAIT_FIND_POSE;

    RCLCPP_INFO(
      this->get_logger(),
      "[PersonApproachRecovery] customer approach reached; requesting pose confirmation with find_pose '%s'",
      person_find_pose_argument_.c_str());

    if (!promptFindPoseTarget(person_find_pose_argument_)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[PersonApproachRecovery] could not announce pose confirmation prompt");
    }

    const std::string find_pose_instruction =
      "Justina find_pose " + person_find_pose_argument_;
    return dispatchOverrideInstruction(find_pose_instruction);
  }

  void announceFailureAndContinue(
    const supervisor_interfaces::msg::TaskExecutionReport & feedback,
    const std::string & reason)
  {
    bool active_plan = true;
    const bool knows_plan_state = getPlanActiveStatus(active_plan);

    RCLCPP_ERROR(
      this->get_logger(),
      "SM_WAIT_EXECUTION_RESULT: action failed. action='%s' arg='%s' status=%s approved=%s reason='%s' worker='%s' plan_active_known=%s plan_active=%s",
      feedback.action.c_str(),
      feedback.argument.c_str(),
      executionStatusName(feedback.status).c_str(),
      feedback.approved ? "true" : "false",
      reason.c_str(),
      feedback.worker_result.c_str(),
      knows_plan_state ? "true" : "false",
      active_plan ? "true" : "false");

    std::string spoken = makeSpeechFriendly(feedback.action);
    if (!feedback.argument.empty()) {
      spoken += " " + makeSpeechFriendly(feedback.argument);
    }
    spoken += " failed: " + failureReasonForSpeech(feedback, reason) + ".";

    // Si supervisor_plan ya reporto que no quedan instrucciones, no prometemos
    // un siguiente paso ni disparamos otro ciclo de ejecucion.
    if (knows_plan_state && !active_plan) {
      spoken += " No more steps.";
    } else {
      spoken += " Next step.";
    }

    if (!sayText(spoken)) {
      RCLCPP_WARN(
        this->get_logger(),
        "SM_WAIT_EXECUTION_RESULT: could not announce failure through TTS");
    }

    if (knows_plan_state && !active_plan) {
      RCLCPP_INFO(
        this->get_logger(),
        "SM_WAIT_EXECUTION_RESULT: failed on last instruction -> finish without dispatching another step");
      state_ = State::SM_FINISH;
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "SM_WAIT_EXECUTION_RESULT: Manager semaphore -> skip failed step and allow next plan dispatch");
    publishAllowNextCycle();
  }

  std::string getCorrectiveInstruction(
    const std::string & action,
    const std::string & argument) const
  {
    // Instrucciones correctivas para fallos de escucha. El objetivo es decir
    // algo antes de repetir la instruccion original.
    if (action == "listen") {
      const std::string attribute = listenAttributeFromArgument(argument);
      if (attribute == "name")     return "Justina say repeat_name";
      if (attribute == "drink")    return "Justina say repeat_drink";
      if (attribute == "order")    return "Justina say repeat_order";
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
        state_ = State::SM_POST_PLAN_TRIGGER;
        break;
      }

      case State::SM_POST_PLAN_TRIGGER:
      {
        RCLCPP_INFO(this->get_logger(),
                    "SM_POST_PLAN_TRIGGER: plan ready.");
        RCLCPP_INFO(this->get_logger(),
                    "SM_POST_PLAN_TRIGGER: Manager semaphore -> allow one plan dispatch + allow execution trigger");
        // Si todavia no hay subscribers para el semaforo, no avanzamos a esperar
        // feedback de ejecucion porque nadie habria recibido la orden inicial.
        if (!publishAllowNextCycle()) {
          break;
        }
        state_ = State::SM_WAIT_EXECUTION_RESULT;
        break;
      }

      case State::SM_WAIT_EXECUTION_RESULT:
      {
        // Estado principal del ciclo: aqui se consume el resultado de la accion
        // anterior y se decide si se pide otra instruccion, se reintenta o termina.
        // Door retries are scheduled locally by the manager. No new planner step
        // is requested until the same check succeeds.
        if (door_retry_pending_)
        {
          if (this->now() < door_retry_time_)
          {
            RCLCPP_DEBUG_THROTTLE(
              this->get_logger(), *this->get_clock(), 1000,
              "SM_WAIT_EXECUTION_RESULT: waiting before retrying door check..."
            );
            break;
          }

          door_retry_pending_ = false;
          // Ask supervisor_plan to resend the last instruction, then allow one
          // execution cycle through the existing semaphore path.
          publishRetryCommand();
          publishAllowNextCycle();
          break;
        }

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

        if (isDoorClosedCheckFailure(feedback))
        {
          // Closed door is recoverable by human intervention, so do not announce
          // it as a generic failure and do not advance to the next plan step yet.
          handleDoorClosedCheck(feedback);
          break;
        }

        if (handleFollowGiftSequence(feedback))
        {
          break;
        }

        if (handleTakeApproachRecovery(feedback))
        {
          break;
        }

        if (handlePersonApproachRecovery(feedback))
        {
          break;
        }

        if (!feedback.approved || isExecutionFailureStatus(feedback.status))
        {
          if (handleDataCaptureFailure(feedback)) {
            break;
          }

          if (isFindPersonAnybodyNotFound(feedback))
          {
            const std::string normalized_argument = normalizeText(feedback.argument);
            if (find_person_anybody_retry_argument_ != normalized_argument) {
              find_person_anybody_retry_argument_ = normalized_argument;
              find_person_anybody_retry_count_ = 0;
            }

            if (find_person_anybody_retry_count_ < MAX_FIND_PERSON_ANYBODY_RETRIES) {
              ++find_person_anybody_retry_count_;
              RCLCPP_WARN(
                this->get_logger(),
                "SM_WAIT_EXECUTION_RESULT: find_person '%s' did not find anybody (retry #%d/%d).",
                feedback.argument.c_str(),
                find_person_anybody_retry_count_,
                MAX_FIND_PERSON_ANYBODY_RETRIES);
              publishRetryCommand();
              publishAllowNextCycle();
              break;
            }

            RCLCPP_WARN(
              this->get_logger(),
              "SM_WAIT_EXECUTION_RESULT: find_person '%s' exhausted %d retries. Moving head to [0.0, 0.2] before continuing.",
              feedback.argument.c_str(),
              MAX_FIND_PERSON_ANYBODY_RETRIES);
            moveHeadForFindPersonRetryExhausted();
            resetFindPersonAnybodyRetryState();
          }

          if (isFindPosePersonNotFound(feedback))
          {
            ++find_pose_retry_count_;
            RCLCPP_WARN(
              this->get_logger(),
              "SM_WAIT_EXECUTION_RESULT: find_pose '%s' did not find a person (retry #%d). Retrying until success.",
              feedback.argument.c_str(),
              find_pose_retry_count_);

            if (find_pose_retry_count_ == 1 ||
                find_pose_retry_count_ % FIND_POSE_REPROMPT_EVERY_RETRIES == 0)
            {
              if (!promptFindPoseTarget(feedback.argument)) {
                RCLCPP_WARN(
                  this->get_logger(),
                  "SM_WAIT_EXECUTION_RESULT: could not repeat find_pose prompt");
              }
            }

            publishRetryCommand();
            publishAllowNextCycle();
            break;
          }

          // Navigation failures retry indefinitely until goal is reached.
          // The only accepted stop is a bumper trigger.
          if (isNavigationFailure(feedback) && !isBumperFailure(feedback))
          {
            ++nav_retry_count_;
            RCLCPP_WARN(
              this->get_logger(),
              "SM_WAIT_EXECUTION_RESULT: navigation failed (retry #%d) for go_to '%s'. reason='%s' worker='%s' - retrying.",
              nav_retry_count_,
              feedback.argument.c_str(),
              feedback.failure_reason.c_str(),
              feedback.worker_result.c_str());
            publishRetryCommand();
            publishAllowNextCycle();
            break;
          }

          if (handleTakeFailureRetry(feedback)) {
            break;
          }

          const std::string status_name = executionStatusName(feedback.status);
          std::string reason;
          if (!feedback.failure_reason.empty() &&
              feedback.failure_reason != supervisor_interfaces::msg::TaskExecutionReport::REASON_NONE)
          {
            reason = status_name + " - " + feedback.failure_reason;
          }
          else if (!feedback.worker_result.empty())
          {
            reason = status_name + " - " + feedback.worker_result;
          }
          else
          {
            reason = status_name;
          }

          announceFailureAndContinue(feedback, reason);
          break;
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
          if (normalizeText(feedback.action) == "find_person") {
            resetFindPersonAnybodyRetryState();
          }

          if (normalizeText(feedback.action) == "find_pose") {
            find_pose_retry_count_ = 0;
          }

          const GoToArrivalVerification go_to_arrival = verifySuccessfulGoToArrival(feedback);
          if (go_to_arrival.applicable && go_to_arrival.verified &&
              !go_to_arrival.within_tolerance)
          {
            ++nav_retry_count_;
            RCLCPP_WARN(
              this->get_logger(),
              "SM_WAIT_EXECUTION_RESULT: go_to '%s' reported success but robot is still %.2f m away from TF '%s' (tolerance %.2f m). Retrying navigation (#%d).",
              feedback.argument.c_str(),
              go_to_arrival.distance_m,
              go_to_arrival.goal_frame.c_str(),
              go_to_success_distance_tolerance_m_,
              nav_retry_count_);
            publishRetryCommand();
            publishAllowNextCycle();
            break;
          }

          if (go_to_arrival.applicable && !go_to_arrival.verified) {
            RCLCPP_WARN(
              this->get_logger(),
              "SM_WAIT_EXECUTION_RESULT: go_to '%s' succeeded but arrival could not be verified against TF: %s. Accepting success.",
              feedback.argument.c_str(),
              go_to_arrival.error_message.c_str());
          }

          nav_retry_count_ = 0;
          if (isTakeAction(feedback)) {
            take_retry_pending_ = false;
            take_retry_instruction_.clear();
            take_approach_retry_state_ = TakeApproachRetryState::IDLE;
            take_approach_retry_argument_.clear();
          }

          updateInstructionPointStateFromGoTo(feedback);

          if (door_waiting_active_ && isDoorOpenCheckSuccess(feedback)) {
            sayText("I detected that the door is open. Thank you.");
          }

          // Confirma datos escuchados cuando el plan hizo say ask_* seguido de
          // listen <parametro>. Si el usuario dice que no, este helper inyecta
          // otra vez el say ask_* y rebobina el listen actual.
          if (handleDataConfirmation(feedback)) {
            break;
          }

          // Any successful execution clears a pending door-wait episode.
          door_waiting_active_ = false;
          door_retry_pending_ = false;
          door_open_request_count_ = 0;

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

  // ===== ROS =====
  rclcpp::Subscription<supervisor_interfaces::msg::SystemStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<supervisor_interfaces::msg::TaskExecutionReport>::SharedPtr
    execution_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr plan_active_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr plan_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_instruction_sub_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr rewind_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr allow_next_instruction_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr allow_plan_instruction_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr instruction_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr corrective_instruction_pub_;
  rclcpp::CallbackGroup::SharedPtr hri_client_group_;
  rclcpp::Client<known_locations_interfaces::srv::LocationsGet>::SharedPtr goto_client_;
  rclcpp::Client<head_interfaces::srv::MoveHead>::SharedPtr head_client_;
  rclcpp::Client<talker_interfaces::srv::Talk>::SharedPtr talker_client_;
  rclcpp::Client<listener_interfaces::srv::Listen>::SharedPtr listener_client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr execution_trigger_timer_;

  // Tools
  // Fachada hacia planning_tools: startPlanning, semantic parser y primitive_to_clips.
  std::shared_ptr<Planning> planning_;

  // false: usa un plan CLIPS ya definido. true: pide una orden hablada y genera
  // hechos CLIPS antes de planear.
  bool natural_language_planning_{false};
  std::string map_frame_{"map"};
  std::string base_frame_{"base_link"};
  double go_to_success_distance_tolerance_m_{0.75};
  static constexpr int MAX_NATURAL_LANGUAGE_LISTEN_RETRIES = 3;
  bool at_instruction_point_{false};
  bool known_locations_loaded_{false};
  std::unordered_set<std::string> known_locations_;

  // FSM
  State state_{State::SM_INIT};

  // Shared state
  // Estado actualizado por callbacks y consumido por tick().
  std::mutex mtx_;
  std::optional<supervisor_interfaces::msg::SystemStatus> last_status_;
  bool got_status_{false};

  // Evita encadenar multiples instrucciones correctivas para el mismo fallo.
  bool corrective_pending_ = false;

  // Estado para confirmar datos guardados por patrones say ask_* + listen X.
  // La cantidad maxima de intentos se define aqui.
  static constexpr int MAX_CONFIRMATION_ATTEMPTS = 3;
  std::string pending_confirmation_ask_key_;
  std::string pending_confirmation_listen_argument_;
  std::string pending_confirmation_value_;
  int confirmation_attempts_{0};

  // Estado del episodio "puerta cerrada".
  static constexpr int MAX_DOOR_OPEN_REQUESTS = 3;
  bool door_waiting_active_{false};
  bool door_retry_pending_{false};
  int door_open_request_count_{0};
  rclcpp::Time door_retry_time_{0, 0, RCL_ROS_TIME};

  // Conteo de reintentos de navegacion (go_to). Se resetea al exito.
  // Solo un bumper activo acepta detener la navegacion permanentemente.
  int nav_retry_count_{0};
  int find_pose_retry_count_{0};
  static constexpr int MAX_FIND_PERSON_ANYBODY_RETRIES = 3;
  int find_person_anybody_retry_count_{0};
  std::string find_person_anybody_retry_argument_;
  // Reintento unico para "Justina take <objeto>" antes de saltar al siguiente paso.
  bool take_retry_pending_{false};
  std::string take_retry_instruction_;
  // Recuperacion especifica: si take reporta pose_out_of_range, primero acerca
  // la base con approach y luego reintenta el mismo take una sola vez.
  TakeApproachRetryState take_approach_retry_state_{TakeApproachRetryState::IDLE};
  std::string take_approach_retry_argument_;
  // Recuperacion para clientes/personas: despues de approach customer_N,
  // pide confirmar waving_N, actualiza la pose con find_pose y acerca otra vez.
  static constexpr int FIND_POSE_REPROMPT_EVERY_RETRIES = 3;
  PersonApproachRecoveryState person_approach_recovery_state_{
    PersonApproachRecoveryState::IDLE};
  std::string person_approach_argument_;
  std::string person_find_pose_argument_;
  int person_find_pose_retry_count_{0};

  // Secuencia dinamica para recibir el regalo mientras sigue a una persona:
  // say instruct_follow_person -> escucha directa del manager -> follow/drop segun respuesta.
  static constexpr int MAX_FOLLOW_GIFT_LISTEN_ATTEMPTS = 3;
  FollowGiftState follow_gift_state_{FollowGiftState::IDLE};

  std::optional<supervisor_interfaces::msg::TaskExecutionReport> last_execution_feedback_;
  bool got_execution_feedback_{false};

  // Estado reportado por supervisor_plan.
  bool plan_active_{false};
  bool got_plan_active_{false};
  bool plan_ready_signal_{false};

  // Instruccion actual solo para logs/diagnostico.
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

bool getNaturalLanguagePlanningFromArgv(int argc, char ** argv)
{
  // Se parsea manualmente porque este nodo puede correrse con ros2 run o desde
  // launch; soporta natural_language_planning=true y natural_language_planning:=true.
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
