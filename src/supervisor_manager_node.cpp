#include <rclcpp/rclcpp.hpp>

#include <supervisor_interfaces/msg/system_status.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/int32.hpp>

#include <mutex>
#include <optional>
#include <string>

#include <planning_tools/Planning.hpp>
#include "supervisor_executor_interfaces/srv/authorize_task.hpp"

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
    input_trigger_pub_      = this->create_publisher<std_msgs::msg::Empty>("/input_trigger", 10);
    start_planning_client_  = this->create_client<std_srvs::srv::Trigger>("/planning/clips/start_planning");
    instruction_cmd_pub_    = this->create_publisher<std_msgs::msg::Int32>("/supervisor/supervisor_manager/instruction_command", 10);


    // --- Service server: authorize_task ---
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
      std::chrono::milliseconds(100),
      std::bind(&SupervisorManagerNode::tick, this)
    );

    // --- One-shot init for Planning tool (safe shared_from_this) ---
    init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(0),
      std::bind(&SupervisorManagerNode::initPlanningOnce, this)
    );

  }

private:
  enum class State : uint8_t
{
  SM_INIT,
  SM_VERIFY_SYS,
  SM_CREATE_PLAN,
  SM_WAIT_PLAN_RESPONSE,   // NEW
  SM_POST_PLAN_TRIGGER,
  SM_WAIT_AUTHORIZE,
  SM_ON_AUTHORIZE
};


  // ===== One-shot init =====
  void initPlanningOnce()
  {
    init_timer_->cancel();  // one-shot
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

  void authorizeTaskCallback( const supervisor_executor_interfaces::srv::AuthorizeTask::Request::SharedPtr req,
                              supervisor_executor_interfaces::srv::AuthorizeTask::Response::SharedPtr res)
  {
    // Default: approved true para no bloquear por ahora (si quieres, lo endurecemos luego)
    res->approved = true;
    res->message  = "ok";

    // Lógica pedida
    if (req->action == "Check")
    {
      if (req->worker_result == "success")
      {
        {
          std::lock_guard<std::mutex> lock(mtx_);
          authorize_ok_ = true;
        }
        RCLCPP_INFO(this->get_logger(), "authorize: Check success -> advance");
        return;
      }

      if (req->worker_result == "failure")
      {
        // publica "retry" = 1
        std_msgs::msg::Int32 msg;
        msg.data = 1;
        instruction_cmd_pub_->publish(msg);

        {
          std::lock_guard<std::mutex> lock(mtx_);
          authorize_retry_ = true;
        }

        RCLCPP_WARN(this->get_logger(), "authorize: Check failure -> publish retry=1 and re-trigger");
        return;
      }

    // Otros valores no esperados
    RCLCPP_WARN(this->get_logger(), "authorize: Check with unknown worker_result='%s'",
                req->worker_result.c_str());
    return;
  }

  // Si no es Check, puedes decidir comportamiento:
  // por ahora: no hace nada especial, solo aprueba.
  RCLCPP_DEBUG(this->get_logger(), "authorize: action='%s' (no special logic)", req->action.c_str());
}


  // ===== Helpers =====
  bool getLatestStatus(supervisor_interfaces::msg::SystemStatus & out)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!got_status_ || !last_status_.has_value()) return false;
    out = *last_status_;
    return true;
  }

  bool consumeAuthorizeCalled()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!authorize_called_) return false;
    authorize_called_ = false;
    return true;
  }

  void publishInputTriggerOnce()
  {
    input_trigger_pub_->publish(std_msgs::msg::Empty{});
  }

  void printAllOk()
  {
    RCLCPP_INFO(this->get_logger(), "hardware system ok");
    RCLCPP_INFO(this->get_logger(), "hri system ok");
    RCLCPP_INFO(this->get_logger(), "navigation system ok");
  }

  bool consumeAuthorizeOk()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!authorize_ok_) return false;
    authorize_ok_ = false;
    return true;
  }

  bool consumeAuthorizeRetry()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!authorize_retry_) return false;
    authorize_retry_ = false;
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
        // (lo dejo como lo tenías: trigger continuo aquí, por si el otro nodo lo requiere)
        input_trigger_pub_->publish(std_msgs::msg::Empty{});

        supervisor_interfaces::msg::SystemStatus st;
        /*if (!getLatestStatus(st))
        {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "SM_VERIFY_SYS: No message received yet on /system/status"
          );
          break;
        }*/
        state_ = State::SM_CREATE_PLAN;
        if (st.hardware && st.hri && st.navigation)
        {
          RCLCPP_INFO(this->get_logger(),
                      "SM_VERIFY_SYS: hardware/hri/navigation are OK -> SM_CREATE_PLAN");
          state_ = State::SM_CREATE_PLAN;
        }
        else
        {
          RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1500,
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
        RCLCPP_INFO(this->get_logger(), "SM_CREATE_PLAN: Create a plan with semantic command");
        std::string plan = planning_->parseSemanticCommand("Go to the entrance and wait for the door and to be oppened and navigate to the inspection point and wait until you listen continue command and navigate to the exit");
        RCLCPP_INFO(this->get_logger(), "Plan semantic parsed: %s", plan.c_str());
        planning_ -> primitiveToClips(plan);
        RCLCPP_INFO(this->get_logger(), "Plan sent to CLIPS");
        state_ = State::SM_POST_PLAN_TRIGGER;
        break;
      }
      
      case State::SM_POST_PLAN_TRIGGER:
      {
        RCLCPP_INFO(this->get_logger(),"SM_POST_PLAN_TRIGGER: publishing /input_trigger (one-shot)");
        publishInputTriggerOnce();
        state_ = State::SM_WAIT_AUTHORIZE;
        break;
      }

      case State::SM_WAIT_AUTHORIZE:
      {
        if (consumeAuthorizeOk())
        {
          RCLCPP_INFO(this->get_logger(), "SM_WAIT_AUTHORIZE: Check success -> next trigger");
          publishInputTriggerOnce();
          state_ = State::SM_WAIT_AUTHORIZE;
          break;
        }
      
        // 2) si fue failure => retry (publicaste instruction_command=1 en el callback)
        if (consumeAuthorizeRetry())
        {
          RCLCPP_INFO(this->get_logger(), "SM_WAIT_AUTHORIZE: Check failure -> retry trigger");
          publishInputTriggerOnce();
          // NO avanzas de etapa, solo reintentas y vuelves a esperar authorize
          state_ = State::SM_WAIT_AUTHORIZE;
          break;
        }
      
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "SM_WAIT_AUTHORIZE: waiting for authorize_task..."
        );
        break;
      }


      case State::SM_ON_AUTHORIZE:
      {
        // “Responder todo ok” (por ahora: solo imprimir)
        printAllOk();

        // Luego vuelve a lanzar input_trigger y regresa a esperar la siguiente llamada
        RCLCPP_INFO(this->get_logger(),
                    "SM_ON_AUTHORIZE: publishing /input_trigger (one-shot) and looping back to SM_WAIT_AUTHORIZE");
        publishInputTriggerOnce();

        state_ = State::SM_WAIT_AUTHORIZE;
        break;
      }

      case State::SM_WAIT_PLAN_RESPONSE:
      {
        RCLCPP_INFO(this->get_logger(), "SM_WAIT_PLAN_RESPONSE: waiting for response from /start_planning...");
        if (start_planning_future_.valid() &&
            start_planning_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
          auto resp = start_planning_future_.get();
          RCLCPP_INFO(this->get_logger(), "SM_WAIT_PLAN_RESPONSE: Aqui");
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
            // Decide qué hacer: reintentar o regresar a verify
            start_planning_sent_ = false;
            state_ = State::SM_VERIFY_SYS;
          }
        }
        else
        {
          RCLCPP_DEBUG_THROTTLE(
                                this->get_logger(),
                                *this->get_clock(),
                                1000,
                                "SM_WAIT_PLAN_RESPONSE: waiting response from /start_planning... 111"
                                );
        }

        break;
      }

      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown state, resetting FSM");
        state_ = State::SM_INIT;
        break;
    }
  }

  // ===== ROS =====
  rclcpp::Subscription<supervisor_interfaces::msg::SystemStatus>::SharedPtr sub_status_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr input_trigger_pub_;
  rclcpp::Service<supervisor_executor_interfaces::srv::AuthorizeTask>::SharedPtr authorize_task_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr instruction_cmd_pub_;


  // Planning tool
  std::shared_ptr<Planning> planning_;

  // ===== FSM state =====
  State state_{State::SM_INIT};
  bool authorize_ok_{false};        // success: avanzar
  bool authorize_retry_{false};     // failure: retry


  // ===== Shared state =====
  std::mutex mtx_;
  std::optional<supervisor_interfaces::msg::SystemStatus> last_status_;
  bool got_status_{false};

  // ===== Service event flag =====
  bool authorize_called_{false};

  // --- Planning service client ---
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_planning_client_;
  std::shared_future<std::shared_ptr<std_srvs::srv::Trigger::Response>> start_planning_future_;
  bool start_planning_sent_{false};

};

/*int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SupervisorManagerNode>());
  rclcpp::shutdown();
  return 0;
}*/


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SupervisorManagerNode>();

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(),
    4   // <- con 2 hilos basta
  );

  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

