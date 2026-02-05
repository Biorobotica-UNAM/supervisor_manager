#include <rclcpp/rclcpp.hpp>

#include <supervisor_interfaces/msg/system_status.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>
#include <optional>
#include <string>
#include <array>

#include <planning_tools/Planning.hpp>

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

    // ACK: “ya se ejecutó / ya se dijo” la instrucción enviada (por ahora Empty)
    instruction_done_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/supervisor_manager/instruction_done",
      rclcpp::QoS(10),
      std::bind(&SupervisorManagerNode::instructionDoneCallback, this, std::placeholders::_1)
    );

    // --- Publishers ---
    input_trigger_pub_ = this->create_publisher<std_msgs::msg::Empty>("/input_trigger", 10);

    instruction_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/supervisor_manager/instruction", 10
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
    SM_SEND_OK,
    SM_CREATE_PLAN,
    SM_EXECUTING_PLAN
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

  void instructionDoneCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    instruction_done_ = true;
  }

  // ===== Helpers =====
  bool consumeInstructionDone()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!instruction_done_) return false;
    instruction_done_ = false;
    return true;
  }

  bool getLatestStatus(supervisor_interfaces::msg::SystemStatus & out)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!got_status_ || !last_status_.has_value()) return false;
    out = *last_status_;
    return true;
  }

  void publishInstructionSay(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = "Justina say " + text;  // formato pedido
    instruction_pub_->publish(msg);
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
        // Mantengo tu trigger por si el otro nodo lo necesita
        input_trigger_pub_->publish(std_msgs::msg::Empty{});

        supervisor_interfaces::msg::SystemStatus st;
        if (!getLatestStatus(st))
        {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "SM_VERIFY_SYS: No message received yet on /system/status"
          );
          break;
        }

        // Si todo está ok (los tres bool true) -> siguiente estado
        if (st.hardware && st.hri && st.navigation)
        {
          RCLCPP_INFO(this->get_logger(), "SM_VERIFY_SYS: hardware/hri/navigation are OK -> SM_SEND_OK");
          ok_step_ = 0;
          waiting_ack_ = false;
          state_ = State::SM_SEND_OK;
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

      case State::SM_SEND_OK:
      {
        // Mensajes a decir, uno por uno
        static const std::array<std::string, 3> kOkMessages = {
          "hardware system ok",
          "hri system ok",
          "navigation system ok"
        };

        if (ok_step_ >= kOkMessages.size())
        {
          RCLCPP_INFO(this->get_logger(), "SM_SEND_OK: all OK messages sent -> SM_CREATE_PLAN");
          state_ = State::SM_CREATE_PLAN;
          break;
        }

        if (!waiting_ack_)
        {
          // Enviar la instrucción actual
          publishInstructionSay(kOkMessages[ok_step_]);
          waiting_ack_ = true;

          // TODO: aquí debería dispararse la acción real (TTS/ejecutor).
          // Por ahora asumimos que el ejecutor avisará con un std_msgs/Empty en:
          // /supervisor_manager/instruction_done
          RCLCPP_INFO(this->get_logger(), "SM_SEND_OK: sent instruction %zu/3, waiting ACK...",
                      ok_step_ + 1);
          break;
        }

        // Esperar el ACK (Empty)
        if (consumeInstructionDone())
        {
          RCLCPP_INFO(this->get_logger(), "SM_SEND_OK: ACK received for %zu/3", ok_step_ + 1);
          ok_step_++;
          waiting_ack_ = false;  // listo para mandar la siguiente
        }
        else
        {
          RCLCPP_DEBUG_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "SM_SEND_OK: waiting ACK on /supervisor_manager/instruction_done..."
          );
        }

        break;
      }

      case State::SM_CREATE_PLAN:
      {
        RCLCPP_INFO(this->get_logger(), "SM_CREATE_PLAN: calling planning->startPlanning()");

        if (!planning_)
        {
          RCLCPP_WARN(this->get_logger(), "SM_CREATE_PLAN: planning_ not initialized yet, staying here...");
          break;
        }

        planning_->startPlanning();
        state_ = State::SM_EXECUTING_PLAN;
        break;
      }

      case State::SM_EXECUTING_PLAN:
      {
        // Aquí únicamente se ejecuta un plan generado. Por ahora, se queda aquí.
        // TODO: cuando esté listo el ejecutor, aquí se monitorea avance/finalización del plan.
        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          3000,
          "SM_EXECUTING_PLAN: executing plan (placeholder) - staying in this state."
        );
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
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr instruction_done_sub_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr input_trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr instruction_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  // Planning tool
  std::shared_ptr<Planning> planning_;

  // ===== FSM state =====
  State state_{State::SM_INIT};

  // ===== Status memory =====
  std::mutex mtx_;
  std::optional<supervisor_interfaces::msg::SystemStatus> last_status_;
  bool got_status_{false};

  // ===== Instruction ACK flow =====
  bool instruction_done_{false}; // set by callback, consumed by FSM
  size_t ok_step_{0};
  bool waiting_ack_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SupervisorManagerNode>());
  rclcpp::shutdown();
  return 0;
}
