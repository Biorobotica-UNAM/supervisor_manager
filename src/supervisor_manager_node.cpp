#include <rclcpp/rclcpp.hpp>
#include <supervisor_interfaces/msg/system_status.hpp>

#include <mutex>
#include <optional>
#include <string>

class SupervisorManagerNode : public rclcpp::Node
{
    public:
        SupervisorManagerNode() : Node("supervisor_manager_node")
        {
            RCLCPP_INFO(this->get_logger(), "supervisor_manager_node: started");
            sub_ = this->create_subscription<supervisor_interfaces::msg::SystemStatus>("/system/status",
                                                                                        rclcpp::QoS(10),
                                                                                        std::bind(&SupervisorManagerNode::statusCallback, this, std::placeholders::_1));

            // Tick FSM (10 Hz)
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                            std::bind(&SupervisorManagerNode::tick, this));
        }

    private:
        enum class State : uint8_t
        {
            SM_START,
            SM_VERIFY_SYS,
            SM_SEND_OK,
            SM_SEND_LISTEN
        };

        void statusCallback(const supervisor_interfaces::msg::SystemStatus::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            last_status_ = *msg;
            got_status_ = true;
        }

        void tick()
        {
            switch (state_)
            {
                case State::SM_START:              
                    RCLCPP_INFO(this->get_logger(), "Estado 0 (START): Bienvenida / inicio de FSM");
                    RCLCPP_INFO(this->get_logger(), "Esperando mensajes en /system/status ...");
                    state_ = State::SM_VERIFY_SYS;  // pasa a estado 1
                break;
      

                case State::SM_VERIFY_SYS:
                {
                    // Se queda en este estado para siempre.
                    supervisor_interfaces::msg::SystemStatus local_copy;
                    bool have_msg = false;

                    {
                        std::lock_guard<std::mutex> lock(mtx_);
                        have_msg = got_status_;
                        if (have_msg) local_copy = *last_status_;
                    }
                
                    if (!have_msg)
                    {
                        RCLCPP_WARN_THROTTLE(
                                this->get_logger(),
                                *this->get_clock(),
                                2000,
                                "Estado 1 (MONITOR): aún no llega ningún SystemStatus...");
                        break;
                    }
                
                    // ---- Imprime lo recibido ----
                    // Asumiendo que existe: bool hardware
                    RCLCPP_INFO_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            500,
                            "Estado 1 (MONITOR): recibido SystemStatus | hardware=%s",
                            local_copy.hardware ? "true" : "false");

                    RCLCPP_INFO_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            500,
                            "Estado 1 (MONITOR): recibido SystemStatus | hri=%s",
                            local_copy.hri ? "true" : "false");
                            
                    RCLCPP_INFO_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            500,
                            "Estado 1 (MONITOR): recibido SystemStatus | navigation=%s",
                            local_copy.navigation ? "true" : "false");
                    

                break; }

                default:
                    RCLCPP_ERROR(this->get_logger(), "Estado desconocido, reiniciando FSM");
                    state_ = State::SM_START
                    break;
                
    
            } // switch
        }
            

  // ROS
rclcpp::Subscription<supervisor_interfaces::msg::SystemStatus>::SharedPtr sub_;
rclcpp::TimerBase::SharedPtr timer_;

// Estado FSM
State state_{State::START};

// Último mensaje recibido
std::mutex mtx_;
std::optional<supervisor_interfaces::msg::SystemStatus> last_status_;
bool got_status_{false};
  
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SupervisorManagerNode>());
  rclcpp::shutdown();
  return 0;
}
