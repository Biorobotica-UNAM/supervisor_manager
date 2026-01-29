#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <string>
#include <sstream>
#include <vector>



int main(int argc, char **argv)
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("supervisor_manager_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}