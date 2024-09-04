#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace mobile_bot_remote {
    class TaskServer : public rclcpp::Node {
    public:
        explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
            : Node("task_server", options) {
            RCLCPP_INFO(get_logger(), "Task server started");
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(mobile_bot_remote::TaskServer);