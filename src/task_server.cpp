#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "mobile_bot/action/manipulator_task.hpp"

namespace mobile_bot_remote {
    class TaskServer : public rclcpp::Node {
    public:
        explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
            : Node("task_server", options) {
            RCLCPP_INFO(get_logger(), "Task server started");
            action_server_ = rclcpp_action::create_server<mobile_bot::action::ManipulatorTask>(
                this, "task_server", 
                std::bind(&TaskServer::goalCallback, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&TaskServer::cancelCallback, this, std::placeholders::_1),
                std::bind(&TaskServer::acceptedCallback, this, std::placeholders::_1));
        }

    private:
        rclcpp_action::Server<mobile_bot::action::ManipulatorTask>::SharedPtr action_server_;

        rclcpp_action::GoalResponse goalCallback(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const mobile_bot::action::ManipulatorTask::Goal> goal)
        {
            RCLCPP_INFO(get_logger(), "Received goal request with id %d", goal->task_number);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse cancelCallback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<mobile_bot::action::ManipulatorTask>> goal_handle)
        {
            (void)goal_handle;
            RCLCPP_INFO(get_logger(), "Received request to cancel goal");
            auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
            auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");
            arm_move_group.stop();
            gripper_move_group.stop();
            return rclcpp_action::CancelResponse::ACCEPT;
        }
        void acceptedCallback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<mobile_bot::action::ManipulatorTask>> goal_handle)
        {
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            RCLCPP_INFO(get_logger(), "Goal has been accepted");
            // std::thread{ std::bind(&TaskServer::execute, this, std::placeholders::_1), goal_handle }.detach();
        }

        
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(mobile_bot_remote::TaskServer);