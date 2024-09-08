#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "manipulator_msgs/action/manipulator_task.hpp"

#include <memory>

namespace manipulator_server_remote
{
    class TaskServer : public rclcpp::Node
    {
    public:
        explicit TaskServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("task_server", options)
        {
            RCLCPP_INFO(get_logger(), "Task server started");
            action_server_ = rclcpp_action::create_server<manipulator_msgs::action::ManipulatorTask>(
                this, "task_server",
                std::bind(&TaskServer::goalCallback, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&TaskServer::cancelCallback, this, std::placeholders::_1),
                std::bind(&TaskServer::acceptedCallback, this, std::placeholders::_1));
        }

    private:
        rclcpp_action::Server<manipulator_msgs::action::ManipulatorTask>::SharedPtr action_server_;

        rclcpp_action::GoalResponse goalCallback(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const manipulator_msgs::action::ManipulatorTask::Goal> goal)
        {
            RCLCPP_INFO(get_logger(), "Received goal request %s", goal->task_string.c_str());
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse cancelCallback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<manipulator_msgs::action::ManipulatorTask>> goal_handle)
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
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<manipulator_msgs::action::ManipulatorTask>> goal_handle)
        {
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            RCLCPP_INFO(get_logger(), "Goal has been accepted");
            std::thread{std::bind(&TaskServer::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<manipulator_msgs::action::ManipulatorTask>> goal_handle)
        {
            RCLCPP_INFO(get_logger(), "Executing goal");
            auto result = std::make_shared<manipulator_msgs::action::ManipulatorTask::Result>();

            // MoveIt 2 Interface
            auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
            auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

            moveit::core::RobotStatePtr arm_current_state = arm_move_group.getCurrentState();
            // auto arm_model_group = arm_move_group.getCurrentState()->getRobotModel()->getJointModelGroup(arm_move_group.getName());
            // std::vector<double> arm_joint_group_positions;
            // arm_current_state->copyJointGroupPositions(arm_model_group, arm_joint_group_positions);


            // Define joint goals
            std::vector<double> arm_joint_goal;
            std::vector<double> gripper_joint_goal;

            std::string task_string = goal_handle->get_goal()->task_string;

            RCLCPP_INFO(get_logger(), "Task string: %s", task_string.c_str());

            if(task_string == "open-gripper"){
                // arm_joint_goal = arm_joint_group_positions;
                RCLCPP_INFO(get_logger(), "Opening gripper");
                arm_joint_goal = {0.0, 0.0, 0.0};
                gripper_joint_goal = {-1.0, 1.0};
            } 
            else if(task_string == "close-gripper"){
                RCLCPP_INFO(get_logger(), "Closing gripper");
                arm_joint_goal = {0.0, 0.0, 0.0};
                gripper_joint_goal = {0.0, 0.0};
            }
            else if(task_string == "lower-arm-and-grip"){
                arm_joint_goal = {-0.11, -1, 0.212};
                gripper_joint_goal = {-0.25, 0.25};
            }
            else if(task_string == "raise-arm-and-grip"){
                arm_joint_goal = {0.0, 0.0, 0.0};
                gripper_joint_goal = {-0.25, 0.25};
            }
            else{
                RCLCPP_ERROR(get_logger(), "Invalid task string");
                result->success = false;
                goal_handle->abort(result);
            }


            // Set joint goal
            bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
            bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);
            if (!arm_within_bounds | !gripper_within_bounds)
            {
                RCLCPP_WARN(get_logger(),
                    "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");

                result->success = false;
                goal_handle->abort(result);
                return;
            }


            // Plan the motion
            moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
            moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
            bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            bool gripper_plan_success = (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            

            if(arm_plan_success && gripper_plan_success)
            {
                RCLCPP_INFO(get_logger(), "Planner SUCCEED, moving the arme and the gripper");
                arm_move_group.move();
                gripper_move_group.move();
            }    
            else
            {
                result->success = false;
                goal_handle->abort(result);  // Added this line
                RCLCPP_ERROR(get_logger(), "One or more planners failed!");
                return;
            }

            // Set the result
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal succeeded");

        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(manipulator_server_remote::TaskServer);