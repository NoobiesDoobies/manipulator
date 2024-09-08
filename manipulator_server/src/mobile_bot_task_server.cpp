#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "manipulator_msgs/action/go_to_task.hpp"
#include "manipulator_msgs/action/approach_object_task.hpp"

#include <memory>

namespace mobile_bot_server_remote
{
    class TaskServer: public rclcpp::Node
    {
    public:
        explicit TaskServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("mobile_bot_task_server", options)
        {
            RCLCPP_INFO(get_logger(), "Task server started");
            action_server_ = rclcpp_action::create_server<manipulator_msgs::action::ApproachObjectTask>(
                this, "approach_object",
                std::bind(&TaskServer::goalCallback, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&TaskServer::cancelCallback, this, std::placeholders::_1),
                std::bind(&TaskServer::acceptedCallback, this, std::placeholders::_1));

            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/omni_wheel_controller/cmd_vel", 10);       
        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_object_detected_sub_;
        geometry_msgs::msg::PoseStamped target_pose_;
        std_msgs::msg::Bool is_object_detected_;
        
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

        rclcpp_action::Server<manipulator_msgs::action::ApproachObjectTask>::SharedPtr action_server_;
               rclcpp_action::GoalResponse goalCallback(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const manipulator_msgs::action::ApproachObjectTask::Goal> goal)
        {
            RCLCPP_INFO(get_logger(), "Apparoaching object at topic %s", goal->target_pose_topic.c_str());
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse cancelCallback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<manipulator_msgs::action::ApproachObjectTask>> goal_handle)
        {
            (void)goal_handle;
            RCLCPP_INFO(get_logger(), "Received request to cancel goal");

            geometry_msgs::msg::TwistStamped cmd_vel;
            cmd_vel.header.stamp = now();
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.linear.y = 0;
            cmd_vel.twist.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_vel);

            return rclcpp_action::CancelResponse::ACCEPT;
        }
        void acceptedCallback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<manipulator_msgs::action::ApproachObjectTask>> goal_handle)
        {
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            RCLCPP_INFO(get_logger(), "Goal has been accepted");
            target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                goal_handle->get_goal()->target_pose_topic, 10, std::bind(&TaskServer::targetPoseCallback, this, std::placeholders::_1));

            is_object_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                goal_handle->get_goal()->is_object_detected_topic, 10, std::bind(&TaskServer::isObjectDetectedCallback, this, std::placeholders::_1));
            

            std::thread{std::bind(&TaskServer::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<manipulator_msgs::action::ApproachObjectTask>> goal_handle)
        {
            RCLCPP_INFO(get_logger(), "Executing goal");
            RCLCPP_INFO(get_logger(), "Target pose x: %f, y: %f, z: %f", target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z);
            RCLCPP_INFO(get_logger(), "Distance threshold: %f", goal_handle->get_goal()->distance_threshold);
            RCLCPP_INFO(get_logger(), "Distance: %f", goal_handle->get_goal()->distance);

            auto result = std::make_shared<manipulator_msgs::action::ApproachObjectTask::Result>();

            if(abs((target_pose_.pose.position.z - goal_handle->get_goal()->distance) > goal_handle->get_goal()->distance_threshold) || !is_object_detected_.data){
                
                RCLCPP_INFO(get_logger(), "Approaching object at pose x: %f, y: %f, z: %f, dist: %f, thres: %f", target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z, goal_handle->get_goal()->distance, goal_handle->get_goal()->distance_threshold);
                geometry_msgs::msg::TwistStamped cmd_vel;
                cmd_vel.header.stamp = now();
                // cmd_vel.twist.linear.x = -1.5* (goal_handle->get_goal()->distance - target_pose_.pose.position.z);
                // cmd_vel.twist.linear.y = -1.5*target_pose_.pose.position.y;
                // cmd_vel.twist.angular.z = 0.0;

                // if(abs(cmd_vel.twist.linear.x) < 0.1 && abs(cmd_vel.twist.linear.x) != 0){
                //     cmd_vel.twist.linear.x = 0.1 * (cmd_vel.twist.linear.x/abs(cmd_vel.twist.linear.x));
                // }
                // else if(abs(cmd_vel.twist.linear.x) > 0.3){
                //     cmd_vel.twist.linear.x = 0.3 * (cmd_vel.twist.linear.x/abs(cmd_vel.twist.linear.x));
                // }

                cmd_vel.twist.linear.x = 0.5 * (goal_handle->get_goal()->distance - target_pose_.pose.position.z)/abs(goal_handle->get_goal()->distance - target_pose_.pose.position.z);
                cmd_vel.twist.linear.y = 0.0;

                cmd_vel_pub_->publish(cmd_vel);
            }
            geometry_msgs::msg::TwistStamped cmd_vel;
            cmd_vel.header.stamp = now();
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.linear.y = 0;
            cmd_vel.twist.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_vel);

            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal succeeded");

        } 

        // create callback for subscription
        void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            RCLCPP_INFO(get_logger(), "Received target pose");
            target_pose_ = *msg;
        }

        void isObjectDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
        {
            RCLCPP_INFO(get_logger(), "Received object detection status");
            is_object_detected_ = *msg;
        }
    
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(mobile_bot_server_remote::TaskServer);

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mobile_bot_server_remote::TaskServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}