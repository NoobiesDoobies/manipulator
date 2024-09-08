#ifndef PICKUP_BOX_BT_HPP_
#define PICKUP_BOX_BT_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "manipulator_msgs/action/manipulator_task.hpp"

class RotateToFindObject: public BT::StatefulActionNode
{
  public:
    RotateToFindObject(const std::string& name, const BT::NodeConfiguration& config):
      BT::StatefulActionNode(name, config){
        node_ = rclcpp::Node::make_shared(name);
        twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/omni_wheel_controller/cmd_vel", 10);
        is_detected_sub_ = node_->create_subscription<std_msgs::msg::Bool>("/aruco_detect/is_detected", 10, std::bind(&RotateToFindObject::isDetectedCallback, this, std::placeholders::_1));

      }

    static BT::PortsList providedPorts(){
      return{BT::InputPort<float>("angular_speed")};
    }

    BT::NodeStatus onStart() override
    {
      getInput("angular_speed", angular_speed_);

      RCLCPP_INFO(node_->get_logger(), "RotateToFindObject started with angular speed: %f", angular_speed_);
      is_detected_ = false;
      
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      if(is_detected_)
      {
        RCLCPP_INFO(node_->get_logger(), "Object detected");
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(), "Object not detected, Rotating to find the object");
        auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
        twist_msg->header.stamp = node_->now();
        twist_msg->header.frame_id = "omnibot_base_link";
        twist_msg->twist.angular.z = angular_speed_;

        twist_pub_->publish(*twist_msg);
        return BT::NodeStatus::RUNNING;
      }


    }

    void onHalted() override
    {
      RCLCPP_INFO(node_->get_logger(), "RotateToFindObject halted");
    }

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_detected_sub_;
    float angular_speed_;
    bool is_detected_;

    void isDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      is_detected_ = msg->data;
    }

 
};


// Example of custom SyncActionNode (synchronous action)
// without ports.
class ApproachObject: public BT::StatefulActionNode
{
  public:
    ApproachObject(const std::string& name, const BT::NodeConfiguration& config):
      BT::StatefulActionNode(name, config){
        node_ = rclcpp::Node::make_shared(name);
        twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/omni_wheel_controller/cmd_vel", 10);
        object_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/aruco_detect/object_pose", 10, std::bind(&ApproachObject::objectDistCallback, this, std::placeholders::_1));
        is_detected_sub_ = node_->create_subscription<std_msgs::msg::Bool>("/aruco_detect/is_detected", 10, std::bind(&ApproachObject::isDetectedCallback, this, std::placeholders::_1));
      }

    static BT::PortsList providedPorts(){
      return{
          BT::InputPort<float>("stop_distance"),
          BT::InputPort<float>("approaching_velocity"),
          BT::InputPort<float>("distance_error"),
        };
    }

    BT::NodeStatus onStart() override
    {
      getInput("stop_distance", stop_distance_);
      getInput("approaching_velocity", approaching_velocity_);
      getInput("distance_error", distance_error_);

      RCLCPP_INFO(node_->get_logger(), "ApproachObject with velocity: %f and stop distance at: %f with error of: %f", approaching_velocity_, stop_distance_, distance_error_);
      
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      if(abs(object_dist_ - stop_distance_) < distance_error_)
      {
        RCLCPP_INFO(node_->get_logger(), "Object reached");
        return BT::NodeStatus::SUCCESS;
      }
      else{
        RCLCPP_INFO(node_->get_logger(), "Approaching object");
        auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
        twist_msg->header.stamp = node_->now();
        twist_msg->header.frame_id = "omnibot_base_link";
        twist_msg->twist.linear.x = approaching_velocity_;

        return BT::NodeStatus::RUNNING;
      }
    }

    void onHalted() override
    {
      RCLCPP_INFO(node_->get_logger(), "ApproachObject halted");
    }

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_detected_sub_;
    float stop_distance_;
    float approaching_velocity_;
    bool is_detected_;
    float object_dist_;
    float distance_error_;

    void objectDistCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      object_dist_ = sqrt(pow(msg->pose.position.x, 2) + pow(msg->pose.position.y, 2));
    }

    void isDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      is_detected_ = msg->data;
    }


 
};




class MoveManipulator : public BT::SyncActionNode
{
public:
    MoveManipulator(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), result_code_(rclcpp_action::ResultCode::UNKNOWN)
    {
        node_ = rclcpp::Node::make_shared(name);
        manipulator_task_client_ = rclcpp_action::create_client<manipulator_msgs::action::ManipulatorTask>(node_, "task_server");
        timer_ = node_->create_wall_timer( std::chrono::milliseconds(500), std::bind(&MoveManipulator::send_goal, this));
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("task_string")};
    }
    
    BT::NodeStatus tick() override
    {
        if (!getInput<std::string>("task_string", task_string_))
        {
            throw BT::RuntimeError("missing required input [task_string]");
        }
        while(rclcpp::ok() && result_code_ == rclcpp_action::ResultCode::UNKNOWN)
        {
            rclcpp::spin_some(node_);
        }
        return BT::NodeStatus::SUCCESS;
    }



private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<manipulator_msgs::action::ManipulatorTask>::SharedPtr manipulator_task_client_; // Action client as a class member
    rclcpp_action::ClientGoalHandle<manipulator_msgs::action::ManipulatorTask>::SharedPtr goal_handle_;
    rclcpp_action::ResultCode result_code_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string task_string_;
    void send_goal()
    {
        // if (!manipulator_task_client_->wait_for_action_server(std::chrono::seconds(10))) {
        //     RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
        //     rclcpp::shutdown();
        //     return;
        // }

        auto goal_msg = manipulator_msgs::action::ManipulatorTask::Goal();
        goal_msg.task_string = task_string_;

        RCLCPP_INFO(node_->get_logger(), "Sending goal %s", task_string_.c_str());

        auto send_goal_options = rclcpp_action::Client<manipulator_msgs::action::ManipulatorTask>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&MoveManipulator::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&MoveManipulator::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&MoveManipulator::resultCallback, this, std::placeholders::_1);

        manipulator_task_client_->async_send_goal(goal_msg, send_goal_options);
    }
    void goalResponseCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<manipulator_msgs::action::ManipulatorTask>> future)
    {
        goal_handle_ = future;
        auto goal_handle = future;
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedbackCallback(
        rclcpp_action::ClientGoalHandle<manipulator_msgs::action::ManipulatorTask>::SharedPtr,
        const std::shared_ptr<const manipulator_msgs::action::ManipulatorTask::Feedback> feedback)
    {
        RCLCPP_INFO(node_->get_logger(), "Received feedback: %d%%", feedback->percentage);
    }

    void resultCallback(const rclcpp_action::ClientGoalHandle<manipulator_msgs::action::ManipulatorTask>::WrappedResult & result)
    {
        result_code_ = result.code;
        if (result.result) {
            std::stringstream ss;
            ss << "Result received: " << result.result->success;
            RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
        }
    }
};

#endif 