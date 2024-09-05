#ifndef PICKUP_BOX_BT_HPP_
#define PICKUP_BOX_BT_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


class RotateToFindObject: public BT::StatefulActionNode
{
  public:
    RotateToFindObject(const std::string& name, const BT::NodeConfiguration& config):
      BT::StatefulActionNode(name, config){
        node_ = rclcpp::Node::make_shared(name);
        twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/omnidirectional_controller/cmd_vel", 10);
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
class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string& name) :
      BT::SyncActionNode(name, {})
  {}

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};


class GripperInterface
{
public:
  GripperInterface() : _open(true) {}

  BT::NodeStatus open()
  {
    _open = true;
    std::cout << "Gripper open" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus close()
  {
    _open = false;
    std::cout << "Gripper close" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

private:
  bool _open;
};

#endif 