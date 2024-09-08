#ifndef PICKUP_BOX_BT_HPP_
#define PICKUP_BOX_BT_HPP_

// #include "behaviortree_cpp_v3/action_node.h"
// #include "behaviortree_cpp_v3/bt_factory.h"

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

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
#include "manipulator_msgs/action/approach_object_task.hpp"

class MoveManipulator : public BT::RosActionNode<manipulator_msgs::action::ManipulatorTask>
{
public:
    MoveManipulator(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
        : BT::RosActionNode<manipulator_msgs::action::ManipulatorTask>(name, conf, params)
    {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({BT::InputPort<std::string>("task_string")});
    }

    bool setGoal(Goal& goal) override
    {
        goal.task_string = getInput<std::string>("task_string").value();
        return true;
    }

    void onHalt() override
    {
        RCLCPP_INFO(logger(), "MoveManipulator halted");
    }

    BT::NodeStatus onResultReceived(const WrappedResult& wr) override
    {
        if (wr.result->success)
        {
            RCLCPP_INFO(logger(), "MoveManipulator succeeded");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(logger(), "MoveManipulator failed");
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "MoveManipulator failed with error code %d", error);
        return BT::NodeStatus::FAILURE;
    }
};

class ApproachObject : public BT::RosActionNode<manipulator_msgs::action::ApproachObjectTask>
{
public:
    ApproachObject(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
        : BT::RosActionNode<manipulator_msgs::action::ApproachObjectTask>(name, conf, params)
    {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<float>("distance"),
            BT::InputPort<float>("distance_threshold")
        });
    }

    bool setGoal(Goal& goal) override
    {
        goal.distance = getInput<float>("distance").value();
        goal.distance_threshold = getInput<float>("distance_threshold").value();
        goal.target_pose_topic = "aruco_detect/object_pose";
        goal.is_object_detected_topic = "aruco_detect/is_detected";
        return true;
    }

    void onHalt() override
    {
        RCLCPP_INFO(logger(), "ApproachObject halted");
    }

    BT::NodeStatus onResultReceived(const WrappedResult& wr) override
    {
        if (wr.result->success)
        {
            RCLCPP_INFO(logger(), "ApproachObject succeeded");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(logger(), "ApproachObject failed");
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "ApproachObject failed with error code %d", error);
        return BT::NodeStatus::FAILURE;
    }
};


#endif 