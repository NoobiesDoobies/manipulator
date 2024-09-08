#include "rclcpp/rclcpp.hpp"
#include "manipulator_bt/pickup_box_bt.hpp"
#include <filesystem>
#include <iostream>

int main(int argc, char **argv){
    std::filesystem::path ros_ws_path = std::filesystem::current_path();
    std::string xml_file_name = "/src/manipulator_bt/behaviour_trees/pickup_box_bt3.xml";
    std::string xml_path = ros_ws_path.string() + xml_file_name;

    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;

    BT::NodeBuilder rotate_to_find_object = [](const std::string& name, const BT::NodeConfiguration& config){
        return std::make_unique<RotateToFindObject>(name, config);
    };
    factory.registerBuilder<RotateToFindObject>("RotateToFindObject", rotate_to_find_object);

    BT::NodeBuilder approach_object = [](const std::string& name, const BT::NodeConfiguration& config){
        return std::make_unique<ApproachObject>(name, config);
    };
    factory.registerBuilder<ApproachObject>("ApproachObject", approach_object);

    BT::NodeBuilder move_manipulator = [](const std::string& name, const BT::NodeConfiguration& config){
        return std::make_unique<MoveManipulator>(name, config);
    };
    factory.registerBuilder<MoveManipulator>("MoveManipulator", move_manipulator);
    // Create an executor to spin the node
    // auto node = std::make_shared<rclcpp::Node>("move_manipulator_node");
    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(node);

    auto tree = factory.createTreeFromFile(xml_path);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while(rclcpp::ok() && status == BT::NodeStatus::RUNNING){
        status = tree.tickRoot();
        // executor.spin_some(); 
    }


    rclcpp::shutdown();
    return 0;
}