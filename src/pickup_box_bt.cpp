#include "rclcpp/rclcpp.hpp"
#include "mobile_bot/pickup_box_bt.hpp"
#include <filesystem>
#include <iostream>
// #define DEFAULT_BT_XML "/home/carlios/manipulator_ws/src/manipulator/behaviour_trees/pickup_box.xml"

int main(int argc, char **argv){
    std::filesystem::path ros_ws_path = std::filesystem::current_path();
    std::string xml_file_name = "/src/manipulator/behaviour_trees/pickup_box_bt3.xml";
    std::string xml_path = ros_ws_path.string() + xml_file_name;

    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;

    BT::NodeBuilder rotate_to_find_object = [](const std::string& name, const BT::NodeConfiguration& config){
        return std::make_unique<RotateToFindObject>(name, config);
    };

    factory.registerBuilder<RotateToFindObject>("RotateToFindObject", rotate_to_find_object);

    auto tree = factory.createTreeFromFile(xml_path);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while(rclcpp::ok() && status == BT::NodeStatus::RUNNING){
        status = tree.tickRoot();
    }

    return 0;
}