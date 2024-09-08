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

    auto move_manipulator_nh = std::make_shared<rclcpp::Node>("move_manipulator_client");
    BT::RosNodeParams move_manipulator_params;
    move_manipulator_params.nh = move_manipulator_nh;
    move_manipulator_params.default_port_value = "task_server";
    factory.registerNodeType<MoveManipulator>("MoveManipulator", move_manipulator_params);

    auto tree = factory.createTreeFromFile(xml_path);

    while(rclcpp::ok() ){
        tree.tickWhileRunning();
        // executor.spin_some(); 
    }


    rclcpp::shutdown();
    return 0;
}