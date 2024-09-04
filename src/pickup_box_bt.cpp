#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#define DEFAULT_BT_XML "/home/carlios/manipulator_ws/src/manipulator/behaviour_trees/pickup_box.xml"


using namespace BT;

// Custom node classes declaration (or header inclusion)
class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string& name) :
      BT::SyncActionNode(name, {})
  {}

  // You must override the virtual function tick()
  NodeStatus tick() override
  {
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return NodeStatus::SUCCESS;
  }
};

class IsArucoDetected : public BT::SyncActionNode
{
public:
    IsArucoDetected(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override
    {
        // Custom logic to check for ArUco tag detection
        // bool detected = detectArucoTag();
        bool detected = false; // Dummy value
        return detected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class IsCloseEnough : public BT::SyncActionNode
{
public:
    IsCloseEnough(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override
    {
        // Custom logic to check if the robot is close enough to the ArUco tag
        // double distance = getDistanceToArucoTag();
        double distance = 0.2; // Dummy value
        return distance < 0.3 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("distance") };
    }
};

class RotateToSearch : public BT::AsyncActionNode
{
public:
    RotateToSearch(const std::string& name) : BT::AsyncActionNode(name, {}), is_rotating(true) {}
    
    BT::NodeStatus tick() override
    {
        if (is_rotating)
        {
            // Custom logic to rotate and search for ArUco tags
            // rotateRobot();
            
            // Simulate an ongoing rotation operation
            return BT::NodeStatus::RUNNING;
        }
        else
        {
            return BT::NodeStatus::SUCCESS;
        }
    }

    void halt() override
    {
        is_rotating = false; // Stop rotating when halted
    }

private:
    bool is_rotating;
};


class LowerArm : public BT::SyncActionNode
{
public:
    LowerArm(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override
    {
        // Custom logic to lower the arm
        // lowerArm();
        return BT::NodeStatus::SUCCESS;
    }
};

class RaiseArm : public BT::SyncActionNode
{
public:
    RaiseArm(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override
    {
        // Custom logic to raise the arm
        // raiseArm();

        return BT::NodeStatus::SUCCESS;
    }
};

class OpenGripper : public BT::SyncActionNode
{
public:
    OpenGripper(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override
    {
        // Custom logic to open the gripper
        // gripper.open();
        return BT::NodeStatus::SUCCESS;
    }
};

class CloseGripper : public BT::SyncActionNode
{
public:
    CloseGripper(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override
    {
        // Custom logic to close the gripper
        // gripper.close();
        return BT::NodeStatus::SUCCESS;
    }
};


int main()
{
    // Create the factory
    BT::BehaviorTreeFactory factory;

    // Register custom nodes
    factory.registerNodeType<ApproachObject>("ApproachObject");
    factory.registerNodeType<IsArucoDetected>("IsArucoDetected");
    // factory.registerNodeType<IsArucoDetected>("is_aruco_detected_after_rotating"); // Registering twice for different instances
    factory.registerNodeType<IsCloseEnough>("IsCloseEnough");
    factory.registerNodeType<RotateToSearch>("RotateToSearch");
    factory.registerNodeType<LowerArm>("LowerArm");
    factory.registerNodeType<RaiseArm>("RaiseArm");
    factory.registerNodeType<OpenGripper>("OpenGripper");
    factory.registerNodeType<CloseGripper>("CloseGripper");

    // Register Gripper actions

    // Create the tree from the XML file
    auto tree = factory.createTreeFromFile(DEFAULT_BT_XML);

    // Loop to continuously tick the tree
    while (true)
    {
        BT::NodeStatus status = tree.tickRoot();
        if (status == BT::NodeStatus::SUCCESS)
        {
            std::cout << "Task completed successfully." << std::endl;
            break;
        }
        else if (status == BT::NodeStatus::FAILURE)
        {
            std::cout << "Task failed." << std::endl;
            break;
        }

        // Simulate a loop rate (sleep or delay) to control the tick frequency
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
