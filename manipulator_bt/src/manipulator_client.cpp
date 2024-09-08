#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "manipulator_msgs/action/manipulator_task.hpp"

class ManipulatorClient : public rclcpp::Node
{
public:
    using ManipulatorTask = manipulator_msgs::action::ManipulatorTask;
    using GoalHandleManipulatorTask = rclcpp_action::ClientGoalHandle<ManipulatorTask>;

    explicit ManipulatorClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("manipulator_client", options)
    {
        this->client_ptr_ = rclcpp_action::create_client<ManipulatorTask>(this, "task_server");

        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ManipulatorClient::send_goal, this));
    }

private:
    rclcpp_action::Client<ManipulatorTask>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void send_goal()
    {
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = ManipulatorTask::Goal();
        goal_msg.task_string = "open-gripper";  // Example task string

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<ManipulatorTask>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ManipulatorClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&ManipulatorClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&ManipulatorClient::result_callback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(std::shared_ptr<GoalHandleManipulatorTask> goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleManipulatorTask::SharedPtr,
        const std::shared_ptr<const ManipulatorTask::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback: %d%%", feedback->percentage);
    }

    void result_callback(const GoalHandleManipulatorTask::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulatorClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}