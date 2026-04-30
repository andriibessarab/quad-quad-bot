#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <memory>
#include <vector>
#include <chrono>
#include <functional>
#include <string>

// param names
const std::string JOINTS_PARAM_NAME = "joints";
const std::string ACTION_SERVER_NAME_PARAM_NAME = "action_server_name";

class LimbCommander : public rclcpp::Node
{
public:
    // alias
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    LimbCommander() : Node("limb_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Limb Commander node has started!");

        // declare params
        this->declare_parameter<std::string>(ACTION_SERVER_NAME_PARAM_NAME, std::string{});
        this->declare_parameter<std::vector<std::string>>(JOINTS_PARAM_NAME, std::vector<std::string>{});  // joints

        // get params
        action_server_name_ = this->get_parameter(ACTION_SERVER_NAME_PARAM_NAME).as_string();
        joints_ = this->get_parameter(JOINTS_PARAM_NAME).as_string_array();

        // init action client
        this->action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            action_server_name_
        );

        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&LimbCommander::send_goal, this)

        );
    }

    void send_goal()
    {
        // cancel timer so it only fires ones
        this->timer_->cancel();

        // wait for action server to turn on
        while (!action_client_->wait_for_action_server(std::chrono::milliseconds(1000)))
        {
            if (!rclcpp::ok())  // hande interruption
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for action server.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Action server not available, waiting...");
        }
        RCLCPP_INFO(this->get_logger(), "Action server found!");

        // define goal
        auto goal = FollowJointTrajectory::Goal();

        // set joints for goal
        goal.trajectory.joint_names = this->joints_;

        // sample trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {0.5,0.2,-0.5};
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);

        // add points for goal
        goal.trajectory.points.push_back(point);

        // set callback methods in goal options
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&LimbCommander::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&LimbCommander::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&LimbCommander::result_callback, this, std::placeholders::_1);

        // Send goal
        RCLCPP_INFO(this->get_logger(), "Sending goal...");
        this->action_client_->async_send_goal(goal, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Goal sent!");
    }

    void goal_response_callback(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle)
    {
        // log goal status
        if (goal_handle == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, awaiting result");
        }
    }

    auto feedback_callback(
        GoalHandleFollowJointTrajectory::SharedPtr goal_handle,
        const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) -> void
    {
        // log current positions
        if (feedback->actual.positions.size() == joints_.size())
        {
            RCLCPP_INFO(this->get_logger(), "Current joint positions: %.2f %.2f %.2f",
            feedback->actual.positions[0], feedback->actual.positions[1], feedback->actual.positions[2]);
        }
        else
            RCLCPP_WARN(this->get_logger(), "Received feedback with unexpected joint count!");
    }

    void result_callback(const GoalHandleFollowJointTrajectory::WrappedResult &result)
    {
        // log result status
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Target reached successfully!");
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
    }

private:
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;  // action client ptr
    rclcpp::TimerBase::SharedPtr timer_;

    //params
    std::string action_server_name_;
    std::vector<std::string> joints_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LimbCommander>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
