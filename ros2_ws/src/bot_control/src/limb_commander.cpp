#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include <memory>
#include <vector>
#include <chrono>
#include <string>

// param names
const std::string JOINTS_PARAM_NAME = "joints";
const std::string ACTION_SERVER_NAME_PARAM_NAME = "action_server_name";

class LimbCommander : public rclcpp::Node
{
public:
    // alias
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

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
    }

    void send_goal()
    {
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

        auto goal = FollowJointTrajectory::Goal();
        goal.trajectory.joint_names = this->joints_;
        
    }

private:
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;  // action client ptr

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
