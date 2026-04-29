#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include <memory>

class LimbCommander : public rclcpp::Node
{
public:
    // alias
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

    LimbCommander() : Node("limb_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Limb Commander node has started!");

        // init action client
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/joint_trajectory_controller/follow_joint_trajectory"
        );
    }

private:
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;  // action client ptr
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LimbCommander>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
