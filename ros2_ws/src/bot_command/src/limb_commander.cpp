#include "bot_kinematics/ik_solver.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace bot_command {
// param names
const std::string JOINTS_PARAM_NAME = "joints";
const std::string TARGET_TOPIC_PARAM_NAME = "target";
const std::string L1_PARAM_NAME = "l1_len";
const std::string L2_PARAM_NAME = "l2_len";
const std::string L3_PARAM_NAME = "l3_len";

// topic names
const std::string TARGET_TOPIC_NAME =
    "target"; // must match in limb_trajectory_generator.cpp
const std::string JOINT_COMMAND_TOPIC =
    "joint_group_position_controller/commands"; //  comes from
                                                //  bot_control/controllers.yaml

class LimbCommander : public rclcpp::Node {
public:
  LimbCommander() : Node("limb_commander") {
    bot_kinematics::LimbDimensions dims;

    // declare and get params
    try {
      joints_ =
          this->declare_parameter<std::vector<std::string>>(JOINTS_PARAM_NAME);

      // get limb dims from params
      dims.l1 = this->declare_parameter<double>(L1_PARAM_NAME);
      dims.l2 = this->declare_parameter<double>(L2_PARAM_NAME);
      dims.l3 = this->declare_parameter<double>(L3_PARAM_NAME);
    } catch (
        const rclcpp::exceptions::UninitializedStaticallyTypedParameterException
            &e) {
      // not able to resolve all params
      RCLCPP_FATAL(this->get_logger(), "MISSING CRITICAL PARAMETERS: %s",
                   e.what());
      throw e; // kill the node
    }

    // init IK solver
    ik_solver_ = std::make_unique<bot_kinematics::IkSolver>(dims);

    // init subscription to target topic (reciving cartesian points)
    target_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
        TARGET_TOPIC_NAME, 1,
        std::bind(&LimbCommander::target_callback, this,
                  std::placeholders::_1));

    // init publisher of joint angles (to controller)
    joint_command_publisher_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(
            JOINT_COMMAND_TOPIC, 1);

    RCLCPP_INFO(this->get_logger(), "limb_commander node initialized.");
  }

private:
  void target_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    // extract cart coords from msg
    double x = msg->x;
    double y = msg->y;
    double z = msg->z;

    // run IK and store calculated limb joint angles
    bot_kinematics::LimbJointAngles joint_angles =
        ik_solver_->calculate_ik(x, y, z);

    // build cmd
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = joint_angles.to_vector();

    // publish command (for controller)
    joint_command_publisher_->publish(cmd);
  }

  // subscriber for cartesian point targets (from gait planner)
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_subscriber_;

  // publisher of joint angles (to controller)
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      joint_command_publisher_;

  // vector with joint names
  std::vector<std::string> joints_;

  // instance of ik solver
  std::unique_ptr<bot_kinematics::IkSolver> ik_solver_;
};
} // namespace bot_command

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bot_command::LimbCommander>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
