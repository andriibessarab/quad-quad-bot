#include "bot_kinematics/ik_solver.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

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

// other consts
const std::string TARGET_TOPIC_NAME =
    "target"; // must match in limb_trajectory_generator.cpp

class LimbCommander : public rclcpp::Node {
public:
  LimbCommander() : Node("limb_controller") {
    RCLCPP_INFO(this->get_logger(), "Limb Commander node has started!");

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

    // init publisher of joint trajectories (to controller)
    joint_command_publisher_ =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 1);

    RCLCPP_INFO(this->get_logger(), "Waiting for target points...");
  }

private:
  void target_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    // extract cart coords from msg
    double x = msg->x;
    double y = msg->y;
    double z = msg->z;

    RCLCPP_INFO(this->get_logger(),
                "New target received: x=%.2f, y=%.2f, z=%.2f", x, y, z);

    // run IK and store calculated limb joint angles
    bot_kinematics::LimbJointAngles joint_angles =
        ik_solver_->calculate_ik(x, y, z);

    // build cmd
    trajectory_msgs::msg::JointTrajectory cmd;
    cmd.joint_names = joints_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_angles.to_vector();
    point.time_from_start = rclcpp::Duration::from_seconds(0.02);

    cmd.points.push_back(point);
    joint_command_publisher_->publish(cmd);

    // publish cmd
    joint_command_publisher_->publish(cmd);
  }

  // subscriber for cartesian point targets (from gait planner)
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_subscriber_;

  // publisher of joint trajectory (to controller)
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
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
