#include "bot_math/ik_solver.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace bot_control {
// consts
const std::string LOCOMOTION_COMMANDER_NODE_NAME = "locomotion_commander";

// param names
const std::string LIMB_PREFIXES_PARAM_NAME = "limb_prefixes";
const std::string LIMB_INFO_PARAM_NAME = "limb_info";
const std::string IS_RIGHT_PARAM_NAME = "is_right";
const std::string IS_BACK_PARAM_NAME = "is_back";
const std::string JOINTS_PARAM_NAME = "joints";
const std::string L1_PARAM_NAME = "l1_len";
const std::string L2_PARAM_NAME = "l2_len";
const std::string L3_PARAM_NAME = "l3_len";
const std::string COMMAND_DISPATCH_FREQUENCY_PARAM_NAME =
    "command_dispatch_frequency";

// topic names
const std::string TARGET_TOPIC_NAME =
    "target"; // must match in limb_trajectory_generator.cpp
const std::string JOINT_COMMAND_TOPIC =
    "joint_group_position_controller/commands"; //  comes from
                                                //  bot_control/controllers.yaml

class LocomotionCommander : public rclcpp::Node {
public:
  LocomotionCommander() : Node(LOCOMOTION_COMMANDER_NODE_NAME) {
    bot_math::LimbDimensions dims;

    // declare and get params
    try {
      this->limb_prefixes_ = this->declare_parameter<std::vector<std::string>>(
          LIMB_PREFIXES_PARAM_NAME);
      this->joints_ =
          this->declare_parameter<std::vector<std::string>>(JOINTS_PARAM_NAME);

      // store number of joints and limbs
      this->limbs_count_ = limb_prefixes_.size();
      this->joints_count_ = joints_.size();

      // store limb info
      this->limb_info_.reserve(this->limbs_count_);
      for (int i = 0; i < this->limbs_count_; ++i) {
        limb_info info;
        const std::string &prefix = this->limb_prefixes_[i];
        info.is_right = this->declare_parameter<bool>(
            LIMB_INFO_PARAM_NAME + "." + prefix + "." + IS_RIGHT_PARAM_NAME);
        info.is_back = this->declare_parameter<bool>(
            LIMB_INFO_PARAM_NAME + "." + prefix + "." + IS_BACK_PARAM_NAME);
        this->limb_info_.push_back(info);
      }

      // get limb dims from params
      dims.l1 = this->declare_parameter<double>(L1_PARAM_NAME);
      dims.l2 = this->declare_parameter<double>(L2_PARAM_NAME);
      dims.l3 = this->declare_parameter<double>(L3_PARAM_NAME);

      this->command_dispatch_frequency_ = this->declare_parameter<double>(
          COMMAND_DISPATCH_FREQUENCY_PARAM_NAME);
    } catch (
        const rclcpp::exceptions::UninitializedStaticallyTypedParameterException
            &e) {
      // not able to resolve all params
      RCLCPP_FATAL(this->get_logger(), "MISSING CRITICAL PARAMETERS: %s",
                   e.what());
      throw e; // kill the node
    }

    // init IK solver
    this->ik_solver_ = std::make_unique<bot_math::IkSolver>(dims);

    // TODO should instead be standing pose - this is dangerous
    this->joint_command_.data.assign(this->limbs_count_ * this->joints_count_,
                                     0.0);

    // init subscription to target topics (reciving cartesian points)
    this->create_target_subscribers();

    // init publisher of joint angles (to controller)
    joint_command_publisher_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(
            JOINT_COMMAND_TOPIC, 1);

    // timer that publishes command
    this->command_dispatch_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(
            1000.0 / this->command_dispatch_frequency_),
        [this]() {
          this->joint_command_publisher_->publish(this->joint_command_);
        });

    RCLCPP_INFO(this->get_logger(), "%s node initialized.",
                LOCOMOTION_COMMANDER_NODE_NAME.c_str());
  }

  /**
   * Stores info needed to perform appropriate inversions
   */
  struct limb_info {
    bool is_right;
    bool is_back;
  };

private:
  /**
   * Accepts cartesian coords, runs inverse kinematics, and sends joint angles
   * to controller.
   * */
  void target_callback(int prefix_index,
                       const geometry_msgs::msg::Point::SharedPtr msg) {

    // fetch limb info
    const auto &limb_info = this->limb_info_.at(prefix_index);

    // extract cart coords from msg
    double x =
        limb_info.is_back ? -msg->x : msg->x; // invert x for right-side limbs
    double y = msg->y;
    double z = msg->z;

    // run IK and store calculated limb joint angles
    bot_math::LimbJointAngles joint_angles =
        this->ik_solver_->calculate_ik(x, y, z);

    // update command
    auto vec = joint_angles.to_vector();
    int start_index = prefix_index * this->joints_count_;

    // HAA
    // invert output if back limb
    this->joint_command_.data[start_index] =
        limb_info.is_right ? -vec[0] : vec[0];
    // HFE
    this->joint_command_.data[start_index + 1] = vec[1];
    // KFE
    this->joint_command_.data[start_index + 2] = vec[2];
  }

  /**
   * Creates subscriber for all limbs to topics publishig target points.
   */
  void create_target_subscribers() {
    target_subscribers_.reserve(limb_prefixes_.size());

    for (int i = 0; i < this->limbs_count_; ++i) {
      // concat. target topic name
      std::string topic = limb_prefixes_[i] + "/" + TARGET_TOPIC_NAME;

      auto cb = std::bind(&LocomotionCommander::target_callback, this, i,
                          std::placeholders::_1);
      std::function<void(geometry_msgs::msg::Point::SharedPtr)> typed_cb = cb;
      this->target_subscribers_.push_back(
          this->create_subscription<geometry_msgs::msg::Point>(topic, 1,
                                                               typed_cb));
    }
  }

  // subscriber for cartesian point targets (from gait planner)
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr>
      target_subscribers_;

  // publisher of joint angles (to controller)
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      joint_command_publisher_;

  // vectors with limb prefixes (ie., front-left, back-right, etc.) and limb
  // inversions
  std::vector<std::string> limb_prefixes_;
  std::vector<limb_info> limb_info_;
  int limbs_count_;

  // vector with joint names (ie., shoulder, knee, etc)
  std::vector<std::string> joints_;
  int joints_count_;

  /// shared state command updated  as new targets arrive
  std_msgs::msg::Float64MultiArray joint_command_;

  // instance of ik solver
  std::unique_ptr<bot_math::IkSolver> ik_solver_;

  // heartbeat timer to publish command
  rclcpp::TimerBase::SharedPtr command_dispatch_timer_;
  int command_dispatch_frequency_;
};
} // namespace bot_control

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bot_control::LocomotionCommander>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
