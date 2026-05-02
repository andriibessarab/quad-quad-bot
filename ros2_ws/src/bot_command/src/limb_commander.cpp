#include "bot_kinematics/ik_solver.hpp"
#include "bot_kinematics/types.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// param names
const std::string JOINTS_PARAM_NAME = "joints";
const std::string ACTION_SERVER_NAME_PARAM_NAME = "action_server_name";
const std::string TARGET_TOPIC_PARAM_NAME = "target";
const std::string L1_PARAM_NAME = "l1";
const std::string L2_PARAM_NAME = "l2";
const std::string L3_PARAM_NAME = "l3";

// other consts
const std::string TARGET_TOPIC_NAME = "target";

class LimbCommander : public rclcpp::Node {
public:
  // aliases
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory =
      rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  LimbCommander() : Node("limb_controller") {
    RCLCPP_INFO(this->get_logger(), "Limb Commander node has started!");

    // declare and get params
    action_server_name_ =this->declare_parameter<std::string>(ACTION_SERVER_NAME_PARAM_NAME);
    joints_ =this->declare_parameter<std::vector<std::string>>(JOINTS_PARAM_NAME);

    bot_kinematics::LimbDimensions dims;
    dims.l1 = this->declare_parameter<double>(L1_PARAM_NAME);
    dims.l2 = this->declare_parameter<double>(L2_PARAM_NAME);
    dims.l3 = this->declare_parameter<double>(L3_PARAM_NAME);

    // init IK solver
    ik_solver_ = std::make_unique<bot_kinematics::IkSolver>(dims);

    // init action client
    this->action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, action_server_name_);

    // init subscription to target topic
    target_subscriber_ =
        this->create_subscription<geometry_msgs::msg::PointStamped>(
            TARGET_TOPIC_NAME, 1,
            std::bind(&LimbCommander::target_callback, this,
                      std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting for target points...");
  }

private:
  void send_goal() {
    // wait for action server to turn on
    while (!action_client_->wait_for_action_server(
        std::chrono::milliseconds(1000))) {
      if (!rclcpp::ok()) // hande interruption
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for action server.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Action server not available, waiting...");
    }
    RCLCPP_INFO(this->get_logger(), "Action server found!");

    // define goal
    auto goal = FollowJointTrajectory::Goal();

    // set joints for goal
    goal.trajectory.joint_names = this->joints_;

    // sample trajectory point
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0, 0, 0};
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);

    // add points for goal
    goal.trajectory.points.push_back(point);

    // set callback methods in goal options
    auto send_goal_options =
        rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(
        &LimbCommander::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&LimbCommander::feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&LimbCommander::result_callback, this, std::placeholders::_1);

    // Send goal
    RCLCPP_INFO(this->get_logger(), "Sending goal...");
    this->action_client_->async_send_goal(goal, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "Goal sent!");
  }

  void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
                "New target received: x=%.2f, y=%.2f, z=%.2f", msg->point.x,
                msg->point.y, msg->point.z);

    // Run IK
    // auto angles = ik_solver_->calculate_ik(msg->point.x, msg->point.y,
    // msg->point.z);

    // Send the motion goal
    // send_goal(angles);
  }

  void goal_response_callback(
      const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle) {
    // log goal status
    if (goal_handle == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, awaiting result");
    }
  }

  auto feedback_callback(
      GoalHandleFollowJointTrajectory::SharedPtr goal_handle,
      const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
      -> void {
    // log current positions
    if (feedback->actual.positions.size() == joints_.size()) {
      RCLCPP_INFO(this->get_logger(), "Current joint positions: %.2f %.2f %.2f",
                  feedback->actual.positions[0], feedback->actual.positions[1],
                  feedback->actual.positions[2]);
    } else
      RCLCPP_WARN(this->get_logger(),
                  "Received feedback with unexpected joint count!");
  }

  void result_callback(
      const GoalHandleFollowJointTrajectory::WrappedResult &result) {
    // log result status
    switch (result.code) {
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

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr
      action_client_; // action client ptr
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      target_subscriber_;

  // params
  std::string action_server_name_;
  std::vector<std::string> joints_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LimbCommander>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
