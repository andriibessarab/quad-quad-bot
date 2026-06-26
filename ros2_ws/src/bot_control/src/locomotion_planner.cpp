#include "bot_math/limb_trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace bot_control {
// param names
const std::string CONTROL_LOOP_FREQUENCY_PARAM_NAME("control_loop_frequency");
const std::string LIMB_PREFIXES_PARAM_NAME("limb_prefixes");
const std::string HOME_X_PARAM_NAME("home_x");
const std::string HOME_Y_PARAM_NAME("home_y");
const std::string HOME_Z_PARAM_NAME("home_z");
const std::string STEP_HEIGHT_PARAM_NAME("step_height");
const std::string LIMB_INFO_PARAM_NAME("limb_info");
const std::string IS_RIGHT_PARAM_NAME("is_right");
const std::string IS_BACK_PARAM_NAME("is_back");
const std::string HIP_X_OFFSET_PARAM_NAME("hip_x_offset");
const std::string HIP_Y_OFFSET_PARAM_NAME("hip_y_offset");
const std::string SWING_DURATION_PARAM_NAME("swing_duration");
const std::string STANCE_DURATION_PARAM_NAME("stance_duration");

// topic names
const std::string CMD_VEL_TOPIC_NAME("/cmd_vel");

// other consts
const std::string GAIT_PLANNER_NODE_NAME("gait_planner");
const std::string
    TARGET_TOPIC_NAME("target"); // must match in limb_commander.cpp

class LimbTrajectoryGenerator : public rclcpp::Node {
public:
  LimbTrajectoryGenerator()
      : Node(GAIT_PLANNER_NODE_NAME), gait_state_(bot_math::GaitMode::Trot) {
    // get params
    try {
      control_loop_frequency_ =
          this->declare_parameter<double>(CONTROL_LOOP_FREQUENCY_PARAM_NAME);
      limb_prefixes_ = this->declare_parameter<std::vector<std::string>>(
          LIMB_PREFIXES_PARAM_NAME);

      // home point
      home_point_.x = this->declare_parameter<double>(HOME_X_PARAM_NAME);
      home_point_.y = this->declare_parameter<double>(HOME_Y_PARAM_NAME);
      home_point_.z = this->declare_parameter<double>(HOME_Z_PARAM_NAME);

      // trot gait parameters
      step_height_ = this->declare_parameter<double>(STEP_HEIGHT_PARAM_NAME);
      swing_duration_ =
          this->declare_parameter<double>(SWING_DURATION_PARAM_NAME);
      stance_duration_ =
          this->declare_parameter<double>(STANCE_DURATION_PARAM_NAME);

      // symmetric hip offsets — signs derived per limb from is_right/is_back
      double hip_x_offset =
          this->declare_parameter<double>(HIP_X_OFFSET_PARAM_NAME);
      double hip_y_offset =
          this->declare_parameter<double>(HIP_Y_OFFSET_PARAM_NAME);

      hip_x_.reserve(limb_prefixes_.size());
      hip_y_.reserve(limb_prefixes_.size());
      for (const auto &prefix : limb_prefixes_) {
        bool is_back = this->declare_parameter<bool>(
            LIMB_INFO_PARAM_NAME + "." + prefix + "." + IS_BACK_PARAM_NAME);
        bool is_right = this->declare_parameter<bool>(
            LIMB_INFO_PARAM_NAME + "." + prefix + "." + IS_RIGHT_PARAM_NAME);
        // front limbs are +x, back are -x; left limbs are +y, right are -y
        hip_x_.push_back(is_back ? -hip_x_offset : +hip_x_offset);
        hip_y_.push_back(is_right ? -hip_y_offset : +hip_y_offset);
      }

    } catch (
        const rclcpp::exceptions::UninitializedStaticallyTypedParameterException
            &e) {
      // not able to resolve all params
      RCLCPP_FATAL(this->get_logger(), "MISSING CRITICAL PARAMETERS: %s",
                   e.what());
      throw e; // kill the node
    }

    // init math helpers
    create_limb_trajectories();
    gait_start_time_ = this->now();

    // cmd_vel_ is zero-initialized — robot holds stance until a command arrives
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        CMD_VEL_TOPIC_NAME, 1,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          cmd_vel_ = *msg;
        });

    // setup target publishers
    this->create_target_publishers();

    // run control loop at set hz
    this->timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(1000 /
                                                  control_loop_frequency_),
        std::bind(&LimbTrajectoryGenerator::control_loop, this));
  }

private:
  void control_loop() {
    const double elapsed = (this->now() - gait_start_time_).seconds();
    const double cycle_duration = swing_duration_ + stance_duration_;
    double global_phase = std::fmod(elapsed / cycle_duration, 1.0);
    if (global_phase < 0.0) {
      global_phase += 1.0;
    }

    for (size_t i = 0; i < limb_trajectories_.size(); ++i) {
      bot_math::LimbTrajectoryInput input;
      input.gait_mode = gait_state_;
      input.global_phase = global_phase;
      input.step_height = step_height_;
      input.swing_duration = swing_duration_;
      input.stance_duration = stance_duration_;

      // decompose body twist into per-limb foot displacement over one gait
      // cycle
      input.step_x =
          (cmd_vel_.linear.x - cmd_vel_.angular.z * hip_y_[i]) * cycle_duration;
      input.step_y =
          (cmd_vel_.linear.y + cmd_vel_.angular.z * hip_x_[i]) * cycle_duration;

      target_publishers_[i]->publish(
          limb_trajectories_[i].compute_target(input));
    }
  }

  /**
   * Creates trajectory objects for all limbs.
   */
  void create_limb_trajectories() {
    limb_trajectories_.reserve(limb_prefixes_.size());
    for (size_t i = 0; i < limb_prefixes_.size(); ++i) {
      const auto &prefix = limb_prefixes_[i];
      bot_math::LimbTrajectoryConfig config;
      config.name = prefix;
      config.home_point = home_point_;

      // trot diagonal pairs: fl+br lead at phase 0, fr+bl follow at phase 0.5
      config.phase_offset = (prefix == "fl" || prefix == "br") ? 0.0 : 0.5;

      limb_trajectories_.emplace_back(config);
    }
  }

  /**
   * Creates publishers for all limb target topics.
   */
  void create_target_publishers() {
    target_publishers_.reserve(limb_prefixes_.size());
    for (const auto &prefix : limb_prefixes_) {
      target_publishers_.push_back(
          this->create_publisher<geometry_msgs::msg::Point>(
              prefix + "/" + TARGET_TOPIC_NAME, 1));
    }
  }

  // instance vars
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Time gait_start_time_;
  bot_math::GaitMode gait_state_;
  std::vector<bot_math::LimbTrajectory> limb_trajectories_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr>
      target_publishers_;
  geometry_msgs::msg::Twist cmd_vel_;

  // params
  double control_loop_frequency_;
  std::vector<std::string> limb_prefixes_;
  std::vector<double> hip_x_;
  std::vector<double> hip_y_;
  geometry_msgs::msg::Point home_point_;
  double step_height_;
  double swing_duration_;
  double stance_duration_;
};
} // namespace bot_control

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bot_control::LimbTrajectoryGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
