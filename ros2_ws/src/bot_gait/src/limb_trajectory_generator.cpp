#include "bot_math/limb_trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"
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

namespace bot_gait {
// param names
const std::string CONTROL_LOOP_FREQUENCY_PARAM_NAME("control_loop_frequency");
const std::string LIMB_PREFIXES_PARAM_NAME("limb_prefixes");
const std::string HOME_X_PARAM_NAME("home_x");
const std::string HOME_Y_PARAM_NAME("home_y");
const std::string HOME_Z_PARAM_NAME("home_z");

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
    bot_math::LimbTrajectoryInput input;
    input.gait_mode = gait_state_;
    input.step_len = 0.025;
    input.step_height = 0.025;
    input.swing_duration = 0.4;
    input.stance_duration = 0.75;

    const double elapsed = (this->now() - gait_start_time_).seconds();
    const double gait_cycle_duration =
        input.swing_duration + input.stance_duration;
    input.global_phase = std::fmod(elapsed / gait_cycle_duration, 1.0);
    if (input.global_phase < 0.0) {
      input.global_phase += 1.0;
    }

    for (size_t i = 0; i < limb_trajectories_.size(); ++i) {
      geometry_msgs::msg::Point target =
          limb_trajectories_[i].compute_target(input);
      target_publishers_[i]->publish(target);
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

      //// TODO TEMP ////
      if (prefix == "fl" || prefix == "br") {
        config.phase_offset = 0.0;
      } else {
        config.phase_offset = 0.5;
      }

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
  rclcpp::Time gait_start_time_;
  bot_math::GaitMode gait_state_;
  std::vector<bot_math::LimbTrajectory> limb_trajectories_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr>
      target_publishers_;

  // params
  double control_loop_frequency_;
  std::vector<std::string> limb_prefixes_;
  geometry_msgs::msg::Point home_point_;
};
} // namespace bot_gait

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bot_gait::LimbTrajectoryGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
