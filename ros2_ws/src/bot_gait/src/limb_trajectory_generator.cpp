#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace bot_gait {
// param names
const std::string CONTROL_LOOP_FREQUENCY_PARAM_NAME("control_loop_frequency");
const std::string SWING_DURATION_PARAM_NAME("swing_duration");
const std::string STANCE_DURATION_PARAM_NAME("stance_duration");

// other consts
const std::string
    TARGET_TOPIC_NAME("target"); // must match in limb_commander.cpp

class LimbTrajectoryGenerator : public rclcpp::Node {
public:
  LimbTrajectoryGenerator() : Node("limb_trajectory_generator") {
    // get params
    try {
      control_loop_frequency_ =
          this->declare_parameter<double>(CONTROL_LOOP_FREQUENCY_PARAM_NAME);
      swing_duration_ =
          this->declare_parameter<double>(SWING_DURATION_PARAM_NAME);
      stance_duration_ =
          this->declare_parameter<double>(STANCE_DURATION_PARAM_NAME);
    } catch (
        const rclcpp::exceptions::UninitializedStaticallyTypedParameterException
            &e) {
      // not able to resolve all params
      RCLCPP_FATAL(this->get_logger(), "MISSING CRITICAL PARAMETERS: %s",
                   e.what());
      throw e; // kill the node
    }

    /////////temp//////////
    this->phase_start_time_ = this->now(); // start phaze on launch
    this->leg_state_ = SWING;
    ///////////////////////

    // run control loop at set hz
    this->timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(1000 /
                                                  control_loop_frequency_),
        std::bind(&LimbTrajectoryGenerator::control_loop, this));
  }

private:
  void control_loop() {
    switch (this->leg_state_) {
    case SWING: // run bezier curve math
      break;

    case STANCE: // run lerp math
      break;

    case STAND: // return default standing position
    default:
      break;
    }
  }

  double bezier(double p0, double p1, double p2, double p3, double t) {
    return 0.0;
  }

  double lerp(double start, double end, double t) { return 0.0; }

  // instance vars
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time phase_start_time_;
  enum LegState { STAND, SWING, STANCE } leg_state_ = STAND;

  // params
  double control_loop_frequency_;
  double swing_duration_;
  double stance_duration_;
  std::string target_topic_;
};
} // namespace bot_gait

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bot_gait::LimbTrajectoryGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}