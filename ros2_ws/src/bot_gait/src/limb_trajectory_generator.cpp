#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/publisher.hpp"
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

    // setup target publisher
    target_publisher_ =
        this->create_publisher<geometry_msgs::msg::Point>(TARGET_TOPIC_NAME, 1);

    /////////temp//////////
    this->phase_start_time_ = this->now(); // start phaze on launch
    this->leg_state_ = STAND;
    ///////////////////////

    // run control loop at set hz
    this->timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(1000 /
                                                  control_loop_frequency_),
        std::bind(&LimbTrajectoryGenerator::control_loop, this));
  }

private:
  void control_loop() {
    // leg standing - return home coordinate
    if (leg_state_ == STAND) {
      geometry_msgs::msg::Point msg;
      msg.x = -0.012;
      msg.y = 0.02088;
      msg.z = -0.078;

      target_publisher_->publish(msg);

      double elapsed = (this->now() - this->phase_start_time_).seconds();
      if (elapsed >= 5.0) {
        this->leg_state_ = SWING;
        this->phase_start_time_ = this->now();
      }
      return;
    }

    // calculate time elapsed since phase start
    double elapsed = (this->now() - this->phase_start_time_).seconds();

    geometry_msgs::msg::Point msg;

    // find cartesian point at specific phase on appropriate curve for specified
    // state
    switch (this->leg_state_) {
    case SWING: {
      // run bezier curve math
      double t = elapsed / this->swing_duration_;
      if (t >= 1.0) {
        msg.x = 0.024;
        msg.y = 0.02088;
        msg.z = -0.078;
        target_publisher_->publish(msg);

        this->leg_state_ = STANCE;
        this->phase_start_time_ = this->now();

        return;
      }

      // calculate cooridnates
      msg.x = bezier(-0.012, -0.012, 0.024, 0.024, t);
      msg.y = 0.02088;
      msg.z = bezier(-0.078, -0.046, -0.046, -0.078, t);

      break;
    }
    case STANCE: {
      double t = elapsed / this->stance_duration_;
      if (t >= 1.0) {
        msg.x = -0.012;
        msg.y = 0.02088;
        msg.z = -0.078;
        target_publisher_->publish(msg);

        this->leg_state_ = SWING;
        this->phase_start_time_ = this->now();
        return;
      }

      double s = t * t * (3.0 - 2.0 * t); // smoothstep

      msg.x = lerp(0.024, -0.012, s);
      msg.y = 0.02088;
      msg.z = -0.078;
      break;
    }

    default:
      break;
    }

    // publish calculated point
    target_publisher_->publish(msg);
  }

  /**
   * @brief Evaluates a 1D cubic Bezier curve for swing-phase trajectory
   * shaping.
   *
   * @param p0 Curve value at the start of swing.
   * @param p1 First intermediate control value that shapes lift-off.
   * @param p2 Second intermediate control value that shapes touch-down.
   * @param p3 Curve value at the end of swing.
   * @param t Normalized swing phase in the range [0, 1].
   * @return Interpolated curve value at phase `t`.
   */
  double bezier(double p0, double p1, double p2, double p3, double t) {
    double inv_t = 1.0 - t;
    return (inv_t * inv_t * inv_t * p0) + (3.0 * inv_t * inv_t * t * p1) +
           (3.0 * inv_t * t * t * p2) + (t * t * t * p3);
  }

  double lerp(double start, double end, double t) {
    return start + t * (end - start);
  }

  // instance vars
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time phase_start_time_;
  enum LegState { STAND, SWING, STANCE } leg_state_ = STAND;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_publisher_;

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
