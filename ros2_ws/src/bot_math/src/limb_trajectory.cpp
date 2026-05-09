#include "bot_math/limb_trajectory.hpp"

#include <iostream>

namespace bot_math {

LimbTrajectory::LimbTrajectory(const LimbTrajectoryConfig &config)
    : config_(config) {}

geometry_msgs::msg::Point
LimbTrajectory::compute_target(const LimbTrajectoryInput &input) {
  geometry_msgs::msg::Point target;
  double limb_phase = input.global_phase + config_.phase_offset;

  switch (input.gait_mode) {
  case GaitMode::Stand:
  default:
    target = config_.home_point;
    break;
  }

  return target;
}

double LimbTrajectory::bezier(double p0, double p1, double p2, double p3,
                              double t) {
  double inv_t = 1.0 - t;
  return (inv_t * inv_t * inv_t * p0) + (3.0 * inv_t * inv_t * t * p1) +
         (3.0 * inv_t * t * t * p2) + (t * t * t * p3);
}

double LimbTrajectory::lerp(double start, double end, double t) {
  return start + t * (end - start);
}

} // namespace bot_math
