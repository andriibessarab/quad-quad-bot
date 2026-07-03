#include "bot_math/limb_trajectory.hpp"

#include <cmath>
#include <iostream>

namespace bot_math {

LimbTrajectory::LimbTrajectory(const LimbTrajectoryConfig &config)
    : config_(config) {}

geometry_msgs::msg::Point
LimbTrajectory::compute_target(const LimbTrajectoryInput &input) {
  geometry_msgs::msg::Point target = config_.home_point;
  const double phase_offset = (input.gait_mode == GaitMode::Crawl)
                                  ? config_.crawl_phase_offset
                                  : config_.phase_offset;
  double limb_phase = std::fmod(input.global_phase + phase_offset, 1.0);
  if (limb_phase < 0.0) {
    limb_phase += 1.0;
  }

  switch (input.gait_mode) {
    // --- TROT / CRAWL MODE ----
    // Both are duty-cycle gaits (stance then swing); they only differ in
    // per-limb phase offset (above) and the swing/stance timing supplied by
    // the caller, so they share this branch.
  case GaitMode::Trot:
  case GaitMode::Crawl: {
    // deadband
    if (std::abs(input.step_x) < 1e-4 && std::abs(input.step_y) < 1e-4) {
      return config_.home_point;
    }

    const double x_init = config_.home_point.x;
    const double x_final = x_init + input.step_x;
    const double y_init = config_.home_point.y;
    const double y_final = y_init + input.step_y;
    const double z_floor = config_.home_point.z;
    const double z_peak = z_floor + input.step_height;

    const double duty_factor =
        input.stance_duration / (input.stance_duration + input.swing_duration);

    // --- Stance Phase ---
    if (limb_phase < duty_factor) {
      const double t = limb_phase / duty_factor;
      target.x = lerp(x_final, x_init, t);
      target.y = lerp(y_final, y_init, t);
      target.z = z_floor;
    }
    // --- Swing Phase ----
    else {
      const double t = (limb_phase - duty_factor) / (1 - duty_factor);
      target.x = bezier(x_init, x_init, x_final, x_final, t);
      target.y = bezier(y_init, y_init, y_final, y_final, t);
      target.z = bezier(z_floor, z_peak, z_peak, z_floor, t);
    }
    break;
  }
    // --- STAND MODE ---
  case GaitMode::Stand:
  default: {
    target = config_.home_point;
    break;
  }
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
