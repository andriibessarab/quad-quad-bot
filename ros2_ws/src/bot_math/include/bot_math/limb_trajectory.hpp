#ifndef BOT_MATH_LIMB_TRAJECTORY_HPP
#define BOT_MATH_LIMB_TRAJECTORY_HPP

#include "bot_math/types.hpp"
#include <string>

namespace bot_math {

class LimbTrajectory {
public:
  LimbTrajectory(const LimbTrajectoryConfig &config);

  geometry_msgs::msg::Point compute_target(const LimbTrajectoryInput &input);

private:
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
  double bezier(double p0, double p1, double p2, double p3, double t);

  double lerp(double start, double end, double t);

  LimbTrajectoryConfig config_;
};

} // namespace bot_math

#endif
