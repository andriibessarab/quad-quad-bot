#ifndef BOT_KINEMATICS_IK_SOLVER_HPP
#define BOT_KINEMATICS_IK_SOLVER_HPP

#include "bot_kinematics/types.hpp"

namespace bot_kinematics {
class IkSolver {
public:
  /**
   * @brief Construct a new Ik Solver object
   * @param dims The physical lengths of the limb segments in meters
   */
  IkSolver(const LimbDimensions &dims);

  /**
   * @brief Calculates inverse kinematics for a single limb.
   * @param target_x Forward/Backward target (m)
   * @param target_y Lateral (Left/Right) target (m)
   * @param target_z Vertical (Up/Down) target (m)
   * @return LimbJointAngles struct containing [haa, hfe, knee] in Radians.
   */
  LimbJointAngles calculate_ik(double target_x, double target_y,
                               double target_z);

private:
  /// Stored physical constants for the limb
  const LimbDimensions limb_dimensions_;
};
} // namespace bot_kinematics

#endif