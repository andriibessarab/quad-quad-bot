#include "bot_kinematics/ik_solver.hpp"

#include "../include/bot_kinematics/types.hpp"

namespace bot_kinematics {
IkSolver::IkSolver(const LimbDimensions &dims) : limb_dimensions_(dims) {}

LimbJointAngles IkSolver::calculate_ik(double target_x, double target_y,
                                       double target_z) {
  LimbJointAngles angles;

  // TODO
  angles.haa = 0.0;
  angles.hfe = 0.0;
  angles.kfe = 0.0;

  return angles;
}
} // namespace bot_kinematics