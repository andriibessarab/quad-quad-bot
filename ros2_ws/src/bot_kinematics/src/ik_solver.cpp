#include "bot_kinematics/ik_solver.hpp"

#include <cmath>
#include <iostream>
#include <numbers>

#define _USE_MATH_DEFINES

namespace bot_kinematics {
IkSolver::IkSolver(const LimbDimensions &dims) : limb_dimensions_(dims) {}

  LimbJointAngles IkSolver::calculate_ik(double target_x, double target_y, double target_z) {
  LimbJointAngles angles;

  double l1 = limb_dimensions_.l1;
  double l2 = limb_dimensions_.l2;
  double l3 = limb_dimensions_.l3;

  // use derived formulas to calculate the three angles
  // angles.haa = -(std::atan2(std::sqrt(target_y * target_y + target_z * target_z - l1 * l1), l1) + std::atan2(target_y, -target_z) - M_PI / 2);
  // angles.kfe = std::acos((target_y * target_y + target_z * target_z - l1 * l1 + target_x * target_x - l2 * l2 - l3 * l3) / (-2 * l2 * l3));
  // angles.hfe = std::asin((l3 * std::sin(angles.kfe)) / (std::sqrt(target_y * target_y + target_z * target_z - l1 * l1 + target_x * target_x))) + std::atan2(target_x, std::sqrt(target_y * target_y + target_z * target_z - l1 * l1));

  /////////////////////////// CALCULATIONS ///////////////////////////
  // planar distances
  double d_yz_sq = target_y * target_y + target_z * target_z - l1 * l1;
  if (d_yz_sq < 0) d_yz_sq = 0; // prevent NaN
  double d_yz = std::sqrt(d_yz_sq);  // B term from calulations

  double d_sq = d_yz_sq + target_x * target_x;
  double d = std::sqrt(d_sq);

  // HAA
  angles.haa = -(std::atan2(d_yz, l1) + std::atan2(target_y, -target_z) - M_PI_2);

  // raw angles
  double cos_gamma = (l2 * l2 + l3 * l3 - d_sq) / (2 * l2 * l3);
  cos_gamma = std::max(-1.0, std::min(1.0, cos_gamma));
  double gamma = std::acos(cos_gamma); // inner knee angle

  // alpha = angle of femur forward of the straight line to the foot
  double cos_alpha = (l2 * l2 + d_sq - l3 * l3) / (2 * l2 * d);
  cos_alpha = std::max(-1.0, std::min(1.0, cos_alpha));
  double alpha = std::acos(cos_alpha);

  // theta = angle of the foot forward of straight down
  double theta = std::atan2(target_x, d_yz);

  // KFE
  // Bending fwd/up (straightening) is +ve. Zero is L-shape (PI/2).
  angles.kfe = gamma - M_PI_2;

  // HFE
  // 0 rad means  femur points horizontally forward. Bending down is -ve
  angles.hfe = (M_PI / 2) - (alpha + theta);

  return angles;
}
} // namespace bot_kinematics