#include "bot_kinematics/ik_solver.hpp"

#include <cmath>
#include <iostream>

namespace bot_kinematics {
IkSolver::IkSolver(const LimbDimensions &dims) : limb_dimensions_(dims) {}

  LimbJointAngles IkSolver::calculate_ik(double target_x, double target_y, double target_z) {
  LimbJointAngles angles;
  const double PI_2 = M_PI / 2.0;

  double l1 = limb_dimensions_.l1;
  double l2 = limb_dimensions_.l2;
  double l3 = limb_dimensions_.l3;

  // calculate haa with atan
  angles.haa = std::atan2(target_y, -target_z);

  // 2. 2D Projection
  double z_hypot = std::sqrt(target_y * target_y + target_z * target_z);
  double z_adj = z_hypot - l1;
  double D = std::sqrt(target_x * target_x + z_adj * z_adj);

  // Clamping
  if (D > (l2 + l3)) D = l2 + l3 - 0.0001;

  // 3. KFE with your 90-degree offset
  double cos_gamma = (l2 * l2 + l3 * l3 - D * D) / (2.0 * l2 * l3);
  double gamma = std::acos(cos_gamma); // Internal angle

  // ADJUSTMENT: Center it so 90 degrees = 0 radians
  angles.kfe = gamma - PI_2;

  // 4. HFE
  double alpha = std::atan2(target_x, z_adj);
  double cos_beta = (l2 * l2 + D * D - l3 * l3) / (2.0 * l2 * D);
  double beta = std::acos(cos_beta);

  // Since 0 is vertical, we just sum the angles relative to the Z-axis
  angles.hfe = alpha + beta;

  angles.hfe = 0;
  angles.kfe = 0;

  std::cout << "IK< ANGLES" << angles.haa << " " << angles.hfe << " " << angles.kfe << std::endl;


  return angles;
}
} // namespace bot_kinematics