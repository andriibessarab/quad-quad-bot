#ifndef BOT_KINEMATICS_TYPES_HPP
#define BOT_KINEMATICS_TYPES_HPP

#include <vector>

namespace bot_kinematics {

/**
 * @brief Physical dimensions of a single quadruped limb.
 * All measurements are in meters (m).
 */
struct LimbDimensions {
  double l1; // Hip offset
  double l2; // Femur length
  double l3; // Tibia length
};

/**
 * @brief Standard container for quadruped limb joint angles.
 * Units: Radians
 */
struct LimbJointAngles {
  double haa;
  double hfe;
  double kfe;

  std::vector<double> to_vector() const {
    return std::vector<double>{haa, hfe, kfe};
    ;
  }
};
} // namespace bot_kinematics

#endif