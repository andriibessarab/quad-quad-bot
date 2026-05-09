#ifndef BOT_MATH_TYPES_HPP
#define BOT_MATH_TYPES_HPP

#include "geometry_msgs/msg/point.hpp"

#include <string>
#include <vector>

namespace bot_math {
//////////////////////// IK_SOLVER ////////////////////////
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
  }
};

///////////////////// LIMB_TRAJECTORY /////////////////////
/**
 * @brief High-level gait mode.
 */
enum class GaitMode {
  Stand,
  Trot,
};

struct LimbTrajectoryConfig {
  std::string name;    // limb name
  bool is_right;       // true if this limb is on the right side
  bool is_back;        // true if this limb is on the back half
  double phase_offset; // phase shift for this limb in the gait cycle
  geometry_msgs::msg::Point home_point;
};

struct LimbTrajectoryInput {
  GaitMode gait_mode = GaitMode::Stand; // current gate mode
  double global_phase = 0.0;            // global phase of gate (w/ no offset)
  double step_len = 0.0;                // forward step length for walking
  double step_height = 0.0;             // swing foot lift height
  double swing_duration = 0.0;          // duration of swing phase
  double stance_duration = 0.0;         // duration of stance phase
};
} // namespace bot_math

#endif
