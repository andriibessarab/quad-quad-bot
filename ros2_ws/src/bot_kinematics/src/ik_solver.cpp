#include "bot_kinematics/ik_solver.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cmath>
#include <iostream>
#include <numbers>

#define _USE_MATH_DEFINES

namespace bot_kinematics {
std::string LOGGER_NAME = "ik_solver";

IkSolver::IkSolver(const LimbDimensions &dims) : limb_dimensions_(dims) {}

LimbJointAngles IkSolver::calculate_ik(double target_x, double target_y,
                                       double target_z) {
  LimbJointAngles angles;

  double l1 = limb_dimensions_.l1;
  double l2 = limb_dimensions_.l2;
  double l3 = limb_dimensions_.l3;

  /////////////////////////// CALCULATIONS ///////////////////////////
  // Refer to docs for calculation and geometrical representation
  //
  // Assumes [haa, hfe, kfe] = (0, 0, 0) woth shoulder parallel to body and limb
  // in vertical "I" position
  ////////////////////////////////////////////////////////////////////

  // HAA calculation
  target_z *= -1;
  double alpha = std::atan2(target_y, target_z);
  double A = std::sqrt(target_y * target_y + target_z * target_z);
  if (A < l1) { // handle inside-radius targets
    A = std::max(A, l1);
    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                "recieved target within forbidden inner radius - clamping the "
                "boundary");
  }
  double H = std::sqrt(A * A - l1 * l1);
  double beta = std::atan2(H, l1);
  angles.haa = alpha + beta - M_PI_2;

  // HFE & KFE calculation
  double B = std::sqrt(target_x * target_x + H * H);
  if (B < 1e-9) { // happens when target is right under hfe
    B = 1e-9;
    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                "received target at planar hip singularity - clamping B away "
                "from zero");
  }
  if (B > l2 + l3) { // target is further away then femur + tibia
    B = l2 + l3;
    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                "received target at/beyond planar outer reach - saturating to "
                "boundary pose");
  } else if (B < std::abs(l2 - l3)) { // target is too close to hfe
    B = std::abs(l2 - l3);
    RCLCPP_WARN(
        rclcpp::get_logger(LOGGER_NAME),
        "received target at/within planar inner reach limit - saturating "
        "to boundary pose");
  }
  double rho = std::atan2(target_x, H);
  double phi_arg = (l3 * l3 - l2 * l2 - B * B) / (-2 * l2 * B);
  phi_arg = std::clamp(phi_arg, -1.0, 1.0);
  double phi = std::acos(phi_arg);
  angles.hfe = rho - phi;
  double kfe_arg = (B * B - l2 * l2 - l3 * l3) / (-2 * l2 * l3);
  kfe_arg = std::clamp(kfe_arg, -1.0, 1.0);
  angles.kfe = std::acos(kfe_arg) - M_PI;

  return angles;
}
} // namespace bot_kinematics
