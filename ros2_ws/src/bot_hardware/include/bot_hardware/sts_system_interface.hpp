#ifndef BOT_HARDWARE__STS_SYSTEM_INTERFACE_HPP_
#define BOT_HARDWARE__STS_SYSTEM_INTERFACE_HPP_

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <scservo/SCServo.h> // vendored fork

namespace bot_hardware {

class StsSystemInterface : public hardware_interface::SystemInterface {
public:
  // System Interface overridden methods
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
      override;
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // serial bus + SDK
  SMS_STS servo_; // can access all servos since one bus
  std::string serial_port_;
  int baud_rate_;

  // global limits
  double max_velocity_rad_per_s_;
  int servo_speed_units_;
  int servo_acc_units_;

  // per-joint wiring
  std::vector<uint8_t> motor_ids_; // 0 - 253
  std::vector<double> directions_; // +1.0 / -1.0
  std::vector<int> offset_ticks_;  // tick value at URDF zero
  std::vector<int> lower_ticks_;   // physical min tick
  std::vector<int> upper_ticks_;   // physical max tick

  // ros2_control state / command storage
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_position_;
  std::vector<double> last_cmd_positions_; // for the software velocity clamp

  // unit conversion
  static constexpr double kTicksPerRev = 4096.0;
  static constexpr double kRadPerTick = 2.0 * M_PI / kTicksPerRev;
  int rad_to_ticks(double rad, std::size_t i) const;
  double ticks_to_rad(int ticks, std::size_t i) const;
};

} // namespace bot_hardware

#endif // BOT_HARDWARE__STS_SYSTEM_INTERFACE_HPP_
