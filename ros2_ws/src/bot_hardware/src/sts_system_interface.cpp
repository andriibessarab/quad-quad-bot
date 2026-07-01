#include "bot_hardware/sts_system_interface.hpp"

#include <algorithm>
#include <cstddef>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace bot_hardware {

// Init in-memory setup
hardware_interface::CallbackReturn StsSystemInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
  // generic setup
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // resize (and initialize) vectors to hold motor info
  const std::size_t n = info_.joints.size();
  motor_ids_.resize(n);
  directions_.resize(n);
  offset_ticks_.resize(n);
  hw_positions_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);
  hw_efforts_.assign(n, 0.0);
  hw_commands_position_.assign(n, 0.0);
  last_cmd_positions_.assign(n, 0.0);

  // Fetch all settings from XACRO
  try {
    // bus-wide settings injected by ros2_control.urdf.xacro into <hardware>.
    serial_port_ = info_.hardware_parameters.at("serial_port");
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
    max_velocity_rad_per_s_ =
        std::stod(info_.hardware_parameters.at("max_velocity_rad_per_s"));
    servo_speed_units_ =
        std::stoi(info_.hardware_parameters.at("servo_speed_units"));

    // per-joint settings, injected by ros2_control.urdf.xacro into each <joint>
    // block.
    for (std::size_t i = 0; i < n; ++i) {
      const auto &p = info_.joints[i].parameters;
      motor_ids_[i] = static_cast<uint8_t>(std::stoi(p.at("motor_id")));
      directions_[i] = std::stod(p.at("direction"));
      offset_ticks_[i] = std::stoi(p.at("offset_ticks"));
    }
  } catch (const std::out_of_range &e) {
    RCLCPP_ERROR(get_logger(), "Missing required hardware/joint param %s",
                 e.what());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (const std::invalid_argument &e) {
    RCLCPP_ERROR(get_logger(), "Malformed number: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check that every joint exposes exactly one and exactly position command
  // interface
  for (const auto &joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
      RCLCPP_ERROR(
          get_logger(),
          "Joint '%s' must have exactly one 'position' command_interface",
          joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(get_logger(), "Initialised %zu STS joints (%s @ %d baud)", n,
              serial_port_.c_str(), baud_rate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Init hardware contact - open serial, ping servos, and verify bus is alive
hardware_interface::CallbackReturn StsSystemInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // connect to serial port
  if (!servo_.begin(baud_rate_, serial_port_.c_str())) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port %s at %d baud",
                 serial_port_.c_str(), baud_rate_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // make sure can ping all motors
  bool all_ok = true;
  for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
    if (servo_.Ping(motor_ids_[i]) == -1) {
      RCLCPP_ERROR(get_logger(), "Servo ID %d (%s) did not respond to Ping()",
                   motor_ids_[i], info_.joints[i].name.c_str());
      all_ok = false;
    }
  }
  if (!all_ok) {
    servo_.end();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "All %zu servos responded", motor_ids_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Enable all motor torques (hold position) and set current positions as goals
// to avoid snapping on first write
hardware_interface::CallbackReturn StsSystemInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
    // enable motor torque (i.e. hold position)
    if (servo_.EnableTorque(motor_ids_[i], 1) == 0) { // == 0 indicates failure
      RCLCPP_ERROR(get_logger(), "Failed to enable torque on servo ID %d (%s)",
                   motor_ids_[i], info_.joints[i].name.c_str());
      // disable all motor torques since there was an issue
      for (std::size_t j = 0; j < i; ++j) {
        servo_.EnableTorque(motor_ids_[j], 0);
      }
      return hardware_interface::CallbackReturn::ERROR;
    }

    // read servo's feedback into internal buffer
    if (servo_.FeedBack(motor_ids_[i]) == 0) { // == 0 indicates failure
      RCLCPP_ERROR(get_logger(),
                   "Failed to read initial position for servo ID %d (%s)",
                   motor_ids_[i], info_.joints[i].name.c_str());
      // disable all motor torques since there was an issue
      for (std::size_t j = 0; j <= i; ++j) {
        servo_.EnableTorque(motor_ids_[j], 0);
      }
      return hardware_interface::CallbackReturn::ERROR;
    }

    // seed command with CURRENT pose so the joints don't snap to zero on the
    // first write() cycle.
    const double pos = ticks_to_rad(servo_.ReadPos(-1), i);
    hw_positions_[i] = pos;
    hw_commands_position_[i] = pos;
    last_cmd_positions_[i] = pos;
  }

  RCLCPP_INFO(get_logger(), "Torque enabled; commands seeded to current pose");
  return hardware_interface::CallbackReturn::SUCCESS;
}

} // namespace bot_hardware

PLUGINLIB_EXPORT_CLASS(bot_hardware::StsSystemInterface,
                       hardware_interface::SystemInterface);
