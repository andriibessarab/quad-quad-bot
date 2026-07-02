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
  lower_ticks_.resize(n);
  upper_ticks_.resize(n);
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
    servo_acc_units_ =
        std::stoi(info_.hardware_parameters.at("servo_acc_units"));

    // per-joint settings, injected by ros2_control.urdf.xacro into each <joint>
    // block.
    for (std::size_t i = 0; i < n; ++i) {
      const auto &p = info_.joints[i].parameters;
      motor_ids_[i] = static_cast<uint8_t>(std::stoi(p.at("motor_id")));
      directions_[i] = std::stod(p.at("direction"));
      offset_ticks_[i] = std::stoi(p.at("offset_ticks"));
      lower_ticks_[i] = std::stoi(p.at("lower_ticks"));
      upper_ticks_[i] = std::stoi(p.at("upper_ticks"));
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

  // burn EEPROM limits (only when the stored value differs due to finite
  // write-cycle lifetime)
  for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
    if (lower_ticks_[i] == -1 || upper_ticks_[i] == -1)
      continue;
    const int cur_lo =
        servo_.readWord(motor_ids_[i], SMS_STS_MIN_ANGLE_LIMIT_L);
    const int cur_hi =
        servo_.readWord(motor_ids_[i], SMS_STS_MAX_ANGLE_LIMIT_L);
    if (cur_lo == lower_ticks_[i] && cur_hi == upper_ticks_[i])
      continue;
    servo_.unLockEeprom(motor_ids_[i]);
    servo_.writeWord(motor_ids_[i], SMS_STS_MIN_ANGLE_LIMIT_L,
                     static_cast<uint16_t>(lower_ticks_[i]));
    servo_.writeWord(motor_ids_[i], SMS_STS_MAX_ANGLE_LIMIT_L,
                     static_cast<uint16_t>(upper_ticks_[i]));
    servo_.LockEeprom(motor_ids_[i]);
    RCLCPP_INFO(get_logger(),
                "Servo ID %d (%s): updated EEPROM limits [%d, %d]",
                motor_ids_[i], info_.joints[i].name.c_str(), lower_ticks_[i],
                upper_ticks_[i]);
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

// On deactivation, turn off motor torques
hardware_interface::CallbackReturn StsSystemInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
    if (servo_.EnableTorque(motor_ids_[i], 0) ==
        0) { // == 0 means couldn't turn off
      RCLCPP_ERROR(get_logger(), "Failed to disable torque on servo ID %d (%s)",
                   motor_ids_[i], info_.joints[i].name.c_str());
    }
  }
  RCLCPP_INFO(get_logger(), "Torque disabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Export used by controller of all measurable values offered
std::vector<hardware_interface::StateInterface>
StsSystemInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> ifaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    ifaces.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    ifaces.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
                        &hw_efforts_[i]);
  }
  return ifaces;
}

// Export used by controller that it is allowed to write
std::vector<hardware_interface::CommandInterface>
StsSystemInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    ifaces.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_POSITION,
                        &hw_commands_position_[i]);
  }
  return ifaces;
}

// Update feedback
hardware_interface::return_type
StsSystemInterface::read(const rclcpp::Time & /*time*/,
                         const rclcpp::Duration & /*period*/) {
  for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
    // One FeedBack() fetches the whole status block; Read*(-1) then parses
    // from that buffer with no extra bus traffic
    if (servo_.FeedBack(motor_ids_[i]) != 0) {
      hw_positions_[i] = ticks_to_rad(servo_.ReadPos(-1), i);
      hw_velocities_[i] = directions_[i] * servo_.ReadSpeed(-1) * kRadPerTick;
      hw_efforts_[i] = static_cast<double>(servo_.ReadLoad(-1));

    } else { // == 0 means could not read feedback

      RCLCPP_WARN(get_logger(),
                  "FeedBack failed for servo ID %d (%s); keeping last value",
                  motor_ids_[i], info_.joints[i].name.c_str());
    }
  }
  return hardware_interface::return_type::OK;
}

// Move the servos
hardware_interface::return_type
StsSystemInterface::write(const rclcpp::Time & /*time*/,
                          const rclcpp::Duration &period) {
  const std::size_t n = motor_ids_.size();
  const double max_delta = max_velocity_rad_per_s_ *
                           period.seconds(); // max velocity based on limits

  std::vector<uint8_t> ids(n);
  std::vector<int16_t> positions(n);
  std::vector<uint16_t> speeds(n, static_cast<uint16_t>(servo_speed_units_));
  std::vector<uint8_t> accs(n, static_cast<uint8_t>(servo_acc_units_));

  for (std::size_t i = 0; i < n; ++i) {
    // clamp to calibrated physical limits (tick bounds converted to rad)
    double target = hw_commands_position_[i];

    // which of the two is lower and upper rad bound depends on direction; hence
    // using min and max to find out
    const double rad_a = ticks_to_rad(lower_ticks_[i], i);
    const double rad_b = ticks_to_rad(upper_ticks_[i], i);
    target = std::clamp(target, std::min(rad_a, rad_b), std::max(rad_a, rad_b));

    // software velocity clamp: never step more than max_delta in a cycle.
    const double delta =
        std::clamp(target - last_cmd_positions_[i], -max_delta, max_delta);
    const double next = last_cmd_positions_[i] + delta;
    last_cmd_positions_[i] = next;

    // record clamped position
    ids[i] = motor_ids_[i];
    positions[i] = static_cast<int16_t>(rad_to_ticks(next, i));
  }

  // one synchronized transaction drives all servos in a single bus write
  servo_.SyncWritePosEx(ids.data(), static_cast<uint8_t>(n), positions.data(),
                        speeds.data(), accs.data());

  return hardware_interface::return_type::OK;
}

// converts joint position in radians to raw servo ticks, applying per-joint
// direction and zero offset.
int StsSystemInterface::rad_to_ticks(double rad, std::size_t i) const {
  const int ticks =
      static_cast<int>(std::lround(directions_[i] * rad / kRadPerTick)) +
      offset_ticks_[i];
  return std::clamp(ticks, 0, 4095);
}

// converts raw servo ticks to joint position in radians, applying per-joint
// direction and zero offset.
double StsSystemInterface::ticks_to_rad(int ticks, std::size_t i) const {
  return directions_[i] * (ticks - offset_ticks_[i]) * kRadPerTick;
}

} // namespace bot_hardware

PLUGINLIB_EXPORT_CLASS(bot_hardware::StsSystemInterface,
                       hardware_interface::SystemInterface);
