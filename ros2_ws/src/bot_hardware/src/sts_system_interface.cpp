#include "bot_hardware/sts_system_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

#include "ftservo_linux/SMS_STS.h"
#include "yaml-cpp/yaml.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace bot_hardware {

static const char * LOGGER = "StsSystemInterface";

// ============================================================================
// Constructor / Destructor
// ============================================================================

StsSystemInterface::StsSystemInterface()
: driver_(std::make_unique<SMS_STS>()), estop_active_(false) {}

StsSystemInterface::~StsSystemInterface()
{
  if (estop_executor_) {
    estop_executor_->cancel();
  }
  if (estop_thread_.joinable()) {
    estop_thread_.join();
  }
}

// ============================================================================
// on_init
// ============================================================================

hardware_interface::CallbackReturn StsSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Load motor_params.yaml ---
  // The path is injected as a single <param> in the URDF when sim:=false.
  std::string yaml_path;
  try {
    yaml_path = info_.hardware_parameters.at("motor_params_path");
  } catch (const std::out_of_range &) {
    RCLCPP_FATAL(rclcpp::get_logger(LOGGER),
      "Missing URDF param 'motor_params_path'. Was the robot launched with sim:=false?");
    return hardware_interface::CallbackReturn::ERROR;
  }

  YAML::Node cfg;
  try {
    cfg = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger(LOGGER),
      "Failed to load motor_params.yaml from '%s': %s", yaml_path.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Hardware-level params live under the 'hardware' key.
  const auto hw = cfg["hardware"];
  serial_port_            = hw["serial_port"].as<std::string>();
  baud_rate_              = hw["baud_rate"].as<int>();
  max_velocity_rad_per_s_ = hw["max_velocity_rad_per_s"].as<double>();
  servo_speed_units_      = hw["servo_speed_units"].as<uint16_t>();

  // Per-joint params: look up each joint by name under 'joints'.
  // info_.joints is ordered by the URDF (fr→fl→br→bl), which matches controllers.yaml.
  const auto joints_node = cfg["joints"];
  joint_configs_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const std::string & name = info_.joints[i].name;
    if (!joints_node[name]) {
      RCLCPP_FATAL(rclcpp::get_logger(LOGGER),
        "Joint '%s' not found in motor_params.yaml", name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto jn = joints_node[name];
    joint_configs_[i].motor_id     = jn["motor_id"].as<int>();
    joint_configs_[i].direction    = jn["direction"].as<int>();
    joint_configs_[i].offset_ticks = jn["offset_ticks"].as<int>();
  }

  // Allocate shared state/command vectors. NaN = "not yet valid".
  const size_t n = info_.joints.size();
  hw_positions_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_commands_.assign(n, std::numeric_limits<double>::quiet_NaN());
  prev_commands_.assign(n, 0.0);

  // E-stop node on its own thread — fires independently of the control loop.
  estop_node_ = std::make_shared<rclcpp::Node>("sts_hardware_estop");
  estop_sub_ = estop_node_->create_subscription<std_msgs::msg::Bool>(
    "/estop", rclcpp::QoS(1),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      estop_active_.store(msg->data);
      if (msg->data) {
        RCLCPP_WARN(rclcpp::get_logger(LOGGER), "E-STOP activated — torque disabled");
        disable_all_torque();
      } else {
        // When cleared, seed prev_commands from current positions so the
        // velocity clamp doesn't slam joints to wherever hw_commands_ was left.
        for (size_t i = 0; i < prev_commands_.size(); ++i) {
          prev_commands_[i] = hw_positions_[i];
        }
        RCLCPP_INFO(rclcpp::get_logger(LOGGER), "E-STOP cleared — re-enabling torque");
        for (const auto & cfg : joint_configs_) {
          driver_->EnableTorque(cfg.motor_id, 1);
        }
      }
    });

  estop_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  estop_executor_->add_node(estop_node_);
  estop_thread_ = std::thread([this]() { estop_executor_->spin(); });

  RCLCPP_INFO(rclcpp::get_logger(LOGGER),
    "Initialized: %zu joints | port=%s baud=%d max_vel=%.2f rad/s",
    info_.joints.size(), serial_port_.c_str(), baud_rate_, max_velocity_rad_per_s_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// export interfaces — give the controller manager pointers into our vectors
// ============================================================================

std::vector<hardware_interface::StateInterface>
StsSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface>
StsSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return out;
}

// ============================================================================
// on_activate — open port, ping motors, read positions, enable torque
// ============================================================================

hardware_interface::CallbackReturn StsSystemInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (driver_->begin(baud_rate_, serial_port_.c_str()) != 0) {
    RCLCPP_FATAL(rclcpp::get_logger(LOGGER),
      "Cannot open serial port: %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & cfg : joint_configs_) {
    if (driver_->Ping(cfg.motor_id) < 0) {
      RCLCPP_FATAL(rclcpp::get_logger(LOGGER),
        "Motor ID %d did not respond to ping", cfg.motor_id);
      driver_->end();
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Read current positions and seed commands = current, so first write is a no-op.
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    int ticks = driver_->ReadPos(joint_configs_[i].motor_id);
    if (ticks < 0) {
      RCLCPP_FATAL(rclcpp::get_logger(LOGGER),
        "Failed initial position read from motor ID %d", joint_configs_[i].motor_id);
      driver_->end();
      return hardware_interface::CallbackReturn::ERROR;
    }
    hw_positions_[i]  = ticks_to_rad(ticks, joint_configs_[i]);
    hw_velocities_[i] = 0.0;
    hw_commands_[i]   = hw_positions_[i];
    prev_commands_[i] = hw_positions_[i];
  }

  for (const auto & cfg : joint_configs_) {
    driver_->EnableTorque(cfg.motor_id, 1);
  }

  RCLCPP_INFO(rclcpp::get_logger(LOGGER), "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_deactivate — torque off, close port
// ============================================================================

hardware_interface::CallbackReturn StsSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  disable_all_torque();
  driver_->end();
  RCLCPP_INFO(rclcpp::get_logger(LOGGER), "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// read — SyncRead all 12 motors, convert ticks → radians
// ============================================================================

hardware_interface::return_type StsSystemInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const size_t n = joint_configs_.size();
  std::vector<uint8_t> ids(n);
  std::vector<int16_t> pos(n, 0);
  for (size_t i = 0; i < n; ++i) {
    ids[i] = static_cast<uint8_t>(joint_configs_[i].motor_id);
  }

  if (driver_->SyncReadPos(ids.data(), static_cast<uint8_t>(n), pos.data()) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "SyncReadPos failed");
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < n; ++i) {
    hw_positions_[i]  = ticks_to_rad(static_cast<int>(pos[i]), joint_configs_[i]);
    hw_velocities_[i] = 0.0;
  }
  return hardware_interface::return_type::OK;
}

// ============================================================================
// write — clamp velocity, convert radians → ticks, SyncWrite all 12 motors
// ============================================================================

hardware_interface::return_type StsSystemInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (estop_active_.load()) {
    return hardware_interface::return_type::OK;
  }

  const size_t n = joint_configs_.size();
  const double max_step = max_velocity_rad_per_s_ * period.seconds();

  std::vector<uint8_t>  ids(n);
  std::vector<int16_t>  positions(n);
  std::vector<uint16_t> speeds(n, servo_speed_units_);
  std::vector<uint8_t>  accs(n, 0);

  for (size_t i = 0; i < n; ++i) {
    double clamped = std::clamp(
      hw_commands_[i],
      prev_commands_[i] - max_step,
      prev_commands_[i] + max_step);
    prev_commands_[i] = clamped;

    ids[i]       = static_cast<uint8_t>(joint_configs_[i].motor_id);
    positions[i] = static_cast<int16_t>(
      std::clamp(rad_to_ticks(clamped, joint_configs_[i]), 0, 4095));
  }

  if (driver_->SyncWritePosEx(
    ids.data(), static_cast<uint8_t>(n),
    positions.data(), speeds.data(), accs.data()) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "SyncWritePosEx failed");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// Helpers
// ============================================================================

double StsSystemInterface::ticks_to_rad(int ticks, const JointMotorConfig & cfg) const
{
  return cfg.direction * (ticks - cfg.offset_ticks) / 4096.0 * (2.0 * M_PI);
}

int StsSystemInterface::rad_to_ticks(double rad, const JointMotorConfig & cfg) const
{
  return cfg.offset_ticks + cfg.direction * static_cast<int>(rad / (2.0 * M_PI) * 4096.0);
}

void StsSystemInterface::disable_all_torque()
{
  if (!driver_) return;
  for (const auto & cfg : joint_configs_) {
    driver_->EnableTorque(cfg.motor_id, 0);
  }
}

}  // namespace bot_hardware
