#ifndef BOT_HARDWARE__STS_SYSTEM_INTERFACE_HPP_
#define BOT_HARDWARE__STS_SYSTEM_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "std_msgs/msg/bool.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// Forward-declared so the header doesn't need to include the Feetech SDK headers.
// The full type is used only in the .cpp.
class SMS_STS;

namespace bot_hardware {

// Per-joint parameters read from motor_params.yaml via URDF <param> tags.
struct JointMotorConfig {
  int motor_id;
  int direction;     // +1 or -1 — flips physical motion relative to URDF axis
  int offset_ticks;  // tick value (0-4095) that equals 0 radians in the URDF frame
};

class StsSystemInterface : public hardware_interface::SystemInterface
{
public:
  StsSystemInterface();
  ~StsSystemInterface() override;

  // Called once when the plugin is loaded. Parses all URDF <param> tags
  // and allocates state/command storage.
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // Returns references into hw_positions_ / hw_velocities_ that the
  // controller manager hands to any controller that requests state.
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Returns references into hw_commands_ that the position controller writes into.
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Opens the USB-serial port, pings all motors, reads initial positions,
  // seeds hw_commands_ from those positions (so the first write is a no-op).
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Disables torque on all motors and closes the serial port.
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // SyncRead position from all 12 motors, converts ticks → radians.
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Velocity-clamps the commanded angles, converts radians → ticks,
  // SyncWrites to all 12 motors. Skips write if estop is active.
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  double ticks_to_rad(int ticks, const JointMotorConfig & cfg) const;
  int    rad_to_ticks(double rad, const JointMotorConfig & cfg) const;
  void   disable_all_torque();

  // Feetech SDK instance — heap-allocated to avoid including SDK headers here.
  std::unique_ptr<SMS_STS> driver_;

  // Hardware-level params (from URDF <hardware> block)
  std::string serial_port_;
  int         baud_rate_;
  double      max_velocity_rad_per_s_;
  uint16_t    servo_speed_units_;  // raw SDK speed arg; see motor_params.yaml comment

  // Per-joint config, in joint_group_position_controller order (fr→fl→br→bl)
  std::vector<JointMotorConfig> joint_configs_;

  // These three vectors are the shared memory between the hardware interface
  // and the ros2_control framework. StateInterface and CommandInterface objects
  // hold raw pointers into these vectors.
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> prev_commands_;  // used for per-cycle velocity clamping

  // E-stop: a separate rclcpp::Node spun on its own thread so /estop
  // callbacks can arrive even while read()/write() are blocking on serial I/O.
  std::atomic<bool>                                  estop_active_{false};
  std::shared_ptr<rclcpp::Node>                      estop_node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> estop_executor_;
  std::thread                                        estop_thread_;
};

}  // namespace bot_hardware

#endif  // BOT_HARDWARE__STS_SYSTEM_INTERFACE_HPP_
