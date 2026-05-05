#include "rclcpp/rclcpp.hpp"

#include <memory>

class LimbTrajectoryGenerator : public rclcpp::Node {
public:
  LimbTrajectoryGenerator() : Node("limb_trajectory_generator") {
    RCLCPP_INFO(this->get_logger(), "Initializing limb_trajectory_generator node");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LimbTrajectoryGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}