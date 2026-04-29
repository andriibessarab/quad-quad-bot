#include "rclcpp/rclcpp.hpp"
#include <memory>

class LimbCommander : public rclcpp::Node
{
public:
    LimbCommander() : Node("limb_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Limb Commander node has started!");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LimbCommander>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
