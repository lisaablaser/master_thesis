#include <rclcpp/rclcpp.hpp>
#include "automatic_cell_explorer/state_machine.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateMachineNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}