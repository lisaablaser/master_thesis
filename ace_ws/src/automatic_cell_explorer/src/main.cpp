#include <rclcpp/rclcpp.hpp>
#include "automatic_cell_explorer/state_machine.hpp"

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto state_machine = std::make_shared<StateMachineNode>();
    state_machine->execute_state_machine();

    
    while (rclcpp::ok() && !state_machine->is_finished()) {
        rclcpp::spin_some(state_machine);
    }

    std::cout << "Shutting down from main" << std::endl;
    rclcpp::shutdown();

    return 0;
}