#include <gtest/gtest.h>
#include "automatic_cell_explorer/state_machine.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>

TEST(StateMachineTest, InitialState)
{
    rclcpp::init(0, nullptr);
    auto state_machine = std::make_shared<StateMachineNode>();
    state_machine->execute_state_machine();

    
    while (rclcpp::ok() && !state_machine->is_finished()) {
        rclcpp::spin_some(state_machine);
    }

    std::cout << "Shutting down from test" << std::endl;
    // Add assertions to test initial state
    //ASSERT_TRUE(state_machine.isInitialized());
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}