#include <gtest/gtest.h>
#include "automatic_cell_explorer/state_machine.hpp"
#include <iostream>

TEST(StateMachineTest, InitialState)
{
    StateMachine state_machine;

    std::cout << "Testing if it prints somewhere" << std::endl;

    // Add assertions to test initial state
    ASSERT_TRUE(state_machine.isInitialized());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}