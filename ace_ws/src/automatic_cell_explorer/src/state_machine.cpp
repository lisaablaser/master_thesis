#include "automatic_cell_explorer/state_machine.hpp"


StateMachine::StateMachine() : initialized_(false)
{
    // Initialization code...
    initialized_ = true;  // Example setting of the initialized flag
    
}

bool StateMachine::isInitialized() const
{
    return initialized_;
}

void StateMachine::run()
{
    // State machine logic...
}