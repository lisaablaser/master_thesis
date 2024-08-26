#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

class StateMachine
{
public:
    StateMachine();
    bool isInitialized() const;  // Add this method

    void run();
    // Other methods...

private:
    bool initialized_;  // Example private member to track initialization state
};

#endif // STATE_MACHINE_HPP