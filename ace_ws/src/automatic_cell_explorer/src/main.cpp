#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "automatic_cell_explorer/moveit_interface.hpp"
#include "automatic_cell_explorer/move_robot_service.hpp"
#include "automatic_cell_explorer/state_machine.hpp"

int main(int argc, char **argv)
{
    
    rclcpp::init(argc, argv);
    auto moveit_node = std::make_shared<rclcpp::Node>(
    "moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    MvtInterfacePtr mvt_interface =getInterface(moveit_node);

    auto move_robot_node = std::make_shared<MoveRobotNode>(mvt_interface);
    auto state_machine_node = std::make_shared<StateMachineNode>(mvt_interface);

    move_robot_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    moveit_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    state_machine_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    
    
    
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add both nodes to the executor
    executor.add_node(moveit_node);
    executor.add_node(move_robot_node);
    //executor.add_node(state_machine_node);
    

    // Spin the executor (this will run both nodes concurrently)
    std::thread spin_thread([&executor]() {
        executor.spin();
    });


    state_machine_node->execute_state_machine();
    
    

    std::cout << "Shutting down from main" << std::endl;
    rclcpp::shutdown();

    return 0;
}