#include <rclcpp/rclcpp.hpp>
#include "automatic_cell_explorer/moveit_interface.hpp"
#include "automatic_cell_explorer/move_robot_service.hpp"

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto moveit_node = std::make_shared<rclcpp::Node>( //should make const
    "moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    MvtInterfacePtr mvt_interface =getInterface(moveit_node);

    auto move_robot_node = std::make_shared<MoveRobotNode>(mvt_interface);
    
    
    rclcpp::spin(move_robot_node);

    std::cout << "Shutting down from main" << std::endl;
    rclcpp::shutdown();

    return 0;
}