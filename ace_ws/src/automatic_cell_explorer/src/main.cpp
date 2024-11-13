#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "automatic_cell_explorer/moveit_interface.hpp"
#include "automatic_cell_explorer/move_robot_service.hpp"
#include "automatic_cell_explorer/state_machine.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto moveit_node = std::make_shared<rclcpp::Node>(
    "mvt_interface_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).allow_undeclared_parameters(true)
    );
    auto pc_node = std::make_shared<rclcpp::Node>(
    "pc_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    auto rviz_node = std::make_shared<rclcpp::Node>(
    "rviz_tools",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // // Get kinematics parameters
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(moveit_node, "/move_group");
    while (!parameters_client->wait_for_service()) {
      if (!rclcpp::ok()) {
        rclcpp::shutdown();
      }
    }
    rcl_interfaces::msg::ListParametersResult parameter_list = parameters_client->list_parameters({"robot_description_kinematics"},10);
    auto parameters = parameters_client->get_parameters(parameter_list.names);

    moveit_node->set_parameters(parameters);





    MoveGrpPtr mvt_interface = getMoveGroupInterface(moveit_node);
    planning_scene_monitor::PlanningSceneMonitorPtr plm_interface = getPlanningSceeneMonitiorPtr(pc_node);
    RvizToolPtr rviz_tool = getRvizToolPtr(rviz_node, plm_interface);
    
    auto move_robot_service_node = std::make_shared<MoveRobotService>(mvt_interface);
    auto state_machine_node = std::make_shared<StateMachineNode>(mvt_interface, plm_interface, rviz_tool);

    move_robot_service_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    moveit_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    pc_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    state_machine_node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(moveit_node);
    executor.add_node(move_robot_service_node);
    executor.add_node(pc_node);
    executor.add_node(rviz_node);
    

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    std::vector<std::string> topics;
    plm_interface -> getMonitoredTopics(topics);
    for (auto t:topics){std::cout<<t << " ";}
    std::cout<<std::endl;
   
    state_machine_node->execute_state_machine();
    while (rclcpp::ok() ) {
    }
    
    std::cout << "Shutting down from main" << std::endl;
    rclcpp::shutdown();

    return 0;
}