#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>  
#include <moveit_msgs/msg/planning_scene_world.hpp>

#include "automatic_cell_explorer/move_robot_service.hpp"
#include "automatic_cell_explorer/moveit_interface.hpp"
#include "automatic_cell_explorer/exploration_planner/exploration_planner.hpp"



enum class State {Initialise, Capture, Calculate_NBV, Move_robot, Finished, WaitingForOctomap, Error};

class StateMachineNode : public rclcpp::Node
{
public:
    using ExplorationPlannerPtr = std::shared_ptr<ExplorationPlanner>;

    StateMachineNode(MoveGrpPtr mvt_interface);
    void execute_state_machine();
    bool is_finished() const { return finished_; }

private:
    std::shared_ptr<rclcpp::Node> node_;
    MoveGrpPtr mvt_interface_;
    State current_state_;
    std::atomic<bool> finished_;
    std::shared_ptr<octomap::OcTree> octomap_;
    ExplorationPlannerPtr exploration_planner_;
    ExecuteReq current_req_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_trigger_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscriber_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr world_publisher_;
    rclcpp::Client<Execute>::SharedPtr move_client_;


    // State handling methods
    void handle_initialise();
    void handle_capture();
    void handle_calculate_nbv();
    void handle_move_robot();


    // Callback methods
    void update_planning_scene(const octomap_msgs::msg::Octomap::SharedPtr msg);

};

#endif // STATE_MACHINE_HPP