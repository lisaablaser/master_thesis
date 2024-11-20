#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>  
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include "logger.hpp"
#include "automatic_cell_explorer/move_robot_service.hpp"
#include "automatic_cell_explorer/moveit_interface.hpp"
#include "automatic_cell_explorer/octomap_processor.hpp"
#include "automatic_cell_explorer/ep_factory.hpp"




enum class State {Initialise, Capture, Calculate_NBV, Move_robot, Finished, WaitingForOctomap, Error};

class StateMachineNode : public rclcpp::Node
{
public:
    using ExplorationPlannerPtr = std::shared_ptr<ExplorationPlanner>;

    StateMachineNode(MoveGrpPtr mvt_interface, planning_scene_monitor::PlanningSceneMonitorPtr plm_interface, RvizToolPtr rviz_tool);
    void execute_state_machine();
    bool is_finished() const { return finished_; }



private:
    std::shared_ptr<rclcpp::Node> node_;
    MoveGrpPtr mvt_interface_;
    planning_scene_monitor::PlanningSceneMonitorPtr plm_interface_;
    RvizToolPtr rviz_tool_;
    State current_state_;
    PlannerType current_type_;
    std::atomic<bool> finished_;
    OctreePtr octomap_;
    ExplorationPlannerPtr exploration_planner_;
    ExecuteReq current_req_;
    int iteration_;
    Logger logger_;
    SmLog log_;
    NbvCandidates nbv_candidates_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_trigger_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscriber_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr world_publisher_;
    rclcpp::Client<Execute>::SharedPtr move_client_;
    double prev_progress_;

    void updatePlanner(PlannerType type) {
        exploration_planner_ = createPlanner(type, mvt_interface_, octomap_);
    }

    bool is_deafault(Nbv& nbv){ //delete this
        return nbv.pose.isApprox(Eigen::Isometry3d::Identity());
        }
    
    void handle_initialise();
    void handle_capture();
    void handle_calculate_nbv();
    void handle_move_robot();


    // Callback
    void update_planning_scene(const octomap_msgs::msg::Octomap::SharedPtr msg);


    void logIteration() {
        LogEntry entry;
        entry.sm_log = log_;

        if (exploration_planner_) {
            entry.ep_log = exploration_planner_->getLog(); 
        }
        logger_.logData(entry);

        log_ = SmLog{};
        log_.iteration = iteration_;
        iteration_ += 1;
        log_.time = std::chrono::system_clock::now();
    }
};

#endif // STATE_MACHINE_HPP