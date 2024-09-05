#ifndef EXPLORATION_PLANNER_HPP
#define EXPLORATION_PLANNER_HPP

#include <memory>
#include <octomap/octomap.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_interface/planning_interface.h> 

struct RayInfo {
    Eigen::Vector3d start;  // Ray start (sensor origin)
    Eigen::Vector3d end;    // Ray end (hit point or max range)
    bool hit_unknown;       // Whether the ray hit an unknown region
};

class ExplorationPlanner
{
public:
  ExplorationPlanner(std::shared_ptr<rclcpp::Node> node,std::shared_ptr<octomap::OcTree> octo_map);

  robot_trajectory::RobotTrajectory calculate_nbv();
  void update_states(std::shared_ptr<octomap::OcTree> octo_map);
  double calculate_occupied_volume() const;


  std::pair<int, std::vector<RayInfo>> test_sim_view(octomap::OcTree& octree, 
    double max_range);



private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<octomap::OcTree> octo_map_;

  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;

  planning_scene::PlanningScenePtr planning_scene_;
  planning_interface::PlannerManagerPtr planner_instance_;

  robot_trajectory::RobotTrajectory plan(planning_interface::MotionPlanRequest req);
  planning_interface::MotionPlanRequest generate_request();

  std::pair<int, std::vector<RayInfo>> simulateInformationGainWithRays(
    const Eigen::Isometry3d& sensor_state, 
    octomap::OcTree& octree, 
    double max_range); 

  double compute_node_volume(double resolution) const;
  
};


#endif // EXPLORATION_PLANNER_HPP