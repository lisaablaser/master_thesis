#ifndef EXPLORATION_PLANNER_HPP
#define EXPLORATION_PLANNER_HPP

#include <memory>
#include <octomap/octomap.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <moveit/robot_trajectory/robot_trajectory.h>

//#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h> 

class ExplorationPlanner
{
public:
  ExplorationPlanner(std::shared_ptr<rclcpp::Node> node,std::shared_ptr<octomap::OcTree> octo_map);


  robot_trajectory::RobotTrajectory calculate_nbv();

  double calculate_occupied_volume() const;

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<octomap::OcTree> octo_map_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  planning_scene::PlanningScenePtr planning_scene_;
  planning_interface::PlannerManagerPtr planner_instance_;

  double compute_node_volume(double resolution) const;

  planning_interface::MotionPlanRequest generate_request();
  robot_trajectory::RobotTrajectory plan(planning_interface::MotionPlanRequest req);
  
};


#endif // EXPLORATION_PLANNER_HPP