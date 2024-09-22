#ifndef EXPLORATION_PLANNER_HPP
#define EXPLORATION_PLANNER_HPP

#include <memory>
#include <octomap/octomap.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_interface/planning_interface.h> 

#include "automatic_cell_explorer/utils.hpp"
#include "automatic_cell_explorer/moveit_interface.hpp"



class ExplorationPlanner
{
public:
  ExplorationPlanner(MoveGrpPtr mvt_interface,std::shared_ptr<octomap::OcTree> octo_map);

  robot_trajectory::RobotTrajectory calculate_nbv();
 
  double calculate_occupied_volume() const;


  RayView getCurrentRayView(
    double max_range = 100.0);



private:
  MoveGrpPtr mvt_interface_;
  std::shared_ptr<octomap::OcTree> octo_map_;

  robot_trajectory::RobotTrajectory plan(planning_interface::MotionPlanRequest req);
  planning_interface::MotionPlanRequest generate_nvb_candidate();

  double compute_node_volume(double resolution) const;
  
  
};


#endif // EXPLORATION_PLANNER_HPP