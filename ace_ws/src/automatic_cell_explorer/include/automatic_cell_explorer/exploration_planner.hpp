#ifndef EXPLORATION_PLANNER_HPP
#define EXPLORATION_PLANNER_HPP

#include <memory>
#include <octomap/octomap.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_interface/planning_interface.h> 

#include "automatic_cell_explorer/move_robot_service.hpp"
#include "automatic_cell_explorer/utils.hpp"
#include "automatic_cell_explorer/moveit_interface.hpp"



class ExplorationPlanner
{
public:
  ExplorationPlanner(MoveGrpPtr mvt_interface,std::shared_ptr<octomap::OcTree> octo_map);

  automatic_cell_explorer::srv::MoveToNbv::Request calculate_nbv();
 
  double calculate_occupied_volume() const;


  RayView getCurrentRayView(
    double max_range = 100.0);



private:
  MoveGrpPtr mvt_interface_;
  std::shared_ptr<octomap::OcTree> octo_map_;

  moveit::planning_interface::MoveGroupInterface::Plan plan(geometry_msgs::msg::PoseStamped pose);
  geometry_msgs::msg::PoseStamped generate_nvb_candidate();

  double compute_node_volume(double resolution) const;
  
  
};


#endif // EXPLORATION_PLANNER_HPP