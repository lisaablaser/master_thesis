#ifndef EXPLORATION_PLANNER_HPP
#define EXPLORATION_PLANNER_HPP

#include <memory>
#include <optional> 
#include <octomap/octomap.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_interface/planning_interface.h> 

#include "automatic_cell_explorer/execute_srv.hpp"
#include "automatic_cell_explorer/visualize.hpp"
#include "automatic_cell_explorer/moveit_types.hpp"
#include "automatic_cell_explorer/exploration_planner/nbv.hpp"




class ExplorationPlanner
{
public:
  
  ExplorationPlanner(MoveGrpPtr mvt_interface, std::shared_ptr<octomap::OcTree> octo_map);

  ExecuteReq get_nbv_demo();
 
  double calculate_occupied_volume() const;

  std::vector<RayView> getRayCast();

private:
  MoveGrpPtr mvt_interface_;
  std::shared_ptr<octomap::OcTree> octo_map_;
  NbvCandidates nbv_candidates_;

  std::optional<Plan> plan(geometry_msgs::msg::PoseStamped pose);
  void generate_nvb_candidates_circle(double radius, double height, int resolution_degrees);

  double compute_node_volume(double resolution) const;
  Nbv popFirstNbv();

  //NBV_candidates
  //select_nbv_from_candidates, and remove
  //
  
  
};


#endif // EXPLORATION_PLANNER_HPP