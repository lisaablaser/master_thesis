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
/// TODO: make this a template function
{
public:
  
  ExplorationPlanner(MoveGrpPtr mvt_interface, std::shared_ptr<octomap::OcTree> octo_map);

  ExecuteReq get_nbv_demo();

  // Helper functions to visualize
  double calculate_occupied_volume() const;
  void generateCandidates() {generate_nvb_candidates_circle(0.5, 1.5, 10);}
  void evaluateNbvCandidates();
  Nbv getNbv() const;
  NbvCandidates getNbvCandidates()const{return nbv_candidates_;};

private:
  MoveGrpPtr mvt_interface_;
  std::shared_ptr<octomap::OcTree> octo_map_;
  NbvCandidates nbv_candidates_;

  std::optional<Plan> plan(const Eigen::Isometry3d& pose);
  void generate_nvb_candidates_circle(double radius, double height, int resolution_degrees);

  double compute_node_volume(double resolution) const;
  Nbv popFirstNbv();
  
  
};


#endif // EXPLORATION_PLANNER_HPP