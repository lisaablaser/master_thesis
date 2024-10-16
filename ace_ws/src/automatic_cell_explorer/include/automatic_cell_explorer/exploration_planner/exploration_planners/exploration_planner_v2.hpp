#ifndef EXPLORATION_PLANNER_V2_HPP
#define EXPLORATION_PLANNER_V2_HPP

#include "automatic_cell_explorer/exploration_planner/exploration_planner.hpp"


class ExplorationPlannerV2: public ExplorationPlanner
{
public:
  ExplorationPlannerV2(MoveGrpPtr mvt_interface, std::shared_ptr<octomap::OcTree> octo_map)
    : ExplorationPlanner(mvt_interface, octo_map) {}

  void calculateNbvCandidates() override;
  Nbv selectNbv() override;

  RayView getRayView(Nbv & nbv );
  double getCost(Nbv & nbv);

private:

  void evaluateNbvCandidates();
  void calculateCost();

//modifies internal variables. 
  void generateCandidates();
  void update_trajectories();
  void update_ray_views();
  void remove_weak_candidates();
  
};

#endif // EXPLORATION_PLANNER.HPP