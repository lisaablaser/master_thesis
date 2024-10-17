#ifndef EXPLORATION_PLANNER_V3_HPP
#define EXPLORATION_PLANNER_V3_HPP

#include "automatic_cell_explorer/exploration_planner/exploration_planner.hpp"


class ExplorationPlannerV3: public ExplorationPlanner
/*
    This is a Local Planner, efficeintly sweeping a local area. 
*/
{
public:
  ExplorationPlannerV3(MoveGrpPtr mvt_interface,std::shared_ptr<octomap::OcTree> octo_map)
    : ExplorationPlanner(mvt_interface, octo_map)
    {}

  void calculateNbvCandidates() override;
  Nbv selectNbv() override;

private:
  void evaluateNbvCandidates();
  void generateCandidates();
  RayView getRayView(Nbv & nbv ); //move to ep?
  
};

#endif // RANDOM_EXPLORATION_PLANNER.HPP