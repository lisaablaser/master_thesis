#ifndef EXPLORATION_PLANNER_V4_HPP
#define EXPLORATION_PLANNER_V4_HPP


#include "automatic_cell_explorer/exploration_planner/exploration_planner.hpp"



class ExplorationPlannerV4: public ExplorationPlanner
{
public:
  ExplorationPlannerV4(MoveGrpPtr mvt_interface, std::shared_ptr<octomap::OcTree> octo_map)
    : ExplorationPlanner(mvt_interface, octo_map)
     {}

  void calculateNbvCandidates() override;
  Nbv selectNbv() override;
  

private:
  
  void evaluateNbvCandidates();
  void generateCandidates();

  void findTargets();

  
  
};

#endif // EXPLORATION_PLANNER_V4.HPP