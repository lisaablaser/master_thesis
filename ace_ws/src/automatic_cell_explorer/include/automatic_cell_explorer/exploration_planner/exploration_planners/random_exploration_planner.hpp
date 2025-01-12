#ifndef RANDOM_EXPLORATION_PLANNER_HPP
#define RANDOM_EXPLORATION_PLANNER_HPP

#include "automatic_cell_explorer/exploration_planner/exploration_planner.hpp"


class RandomExplorationPlanner: public ExplorationPlanner
/*
  Default Global planner used for ablation studies.
*/
{
public:
  RandomExplorationPlanner(MoveGrpPtr mvt_interface, std::shared_ptr<octomap::OcTree> octo_map)
    : ExplorationPlanner(mvt_interface, octo_map)
     {}

  void calculateNbvCandidates() override;
  Nbv selectNbv() override;

private:
  void evaluateNbvCandidates();
  void generateCandidates();
  void generateCandidatesJointSpace();
  void generateCandidatesTarget();
  
  
};

#endif // RANDOM_EXPLORATION_PLANNER.HPP