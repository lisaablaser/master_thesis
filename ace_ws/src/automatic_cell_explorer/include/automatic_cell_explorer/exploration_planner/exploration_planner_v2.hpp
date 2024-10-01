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
  bool terminationCriteria() const override;

private:

  void evaluateNbvCandidates();
  void generateCandidates();
  
};

#endif // EXPLORATION_PLANNER_V2.HPP