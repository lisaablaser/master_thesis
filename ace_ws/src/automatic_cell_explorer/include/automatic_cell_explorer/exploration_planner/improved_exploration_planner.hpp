#ifndef IMPROVED_EXPLORATION_PLANNER_HPP
#define IMPROVED_EXPLORATION_PLANNER_HPP

#include "automatic_cell_explorer/exploration_planner/exploration_planner.hpp"


class ImprovedExplorationPlanner: public ExplorationPlanner
{
public:
  ImprovedExplorationPlanner(MoveGrpPtr mvt_interface, std::shared_ptr<octomap::OcTree> octo_map)
    : ExplorationPlanner(mvt_interface, octo_map) {}

  void calculateNbvCandidates() override;
  Nbv selectNbv() override;
  bool terminationCriteria() const override;

private:

  void evaluateNbvCandidates();
  void generateCandidates();
  
};

#endif // IMPROVED_EXPLORATION_PLANNER.HPP