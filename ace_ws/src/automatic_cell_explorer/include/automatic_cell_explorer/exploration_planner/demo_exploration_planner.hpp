#ifndef DEMO_EXPLORATION_PLANNER_HPP
#define DEMO_EXPLORATION_PLANNER_HPP

#include "automatic_cell_explorer/exploration_planner/exploration_planner.hpp"


class DemoExplorationPlanner: public ExplorationPlanner
{
public:
  DemoExplorationPlanner(MoveGrpPtr mvt_interface, std::shared_ptr<octomap::OcTree> octo_map)
    : ExplorationPlanner(mvt_interface, octo_map) {generateCandidates(0.5, 1.5, 10);}

  void calculateNbvCandidates() override;
  Nbv selectNbv() override;
  bool terminationCriteria() const override;

private:

  void evaluateNbvCandidates();
  void generateCandidates(double radius, double height, int resolution_degrees);
  
};

#endif // DEMO_EXPLORATION_PLANNER.HPP