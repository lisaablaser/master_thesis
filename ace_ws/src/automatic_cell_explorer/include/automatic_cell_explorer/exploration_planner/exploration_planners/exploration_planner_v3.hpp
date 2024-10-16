#ifndef EXPLORATION_PLANNER_V3_HPP
#define EXPLORATION_PLANNER_V3_HPP

#include "automatic_cell_explorer/exploration_planner/exploration_planner.hpp"


class ExplorationPlannerV3: public ExplorationPlanner
{
public:
  ExplorationPlannerV3(MoveGrpPtr mvt_interface, planning_scene_monitor::PlanningSceneMonitorPtr plm_interface, std::shared_ptr<octomap::OcTree> octo_map)
    : ExplorationPlanner(mvt_interface, octo_map),
      plm_interface_(plm_interface)
     {}

  void calculateNbvCandidates() override;
  Nbv selectNbv() override;

private:
  planning_scene_monitor::PlanningSceneMonitorPtr plm_interface_;
  void evaluateNbvCandidates();
  void generateCandidates();
  
  
};

#endif // RANDOM_EXPLORATION_PLANNER.HPP