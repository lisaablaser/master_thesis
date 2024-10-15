#ifndef RANDOM_RANDOM_EXPLORATION_PLANNER_V2_HPP
#define RANDOM_EXPLORATION_PLANNER_V2_HPP

#include "automatic_cell_explorer/exploration_planner/exploration_planner.hpp"


class RandomExplorationPlannerV2: public ExplorationPlanner
{
public:
  RandomExplorationPlannerV2(MoveGrpPtr mvt_interface, planning_scene_monitor::PlanningSceneMonitorPtr plm_interface, std::shared_ptr<octomap::OcTree> octo_map)
    : ExplorationPlanner(mvt_interface, octo_map),
      plm_interface_(plm_interface)
     {}

  void calculateNbvCandidates() override;
  Nbv selectNbv() override;
  bool terminationCriteria() const override;

private:
  planning_scene_monitor::PlanningSceneMonitorPtr plm_interface_;
  void evaluateNbvCandidates();
  void generateCandidates();
  
  
};

#endif // RANDOM_EXPLORATION_PLANNER_V2.HPP