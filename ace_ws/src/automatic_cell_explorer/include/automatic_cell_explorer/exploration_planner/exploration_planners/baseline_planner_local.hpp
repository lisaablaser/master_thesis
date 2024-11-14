#ifndef BASELINE_PLANNER_LOCAL_HPP
#define BASELINE_PLANNER_LOCAL_HPP


#include "automatic_cell_explorer/exploration_planner/exploration_planner.hpp"



class BaselinePlannerLocal: public ExplorationPlanner
{
public:
  BaselinePlannerLocal(MoveGrpPtr mvt_interface, std::shared_ptr<octomap::OcTree> octo_map)
    : ExplorationPlanner(mvt_interface, octo_map)
     {}

  void calculateNbvCandidates() override;
  Nbv selectNbv() override;
  

private:
  
  void evaluateNbvCandidates();
  void generateCandidates();
  void add_pose_if_valid(std::vector<double> q);


  
  
};

#endif // BASELINE_PLANNER_LOCAL.HPP