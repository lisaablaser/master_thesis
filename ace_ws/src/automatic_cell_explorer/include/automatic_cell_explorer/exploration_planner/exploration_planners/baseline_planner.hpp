#ifndef BASELINE_PLANNER_HPP
#define BASELINE_PLANNER_HPP


#include "automatic_cell_explorer/exploration_planner/exploration_planner.hpp"



class BaselinePlanner: public ExplorationPlanner
{
public:
  BaselinePlanner(MoveGrpPtr mvt_interface, std::shared_ptr<octomap::OcTree> octo_map)
    : ExplorationPlanner(mvt_interface, octo_map)
     {}


  void calculateNbvCandidates(NbvCandidates & nbv_candidates) override;
  Nbv selectNbv(NbvCandidates & nbv_candidates) override;

private:
  
  void evaluateNbvCandidates(NbvCandidates & nbv_candidates);
  void generateCandidates(NbvCandidates & nbv_candidates);
  NbvCandidates findParetoFrontiers(const NbvCandidates & nbv_candidates);
  NbvCandidates sortCandidates(const NbvCandidates & nbv_candidates);

  
  
};

#endif // BASELINE_PLANNER.HPP