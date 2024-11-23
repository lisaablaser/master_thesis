#ifndef EXPLORATION_PLANNER_ACE_HPP
#define EXPLORATION_PLANNER_ACE_HPP


#include "automatic_cell_explorer/exploration_planner/exploration_planner.hpp"



class ExplorationPlannerACE: public ExplorationPlanner
{
public:
  ExplorationPlannerACE(MoveGrpPtr mvt_interface, std::shared_ptr<octomap::OcTree> octo_map)
    : ExplorationPlanner(mvt_interface, octo_map)
     {}

  void calculateNbvCandidates(NbvCandidates & nbv_candidates) override;
  Nbv selectNbv(NbvCandidates & nbv_candidates) override;


  

private:
  
  void evaluateNbvCandidates(NbvCandidates & nbv_candidates);
  void generateCandidates(NbvCandidates & nbv_candidates);

  
  NbvCandidates findParetoFrontiers(const NbvCandidates & nbv_candidates);


  
  
};

#endif // EXPLORATION_PLANNER_ACE.HPP