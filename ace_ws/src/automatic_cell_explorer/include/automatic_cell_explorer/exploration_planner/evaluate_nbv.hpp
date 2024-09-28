#ifndef EVALUATE_NBV_HPP
#define EVALUATE_NBV_HPP

#include "automatic_cell_explorer/exploration_planner/nbv.hpp"
#include "automatic_cell_explorer/exploration_planner/raycast.hpp"



void updateRayViews(NbvCandidates& nbvs,  std::shared_ptr<octomap::OcTree> octo_map);


#endif // EVALUATE_NBV_HPP