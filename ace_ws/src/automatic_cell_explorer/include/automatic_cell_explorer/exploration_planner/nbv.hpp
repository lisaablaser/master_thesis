#ifndef NBV_HPP
#define NBV_HPP

#include "automatic_cell_explorer/moveit_types.hpp"
#include "automatic_cell_explorer/exploration_planner/raycast.hpp"


struct Nbv{
  Eigen::Isometry3d pose;
  Plan plan;
  RayView ray_view;
  double cost;
};

struct NbvCandidates{
  std::vector<Nbv> nbv_candidates;
};

#endif // NBV_HPP