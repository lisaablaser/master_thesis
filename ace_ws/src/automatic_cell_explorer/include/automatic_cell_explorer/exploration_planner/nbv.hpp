#ifndef NBV_HPP
#define NBV_HPP

#include "automatic_cell_explorer/moveit_types.hpp"


struct Nbv{
  geometry_msgs::msg::PoseStamped pose;
  Plan plan;
  double cost;
};

struct NbvCandidates{
  std::vector<Nbv> nbv_candidates;
};

#endif // NBV_HPP