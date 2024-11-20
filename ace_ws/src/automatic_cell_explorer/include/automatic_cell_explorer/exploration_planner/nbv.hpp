#ifndef NBV_HPP
#define NBV_HPP

#include "automatic_cell_explorer/moveit_types.hpp"
#include "automatic_cell_explorer/exploration_planner/raycast.hpp"


struct Nbv{
  Eigen::Isometry3d pose;
  Plan plan;
  RayView ray_view; //TODO: remove this
  double cost;
  double gain;

};

inline bool operator==(const Nbv& nbv1, const Nbv& nbv2) {
    constexpr double tolerance = 1e-6;

    // Compare pose
    if (!(nbv1.pose.translation().isApprox(nbv2.pose.translation(), tolerance) &&
          nbv1.pose.rotation().isApprox(nbv2.pose.rotation(), tolerance))) {
        return false;
    }

    return true;
}

using NbvCandidates = std::vector<Nbv>;


#endif // NBV_HPP