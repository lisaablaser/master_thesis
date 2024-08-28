#ifndef EXPLORATION_PLANNER_HPP
#define EXPLORATION_PLANNER_HPP

#include <memory>
#include <octomap/octomap.h>

class ExplorationPlanner
{
public:
  ExplorationPlanner(std::shared_ptr<octomap::OcTree> planning_scene);

  std::shared_ptr<octomap::point3d> calculate_next_best_view();

  double calculate_occupied_volume() const;

private:

  std::shared_ptr<octomap::OcTree> planning_scene_;

  double compute_node_volume(double resolution) const;

  
};


#endif // EXPLORATION_PLANNER_HPP