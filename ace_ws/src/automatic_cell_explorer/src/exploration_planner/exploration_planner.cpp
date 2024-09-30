#include "automatic_cell_explorer/exploration_planner/exploration_planner.hpp"


std::optional<Plan> ExplorationPlanner::plan(const Eigen::Isometry3d& pose){
  /// TODO: include checks to terminate planner faster. 

  planning_interface::MotionPlanRequest req;

  req.group_name = "ur_manipulator";

  req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
  req.workspace_parameters.min_corner.z = -5.0;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
  req.workspace_parameters.max_corner.z = 5.0;

  mvt_interface_->setPoseTarget(pose);

  /// TODO: configure this somewhere else?
  mvt_interface_->setPlanningTime(0.1); 

  auto const [success, plan] = [&mvt_interface = mvt_interface_]{
    Plan p;
    auto const ok = static_cast<bool>(mvt_interface->plan(p));
    return std::make_pair(ok, p);
  }();

  if(success) {
    std::cout << "Planning succeeded. Executing..." << std::endl;
    return plan;

  } else {
    std::cout << "Planning failed!" << std::endl;
    return std::nullopt; 

  }

  return plan;
}

double ExplorationPlanner::calculate_occupied_volume() const
{
  double volume = 0.0;
  for (auto it = octo_map_->begin_leafs(), end = octo_map_->end_leafs(); it != end; ++it) {
      if (octo_map_->isNodeOccupied(*it)) {
          volume += compute_node_volume(octo_map_->getResolution());
      }
  }
  return volume;
}

double ExplorationPlanner::compute_node_volume(double resolution) const
{
  return resolution * resolution * resolution;
}