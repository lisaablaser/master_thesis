#ifndef EP_FACTORY_HPP
#define EP_FACTORY_HPP

#include "automatic_cell_explorer/exploration_planner/exploration_planner.hpp"
#include "automatic_cell_explorer/exploration_planner/exploration_planners/demo_exploration_planner.hpp"
#include "automatic_cell_explorer/exploration_planner/exploration_planners/random_exploration_planner.hpp"
#include "automatic_cell_explorer/exploration_planner/exploration_planners/exploration_planner_ACEP_local.hpp"
#include "automatic_cell_explorer/exploration_planner/exploration_planners/exploration_planner_ACEP_global.hpp"
#include "automatic_cell_explorer/exploration_planner/exploration_planners/baseline_planner_local.hpp"
#include "automatic_cell_explorer/exploration_planner/exploration_planners/baseline_planner_global.hpp"

enum class PlannerType {
    Local,
    Global
};

std::shared_ptr<ExplorationPlanner> createPlanner(PlannerType type, MoveGrpPtr mvt_interface, std::shared_ptr<octomap::OcTree> octo_map);

#endif // EP_FACTORY_HPP