#ifndef EXPLORATION_PLANNER_HPP
#define EXPLORATION_PLANNER_HPP

#include <memory>
#include <optional> 
#include <octomap/octomap.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_interface/planning_interface.h> 
#include "automatic_cell_explorer/moveit_types.hpp"
#include "automatic_cell_explorer/exploration_planner/nbv.hpp"


class ExplorationPlanner {
public:
    ExplorationPlanner(MoveGrpPtr mvt_interface, std::shared_ptr<octomap::OcTree> octo_map)
        : mvt_interface_(mvt_interface), octo_map_(octo_map) {}

    virtual ~ExplorationPlanner() = default;

    // The base class declares virtual methods that derived classes will implement.
    virtual void calculateNbvCandidates() = 0;  // Pure virtual: Must be implemented by derived classes
    virtual Nbv selectNbv() = 0;  // Pure virtual: Must be implemented by derived classes

    
    NbvCandidates getNbvCandidates() const {
        return nbv_candidates_;
    }

    virtual bool terminationCriteria() const = 0;
    // termintionCriteria();

protected:
    MoveGrpPtr mvt_interface_;
    std::shared_ptr<octomap::OcTree> octo_map_; 
    NbvCandidates nbv_candidates_;  // Protected: Can be accessed and modified by derived classes

    // Utility methods shared between planners
    std::optional<Plan> plan(const Eigen::Isometry3d& pose);
    double calculate_occupied_volume() const;
    double compute_node_volume(double resolution) const;

};
#endif //EXPLORATION_PLANNER_HPP
