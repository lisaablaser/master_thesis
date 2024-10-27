#ifndef EXPLORATION_PLANNER_HPP
#define EXPLORATION_PLANNER_HPP

#include <memory>
#include <optional> 
#include <octomap/octomap.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_interface/planning_interface.h> 

#include "logger.hpp"
#include "automatic_cell_explorer/moveit_types.hpp"
#include "automatic_cell_explorer/clustering.hpp"
#include "automatic_cell_explorer/exploration_planner/nbv.hpp"



class ExplorationPlanner {
public:
    ExplorationPlanner(MoveGrpPtr mvt_interface, std::shared_ptr<octomap::OcTree> octo_map)
        : mvt_interface_(mvt_interface), octo_map_(octo_map), clusters_() {}

    virtual ~ExplorationPlanner() = default;


    virtual void calculateNbvCandidates() = 0;  
    virtual Nbv selectNbv() = 0; 

    void updateOctomap(std::shared_ptr<octomap::OcTree> octomap){
        octo_map_ = octomap;
    }
    
    
    NbvCandidates getNbvCandidates() const {
        return nbv_candidates_;
    }
    std::vector<Cluster> getClusters() const {return clusters_;}
    EpLog getLog() const {return log_;}


protected:
    MoveGrpPtr mvt_interface_;
    std::shared_ptr<octomap::OcTree> octo_map_; 
    std::vector<Cluster> clusters_;
    NbvCandidates nbv_candidates_; 
    EpLog log_;
    

    std::optional<Plan> plan(const Eigen::Isometry3d& pose);
    std::optional<Plan> plan(const std::vector<double> & joint_values);
    Eigen::Isometry3d forward_kinematics(std::vector<double> joint_values);
    double calculate_occupied_volume() const;
    double compute_node_volume(double resolution) const;

};
#endif //EXPLORATION_PLANNER_HPP
