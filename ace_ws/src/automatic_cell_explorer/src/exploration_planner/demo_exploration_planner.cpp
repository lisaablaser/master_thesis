#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/math/Vector3.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pluginlib/class_loader.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "automatic_cell_explorer/exploration_planner/evaluate_nbv.hpp"
#include "automatic_cell_explorer/exploration_planner/demo_exploration_planner.hpp"

void DemoExplorationPlanner::calculateNbvCandidates() {
/*
    Demo Planner
*/
    if(nbv_candidates_.nbv_candidates.empty()){
        generateCandidates(0.5, 1.5, 10);
        return;
    }
    evaluateNbvCandidates();

}

Nbv DemoExplorationPlanner::selectNbv(){
    /*
        Get the next Nbv candidate, calculate a Plan and remove from candidates.
    */
    
    while(!nbv_candidates_.nbv_candidates.empty()){

        Nbv nbv = nbv_candidates_.nbv_candidates.front();
        nbv_candidates_.nbv_candidates.erase(nbv_candidates_.nbv_candidates.begin());

        auto result = plan(nbv.pose);
        if (result) {
            Plan valid_plan = *result;  
            nbv.plan = valid_plan;
            return nbv;
        }
    }
    // if no valid nbv exists, return empty
    return Nbv();
}
bool DemoExplorationPlanner::terminationCriteria() const {
    /*
        Termination Criteria from Demo planner: When nbv_candidates are empty.
    */
    bool terminate = nbv_candidates_.nbv_candidates.empty();
    return false;
}

void DemoExplorationPlanner::evaluateNbvCandidates(){
    /*
        Just for inspection, evaluation does not affect the Nbv Selection.
    */
    updateRayViews(nbv_candidates_,octo_map_);
}


void DemoExplorationPlanner::generateCandidates(double radius, double height, int resolution_degrees)
/*
    Generates candidates along a circle.    
*/
{
    nbv_candidates_.nbv_candidates.clear();
    for (int angle_deg = 0; angle_deg < 360; angle_deg += resolution_degrees)
    {
        Nbv nbv;
        
        double angle_rad = angle_deg * M_PI / 180.0;

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Vector3d(radius * std::cos(angle_rad), 
                                             radius * std::sin(angle_rad),  
                                             height);                       

        
        Eigen::AngleAxisd yaw_rotation(angle_rad, Eigen::Vector3d::UnitZ());
        pose.rotate(yaw_rotation);

        nbv.pose = pose;
        nbv.cost = 0.0; 
        nbv.plan = Plan();

        nbv_candidates_.nbv_candidates.push_back(nbv);
    }
}




