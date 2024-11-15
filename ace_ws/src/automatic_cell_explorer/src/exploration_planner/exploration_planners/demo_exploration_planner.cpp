#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/math/Vector3.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pluginlib/class_loader.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>


#include "automatic_cell_explorer/exploration_planner/raycast.hpp"
#include "automatic_cell_explorer/exploration_planner/exploration_planners/demo_exploration_planner.hpp"

void DemoExplorationPlanner::calculateNbvCandidates() {
/*
    Demo Planner
*/
    if(nbv_candidates_.empty()){
        generateCandidates(0.5, 1.5, 10);
        return;
    }
    evaluateNbvCandidates();

}

Nbv DemoExplorationPlanner::selectNbv(){
    /*
        Get the next Nbv candidate, calculate a Plan and remove from candidates.
    */
    
    while(!nbv_candidates_.empty()){

        Nbv nbv = nbv_candidates_.front();
        nbv_candidates_.erase(nbv_candidates_.begin());

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

void DemoExplorationPlanner::evaluateNbvCandidates(){
    /*
        Just for inspection, evaluation does not affect the Nbv Selection.
    */
    
    std::cout << "updating ray view " << std::endl;
    for(Nbv &nbv: nbv_candidates_){
        Eigen::Isometry3d sensor_pose = nbv.pose;
        RayView ray_view = calculateRayView(sensor_pose, octo_map_);
        nbv.ray_view = ray_view;
    
    }
}


void DemoExplorationPlanner::generateCandidates(double radius, double height, int resolution_degrees)
/*
    Generates candidates along a circle.    
*/
{
    nbv_candidates_.clear();
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

        nbv_candidates_.push_back(nbv);
    }
}




