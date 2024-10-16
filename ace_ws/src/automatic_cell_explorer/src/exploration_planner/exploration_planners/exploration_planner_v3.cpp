#include <random>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/math/Vector3.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pluginlib/class_loader.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "automatic_cell_explorer/constants.hpp"
#include "automatic_cell_explorer/exploration_planner/raycast.hpp"
#include "automatic_cell_explorer/exploration_planner/exploration_planners/exploration_planner_v3.hpp"

void ExplorationPlannerV3::calculateNbvCandidates() {
    /*
        initial dummy random generator planner 
    */

    generateCandidates();
    evaluateNbvCandidates();

}


Nbv ExplorationPlannerV3::selectNbv(){
    /*
        Get the next Nbv candidate, Plans are alraeady calculated.
        Select the one with highes information gain. 
    */

    
   
}

void ExplorationPlannerV3::evaluateNbvCandidates(){
    /*
        Evaluates the candidates with rycasting. 
    */
    double max_range = 0.93;

    
    std::cout << "updating ray view " << std::endl;
    for(Nbv &nbv: nbv_candidates_.nbv_candidates){
        Eigen::Isometry3d sensor_pose = nbv.pose;
        RayView ray_view = calculateRayView(sensor_pose, octo_map_, max_range);
        nbv.ray_view = ray_view;
    
    }

    std::cout << "Number of unknowns hit by each view candidate: " << std::endl;
    for(Nbv nbv: nbv_candidates_.nbv_candidates){
        std::cout << nbv.ray_view.num_unknowns << std::endl;
    }

}



void ExplorationPlannerV3::generateCandidates()
/*
   
*/
{

    
}




