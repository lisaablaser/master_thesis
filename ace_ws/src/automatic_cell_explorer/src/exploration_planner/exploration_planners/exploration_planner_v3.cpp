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
        This is a Local Planner, efficeintly sweeping a local area. 
    */

    generateCandidates();
    evaluateNbvCandidates();

}


Nbv ExplorationPlannerV3::selectNbv(){
    /*
        Select the one with highes information gain. 
        Does not care about traj cost, by construction they are short. 
    */

    if (nbv_candidates_.nbv_candidates.empty()) {
        return Nbv();
    }

    auto nbv = nbv_candidates_.nbv_candidates.begin();
    float highest_utility = -INFINITY;

    for (auto it = nbv_candidates_.nbv_candidates.begin(); it != nbv_candidates_.nbv_candidates.end(); ++it )
    {
        float gain = it->ray_view.num_unknowns;
        float utility = gain; 

        if (utility > highest_utility) {
            highest_utility = utility;
            nbv = it;
        }
    }

    std::cout << "Utility of Nbv in Local planner (V3) is: " << highest_utility << std::endl;

    return *nbv;
    
   
}

void ExplorationPlannerV3::evaluateNbvCandidates(){
    /*
        Evaluates the candidates with rycasting. 
    */

    std::cout << "updating ray view " << std::endl;
    for (auto it = nbv_candidates_.nbv_candidates.begin(); it != nbv_candidates_.nbv_candidates.end(); ++it )
    {
        it->ray_view = getRayView(*it);

    }

    std::cout << "Number of unknowns hit by each view candidate: " << std::endl;
    for (auto it = nbv_candidates_.nbv_candidates.begin(); it != nbv_candidates_.nbv_candidates.end(); ++it )
    {
        std::cout << it->ray_view.num_unknowns << std::endl;
    }

}



void ExplorationPlannerV3::generateCandidates()
/*
    Local Planner

    Generatees candidates on a -+45 deg spherical cap around the current position on the wrist joint. 
*/
/// Bug: this can be empty, and leads to a bug.
{
    nbv_candidates_.nbv_candidates.clear();
        
    std::vector<double> joint_values = mvt_interface_->getCurrentJointValues();
    const std::vector<std::string, std::allocator<std::string>>  joint_names = mvt_interface_->getJointNames();

    
    double joint_min = -M_PI/8; 
    double joint_max = M_PI/8; 
    double step_size = 30.0*(M_PI/180); 

    double joint_i = joint_values[3];
    double joint_j = joint_values[4];
    double joint_k = joint_values[5];

    // Generate poses +-45 deg of current pose
    for (double value_i = (joint_i + joint_min) ; value_i <= (joint_i + joint_max); value_i += step_size) {
        for (double value_j = (joint_j + joint_min); value_j <= (joint_j + joint_max); value_j += step_size) {
            for (double value_k = (joint_k + joint_min); value_k <= (joint_k + joint_max); value_k += step_size) {
                Nbv nbv;

                std::vector<double> candidate_joint_values = joint_values;
                candidate_joint_values[3] = value_i; 
                candidate_joint_values[4] = value_j;
                candidate_joint_values[5] = value_k;

                auto result = plan(candidate_joint_values);
                if (result) {
                    nbv.plan = *result;
                    nbv.pose = forward_kinematics(candidate_joint_values);
                    nbv.ray_view = getRayView(nbv);


                    nbv_candidates_.nbv_candidates.push_back(nbv);
                }
                std::cout << " Plans in local planner" << std::endl;
            }
        }
    }
}




RayView ExplorationPlannerV3::getRayView(Nbv & nbv ){

    Eigen::Isometry3d& sensor_pose = nbv.pose;
    RayView ray_view = calculateRayView(sensor_pose, octo_map_);
    
    return ray_view;

}