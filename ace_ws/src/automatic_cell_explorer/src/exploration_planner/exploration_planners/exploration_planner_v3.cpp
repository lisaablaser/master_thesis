#include <random>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/math/Vector3.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pluginlib/class_loader.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "logger.hpp"
#include "automatic_cell_explorer/constants.hpp"
#include "automatic_cell_explorer/exploration_planner/raycast.hpp"
#include "automatic_cell_explorer/exploration_planner/exploration_planners/exploration_planner_v3.hpp"

void ExplorationPlannerV3::calculateNbvCandidates() {
    /*
        This is a Local Planner, efficeintly sweeping a local area. 
    */

    log_ = EpLog{};

    auto s_gen = std::chrono::high_resolution_clock::now();
    generateCandidates();
    auto e_gen = std::chrono::high_resolution_clock::now();
    auto d_gen = std::chrono::duration_cast<std::chrono::milliseconds>(e_gen - s_gen).count();
    log_.generate_t = d_gen; 

    auto s_eval = std::chrono::high_resolution_clock::now();
    evaluateNbvCandidates();
    auto e_eval = std::chrono::high_resolution_clock::now();
    auto d_eval = std::chrono::duration_cast<std::chrono::milliseconds>(e_eval - s_eval).count();
    log_.evaluate_t = d_eval; 
}


Nbv ExplorationPlannerV3::selectNbv(){
    /*
        Select the one with highes information gain. 
        Does not care about traj cost, by construction they are short. 
    */

    if (nbv_candidates_.nbv_candidates.empty()) {
        return Nbv();
        
    }

    const Nbv* nbv = &nbv_candidates_.nbv_candidates.at(0);
    double gain = nbv->ray_view.num_unknowns;
    double cost = 0;
    double best_utility = gain - cost;

    for (const auto& candidate : nbv_candidates_.nbv_candidates) {
        double cand_gain = candidate.ray_view.num_unknowns;
        double cand_cost = 0;
        double utility = cand_gain- cand_cost;

        if (utility > best_utility) {
            
            best_utility = utility;

            nbv = &candidate;
        }
    }
    
    std::cout << "Cost of Nbv is: " << best_utility << std::endl;

    log_.est_gain = static_cast<double>(nbv->ray_view.num_unknowns);
    log_.utility_score = best_utility;

    return *nbv;
    
   
}

void ExplorationPlannerV3::evaluateNbvCandidates(){
    /*
        Evaluates the candidates with rycasting. 
    */

    std::cout << "updating ray view " << std::endl;

    double total_time = 0.0;

    for (Nbv &nbv : nbv_candidates_.nbv_candidates) {
        Eigen::Isometry3d sensor_pose = nbv.pose;

        auto start_time = std::chrono::high_resolution_clock::now();

        RayView ray_view = calculateRayView(sensor_pose, octo_map_);
        nbv.ray_view = ray_view;

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        total_time += duration;
  
    }

    int n_candidates = nbv_candidates_.nbv_candidates.size();
    double average_time = (n_candidates > 0) ? (total_time / n_candidates) : 0.0;
    log_.n_candidates = n_candidates;
    log_.evaluate_av_t = average_time;


    std::cout << "Number of unknowns hit by each view candidate: " << std::endl;
    for(Nbv nbv: nbv_candidates_.nbv_candidates){
        std::cout << nbv.ray_view.num_unknowns << std::endl;
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
    mvt_interface_->getCurrentState(); //maybe a bug fix: prev joint values where somtimes old 
    std::vector<double> joint_values = mvt_interface_->getCurrentJointValues();
    const std::vector<std::string, std::allocator<std::string>>  joint_names = mvt_interface_->getJointNames();

    
    double joint_min = -M_PI/8; 
    double joint_max = M_PI/8; 
    double step_size = 30.0*(M_PI/180); 

    double joint_i = joint_values[3];
    double joint_j = joint_values[4];
    double joint_k = joint_values[5];

    // Generate poses +-45 deg of current pose
    int i = 0;
    for (double value_i = (joint_i + joint_min) ; value_i <= (joint_i + joint_max); value_i += step_size) {
        for (double value_j = (joint_j + joint_min); value_j <= (joint_j + joint_max); value_j += step_size) {
            for (double value_k = (joint_k + joint_min); value_k <= (joint_k + joint_max); value_k += step_size) {
                ++i;
                Nbv nbv;

                std::vector<double> candidate_joint_values = joint_values;
                candidate_joint_values[3] = value_i; 
                candidate_joint_values[4] = value_j;
                candidate_joint_values[5] = value_k;

                std::this_thread::sleep_for(std::chrono::milliseconds(10));

                auto result = plan(candidate_joint_values);
                if (result) {
                    nbv.plan = *result;
                    nbv.pose = forward_kinematics(candidate_joint_values);
                    //nbv.ray_view = getRayView(nbv); //maybe dont needed here, does it already later. 


                    nbv_candidates_.nbv_candidates.push_back(nbv);
                }
                std::cout << " Plans in local planner" << std::endl;
            }
        }
    }
    log_.attempts = i;
}




RayView ExplorationPlannerV3::getRayView(Nbv & nbv ){

    Eigen::Isometry3d& sensor_pose = nbv.pose;
    RayView ray_view = calculateRayView(sensor_pose, octo_map_);
    
    return ray_view;

}