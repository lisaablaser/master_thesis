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
#include "automatic_cell_explorer/exploration_planner/exploration_planners/exploration_planner_ACEP_local.hpp"

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

    if (nbv_candidates_.empty()) {
        return Nbv();
        
    }
    auto s = std::chrono::high_resolution_clock::now();

    const Nbv* nbv = &nbv_candidates_.at(0);
    double gain = nbv->gain;
    double cost = 0;
    double best_utility = gain - cost;

    for (const auto& candidate : nbv_candidates_) {
        double cand_gain = candidate.gain;
        double cand_cost = 0;
        double utility = cand_gain- cand_cost;

        if (utility > best_utility) {
            
            best_utility = utility;

            nbv = &candidate;
        }
    }
    
    std::cout << "Cost of Nbv is: " << best_utility << std::endl;
    auto e = std::chrono::high_resolution_clock::now();
    auto d = std::chrono::duration_cast<std::chrono::microseconds>(e - s).count();
    
    log_.select_t = d;
    log_.est_gain = static_cast<double>(nbv->gain);
    log_.utility_score = best_utility;

    return *nbv;
    
   
}

void ExplorationPlannerV3::evaluateNbvCandidates(){
    /*
        Evaluates the candidates with rycasting. 
    */

    std::cout << "updating ray view " << std::endl;

    double total_time = 0.0;

    for (Nbv &nbv : nbv_candidates_) {
        Eigen::Isometry3d sensor_pose = nbv.pose;

        auto start_time = std::chrono::high_resolution_clock::now();

        RayView ray_view = calculateRayView(sensor_pose, octo_map_);
        nbv.gain = ray_view.num_unknowns;

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        total_time += duration;
  
    }

    int n_candidates = nbv_candidates_.size();
    double average_time = (n_candidates > 0) ? (total_time / n_candidates) : 0.0;
    log_.n_candidates = n_candidates;
    log_.evaluate_av_t = average_time;


    std::cout << "Number of unknowns hit by each view candidate: " << std::endl;
    for(Nbv nbv: nbv_candidates_){
        std::cout << nbv.gain << std::endl;
    }


}



void ExplorationPlannerV3::generateCandidates()
/*
    Local Planner

    Generatees candidates on a -+15 deg spherical cap around the current position on the wrist joint. 
*/
{
    nbv_candidates_.clear();
    mvt_interface_->getCurrentState(); 
    std::vector<double> joint_values = mvt_interface_->getCurrentJointValues();
    const std::vector<std::string, std::allocator<std::string>>  joint_names = mvt_interface_->getJointNames();

    
    double joint_min = -15*(M_PI/180); 
    double joint_max = 15*(M_PI/180); 
    int n_steps = 2;
    double step_size = 2*joint_max/n_steps; 

    double joint_i = joint_values[3];
    double joint_j = joint_values[4];
    double joint_k = joint_values[5];

    int i = 0;
    for (int step_i = 0; step_i <= n_steps; ++step_i) {
    double value_i = joint_i + joint_min + step_i * step_size;
        for (int step_j = 0; step_j <= n_steps; ++step_j) {
            double value_j = joint_j + joint_min + step_j * step_size;
                ++i;
                Nbv nbv;

                std::vector<double> candidate_joint_values = joint_values;
                candidate_joint_values[3] = value_i; 
                candidate_joint_values[4] = value_j;

                std::this_thread::sleep_for(std::chrono::milliseconds(10));

                auto result = plan(candidate_joint_values);
                if (result) {
                    nbv.plan = *result;
                    nbv.pose = getFinalPoseFromPlan(nbv.plan);
                    nbv.cost = compute_traj_lenght(nbv.plan);

                    nbv_candidates_.push_back(nbv);
                }
                std::cout << " Plans in local planner" << std::endl;
        }
    }
    log_.attempts = i;
}

