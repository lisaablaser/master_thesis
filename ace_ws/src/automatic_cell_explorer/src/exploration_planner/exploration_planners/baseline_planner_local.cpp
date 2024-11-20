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
#include "automatic_cell_explorer/clustering.hpp"
#include "automatic_cell_explorer/exploration_planner/nbv.hpp"
#include "automatic_cell_explorer/exploration_planner/raycast.hpp"
#include "automatic_cell_explorer/exploration_planner/exploration_planners/baseline_planner_local.hpp"

void BaselinePlannerLocal::calculateNbvCandidates() {
    /*
        Generate Neighbour Poses Local planner implemented from paper for comparison. 
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


Nbv BaselinePlannerLocal::selectNbv(){
    /*
        Get the next Nbv candidate, Plans are alraeady calculated.
        Select the one with highes information gain. 
    */

    if (nbv_candidates_.empty()) {
        return Nbv();
    }
    
    auto s = std::chrono::high_resolution_clock::now();

    Nbv nbv = Nbv();
    double best_utility = -1;

    for (const auto& candidate : nbv_candidates_) {
        double utility = candidate.gain;

        if (utility > best_utility) {
            
            best_utility = utility;

            nbv = candidate;
        }
    }

    auto e = std::chrono::high_resolution_clock::now();
    auto d = std::chrono::duration_cast<std::chrono::milliseconds>(e - s).count();
    
    std::cout << "Cost of Nbv is: " << best_utility << std::endl;
    log_.select_t = d;
    log_.est_gain = static_cast<double>(nbv.gain);
    log_.utility_score = best_utility;

    return nbv;
    
   
}

void BaselinePlannerLocal::evaluateNbvCandidates(){
    /*
        Evaluates the candidates with rycasting. 
    */

    
    std::cout << "updating ray view " << std::endl;

    double total_time = 0.0;

    for (Nbv &nbv : nbv_candidates_) {

        auto start_time = std::chrono::high_resolution_clock::now();

        RayView ray_view = calculateRayView(nbv.pose, octo_map_);
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



void BaselinePlannerLocal::generateCandidates()
/*
    Local Planner
    Generate candidates +- delta j for all joints, 2*j combinations. 
*/
{
    nbv_candidates_.clear();
    mvt_interface_->getCurrentState(); //maybe a bug fix: prev joint values where somtimes old 
    std::vector<double> q_curr = mvt_interface_->getCurrentJointValues();


    double delta_q = 10.0*(M_PI/180); 

    for(int i = 0; i < 6; ++i){
        std::vector<double> q_new = q_curr;
        q_new[i] += delta_q; 
        add_pose_if_valid(q_new);
        q_new[i] -= 2*delta_q; 
        add_pose_if_valid(q_new);
    }
    
    log_.attempts = 2*6;
    std::cout << " Number of itertions to generate candidates was: " << 2*6 << std::endl;

}

void BaselinePlannerLocal::add_pose_if_valid(std::vector<double> q){
    Nbv nbv;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    auto result = plan(q);
    if (result) {
        nbv.plan = *result;
        nbv.pose = getFinalPoseFromPlan(nbv.plan);//forward_kinematics(q);
        nbv.cost = compute_traj_lenght(nbv.plan);
        nbv_candidates_.push_back(nbv);
    }

}


