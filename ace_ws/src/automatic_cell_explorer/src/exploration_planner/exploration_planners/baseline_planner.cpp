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
#include "automatic_cell_explorer/exploration_planner/exploration_planners/baseline_planner.hpp"


void BaselinePlanner::calculateNbvCandidates(NbvCandidates & nbv_candidates){
    log_ = EpLog{};

    if(nbv_candidates.empty()){
        auto s_gen = std::chrono::high_resolution_clock::now();
        generateCandidates(nbv_candidates);
        auto e_gen = std::chrono::high_resolution_clock::now();
        auto d_gen = std::chrono::duration_cast<std::chrono::milliseconds>(e_gen - s_gen).count();
        log_.generate_t = d_gen; 
    }
    

    auto s_eval = std::chrono::high_resolution_clock::now();
    evaluateNbvCandidates(nbv_candidates);
    auto e_eval = std::chrono::high_resolution_clock::now();
    auto d_eval = std::chrono::duration_cast<std::chrono::milliseconds>(e_eval - s_eval).count();
    log_.evaluate_t = d_eval; 

}

Nbv BaselinePlanner::selectNbv(NbvCandidates & nbv_candidates){
    /*
        Get the next Nbv candidate, Plans are alraeady calculated.
        Select the one with highes information gain which is a pareto frontier.
    */

    if (nbv_candidates.empty()) {
        throw std::runtime_error("The nbv_candidates vector is empty");
    }

    NbvCandidates paretoForntiers = findParetoFrontiers(nbv_candidates);

    auto s = std::chrono::high_resolution_clock::now();

    if (paretoForntiers.empty()) {
        throw std::runtime_error("The nbv_candidates vector is empty");
    }

    Nbv nbv = paretoForntiers.at(0);
    double best_ratio = nbv.gain/nbv.cost;

    for (const auto& candidate : paretoForntiers) {
        if (candidate.cost == 0) {
            std::cerr << "Candidate with zero cost found. Skipping." << std::endl;
            continue; // Skip this candidate
        }

        double ratio = candidate.gain/candidate.cost;

        if (ratio > best_ratio) {
            
            best_ratio = ratio;

            nbv = candidate;
        }
    }

    /// Delete chosen frontier
    removeNbvFromCandidates(nbv_candidates, nbv);

    auto e = std::chrono::high_resolution_clock::now();
    auto d = std::chrono::duration_cast<std::chrono::milliseconds>(e - s).count();
    
    std::cout << "Cost of Nbv is: " << best_ratio << std::endl;
    log_.select_t = d;
    log_.est_gain = static_cast<double>(nbv.gain);
    log_.utility_score = best_ratio;

    return nbv;
    
   
}

void BaselinePlanner::evaluateNbvCandidates(NbvCandidates & nbv_candidates){
    /*
        Evaluates the candidates calculate G(). 
    */

    
    std::cout << "Evaluating candidates " << std::endl;

    double total_time = 0.0;

    if (nbv_candidates.empty()) {
        throw std::runtime_error("The nbv_candidates vector is empty");
    }

    for (Nbv &nbv : nbv_candidates) {
        Eigen::Isometry3d sensor_pose = nbv.pose;

        auto start_time = std::chrono::high_resolution_clock::now();

        RayView ray_view = calculateRayView(sensor_pose, octo_map_);
        nbv.gain = ray_view.num_unknowns;

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        total_time += duration;
  
    }

    //TODO: remove the ones with zero utility
    removeZeroGain(nbv_candidates);

    int n_candidates = nbv_candidates.size();
    double average_time = (n_candidates > 0) ? (total_time / n_candidates) : 0.0;

    log_.evaluate_av_t = average_time;


    std::cout << "Evaluation of candidates: " << std::endl;
    for(Nbv nbv: nbv_candidates){
        std::cout << "Gain: "<< nbv.gain << std::endl;
        std::cout  << "Cost: "<< nbv.cost << std::endl;
    }

}

NbvCandidates BaselinePlanner::findParetoFrontiers(const NbvCandidates & nbv_candidates){

    // First sort by cost
    NbvCandidates sorted_candidates = nbv_candidates;
    filterInvalidPlans(sorted_candidates);

    
    
    std::sort(sorted_candidates.begin(), sorted_candidates.end(),
              [](const Nbv& a, const Nbv& b) {
                  return a.cost < b.cost;
              });

    std::cout << "Sorted candidates after cost: " << std::endl;
    for(Nbv nbv: sorted_candidates){
        std::cout << "Gain: "<< nbv.gain << std::endl;
        std::cout  << "Cost: "<< nbv.cost << std::endl;
    }
   
    // Then check gein
    NbvCandidates paretoFrontiers; 
    double maxGain = -1.0; // Initialize maxGain as a very low value
    for (const auto& candidate : sorted_candidates) {
        if (candidate.gain > maxGain) {
            paretoFrontiers.push_back(candidate); // Keep this candidate
            maxGain = candidate.gain;  // Update maxGain
        }
    }

    std::cout << "Pareto frontiers: " << std::endl;
    for(Nbv nbv: paretoFrontiers){
        std::cout << "Gain: "<< nbv.gain << std::endl;
        std::cout  << "Cost: "<< nbv.cost << std::endl;
    }
    
    log_.n_candidates = sorted_candidates.size(); // Log only the valid views for this round

    return paretoFrontiers;


}


NbvCandidates BaselinePlanner::sortCandidates(const NbvCandidates & nbv_candidates){
    /*
        Sort candidates by cost
    */
    NbvCandidates sorted_candidates = nbv_candidates;
    std::sort(sorted_candidates.begin(), sorted_candidates.end(),
              [](const Nbv& a, const Nbv& b) {
                  return a.cost < b.cost;
              });
    return sorted_candidates;
}





void BaselinePlanner::generateCandidates(NbvCandidates & nbv_candidates)
/*
    Modifies external nbv_candidates 
*/
{
    nbv_candidates.clear();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_j(-M_PI, M_PI);

    int Ns = 3;
    int n_joints = 6;
    std::vector<std::vector<double>>  samle_angles(n_joints, std::vector<double>(Ns, 0.0));
    
    for (size_t i = 0; i < n_joints; ++i) {
        for (size_t j = 0; j < Ns; ++j) { 

            samle_angles[i][j] = dis_j(gen);
        }
        std::cout << "Sample angles generated " <<std::endl;
    }

    int i = 0;
    for (double a : samle_angles[0]) {
        for (double b : samle_angles[1]) {
            for (double c : samle_angles[2]) {
                for (double d : samle_angles[3]) {
                    for (double e : samle_angles[4]) {
                        for (double f : samle_angles[5]) {
                            ++i;
                            Nbv nbv;
                            std::vector<double> q = {a,b,c,d,e,f};
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                            auto result = plan(q);
                            if (result) {
                                nbv.plan = *result;
                                nbv.pose = getFinalPoseFromPlan(nbv.plan);//forward_kinematics(q);
                                nbv.cost = compute_traj_lenght(nbv.plan);
                                nbv_candidates.push_back(nbv);

                            }
                        }
                    }
                    std::cout << " Iteration nr: " << i << std::endl;
                    std::cout << " Current candidates: " << nbv_candidates.size() << std::endl;
                }
            }
        }
    }

    log_.attempts = i;
    std::cout << " Number of itertions to generate candidates was: " << i << std::endl;

}






