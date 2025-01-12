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
#include "automatic_cell_explorer/exploration_planner/exploration_planners/exploration_planner_ACEP_global.hpp"

void ExplorationPlannerV4::calculateNbvCandidates(NbvCandidates & memory) {
    /*
       memory: serves as the candidates archive, keeping all candidates with non zero utility.
       Replans
    */
    log_ = EpLog{};
    nbv_candidates_.clear();
    
    auto s_eval = std::chrono::high_resolution_clock::now();
    // pass over arcive candididates
    if(!memory.empty()){
        std::cout << " archive candidates has size : " << memory.size() << std::endl;
        evaluateNbvCandidates(memory);
        removeZeroGain(memory);
        std::cout << " After removing zero gain candidates : " << memory.size() << std::endl;
        nbv_candidates_ = memory; 

        if(!nbv_candidates_.empty()){
            std::cout << " candidates has size : " << nbv_candidates_.size() << std::endl;
            filterInvalidPlans(nbv_candidates_);
            std::cout << "After filtering valid paths: : " << nbv_candidates_.size() << std::endl;
        }
        
    }



    auto e_eval = std::chrono::high_resolution_clock::now();
    
    auto s_gen = std::chrono::high_resolution_clock::now();
    generateCandidates(memory);
    auto e_gen = std::chrono::high_resolution_clock::now();

    log_.n_candidates = memory.size();

    auto d_eval = std::chrono::duration_cast<std::chrono::milliseconds>(e_eval - s_eval).count();
    auto d_gen = std::chrono::duration_cast<std::chrono::milliseconds>(e_gen - s_gen).count();
    log_.evaluate_t = d_eval; 
    log_.generate_t = d_gen; 
    
    


}


Nbv ExplorationPlannerV4::selectNbv(NbvCandidates & memory){
    /*
        Select the next best view, and remove it from the memory candidates. 
    */

    if (nbv_candidates_.empty()) {
        throw std::runtime_error("The nbv_candidates vector is empty");
        std::cout << " No more candidates, should not happen. Generate new candidates" << std::endl;
    }

    auto s = std::chrono::high_resolution_clock::now();

    Nbv nbv = Nbv();
    double best_ratio = -1;

    for (const auto& candidate : nbv_candidates_) {
        if (candidate.cost == 0) {
            std::cerr << "Should not have zero cost at this point. Candidate with zero cost found. Skipping." << std::endl;
            continue; // Skip this candidate
        }

        double ratio = candidate.gain/candidate.cost;

        if (ratio > best_ratio) {
            best_ratio = ratio;
            nbv = candidate;
        }
    }

    std::cout << "Before removing candidate: : " << memory.size() << std::endl;
    removeNbvFromCandidates(memory, nbv);
    std::cout << "After removing candidate: : " << memory.size() << std::endl;
    auto e = std::chrono::high_resolution_clock::now();
    auto d = std::chrono::duration_cast<std::chrono::microseconds>(e - s).count();
    
    std::cout << "Cost of Nbv is: " << best_ratio << std::endl;
    log_.select_t = d;
    log_.est_gain = static_cast<double>(nbv.gain);
    log_.utility_score = best_ratio;

    return nbv;
    
   
}

void ExplorationPlannerV4::evaluateNbvCandidates(NbvCandidates & nbv_candidates){
    /*
        Evaluates the candidates with rycasting. 
    */

    
    std::cout << "updating ray view " << std::endl;

    double total_time = 0.0;

    if (nbv_candidates.empty()) {
        throw std::runtime_error("The nbv_candidates vector is empty");
    }

    for (Nbv &nbv : nbv_candidates) {
        auto start_time = std::chrono::high_resolution_clock::now();

        RayView ray_view = calculateRayView(nbv.pose, octo_map_);
        nbv.gain = ray_view.num_unknowns;

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        total_time += duration;
  
    }

    int n_candidates = nbv_candidates.size();
    double average_time = (n_candidates > 0) ? (total_time / n_candidates) : 0.0;
    log_.evaluate_av_t = average_time;


    std::cout << "Number of unknowns hit by each view candidate: " << std::endl;
    for(Nbv nbv: nbv_candidates){
        std::cout << nbv.cost << std::endl;
        std::cout << nbv.gain << std::endl;
    }

}





void ExplorationPlannerV4::generateCandidates(NbvCandidates & nbv_candidates)
/*
    Noraml distribution random generate candidates, aim at a cluster center. Append if plan exists. 
*/
{
    clusters_.clear();

    auto start_time = std::chrono::high_resolution_clock::now();
    clusters_ = computeClusters(octo_map_);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    log_.cluster_t = duration;

    WorkspaceBounds bounds = WORK_SPACE;
    OrigoOffset origo = ORIGO;

    double r_min = 0.0, r_max = 0.85; 
    double phi_min = 0.0, phi_max = 2 * M_PI;
    double theta_min = 0.0, theta_max = 3*M_PI/4;


    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_r(r_min, r_max);
    std::uniform_real_distribution<> dis_phi(phi_min, phi_max);
    std::uniform_real_distribution<> dis_theta(theta_min, theta_max);


    int i = 0;
    int max_attempts = 500;
    int N = 10;
    while(nbv_candidates_.size() < N && (i < max_attempts)){ 
        ++i;
        
        std::cout << " Attempt number: " << i << std::endl;
        std::cout << " Candidates size: " << nbv_candidates_.size() << std::endl;
        Nbv nbv;

        if (i > 1000) {  // Arbitrary large number
            std::cerr << "Error: Exceeded debug attempt threshold. Exiting loop." << std::endl;
            break;
        }

        double r = dis_r(gen);
        double phi = dis_phi(gen);
        double theta = dis_theta(gen);
        double x = r * sin(theta) * cos(phi);
        double y = r * sin(theta) * sin(phi);
        double z = r * cos(theta) + origo.z;

        octomap::OcTreeNode* node = octo_map_->search(x, y, z); 

        if (!node || octo_map_->isNodeOccupied(node)) {
            continue;
        }

        nbv.pose = Eigen::Isometry3d::Identity();
        nbv.pose.translate(Eigen::Vector3d(x, y, z)); 

        if (clusters_.empty()) {
            std::cerr << "No clusters available for candidate generation." << std::endl;
            break;
        }
       
        for(Cluster & cluster : clusters_){
            if(nbv_candidates.size() >= N){
                continue;
            }
            ++i;
          
            Eigen::Vector3d position(x, y, z);
            Eigen::Vector3d target_pos(cluster.target.x(), cluster.target.y(), cluster.target.z());

            Eigen::Vector3d direction_to_target = (target_pos - position).normalized();
            Eigen::Vector3d x_axis = direction_to_target;

            // Define the global up vector as the approximate z-axis
            Eigen::Vector3d global_up(0, 0, 1);
            Eigen::Vector3d z_axis = (global_up - global_up.dot(x_axis) * x_axis).normalized();
            Eigen::Vector3d y_axis = z_axis.cross(x_axis);
            Eigen::Matrix3d rotation_matrix;
            rotation_matrix.col(0) = x_axis;  
            rotation_matrix.col(1) = y_axis;  
            rotation_matrix.col(2) = z_axis;  
            nbv.pose.linear() = rotation_matrix;

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            auto result = plan(nbv.pose);
            if (result) {
                RayView ray_view = calculateRayView(nbv.pose, octo_map_);
                double gain = ray_view.num_unknowns;
                Plan valid_plan = *result;  
                nbv.plan = valid_plan;
                nbv.gain = gain;
                nbv.cost = compute_traj_lenght(valid_plan);
                nbv_candidates_.push_back(nbv);
                nbv_candidates.push_back(nbv);

            }
            
        }
        
    }
    
    log_.attempts = i;
    std::cout << " Number of itertions to generate candidates was: " << i << std::endl;

}

