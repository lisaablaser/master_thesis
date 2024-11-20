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
#include "automatic_cell_explorer/exploration_planner/exploration_planners/exploration_planner_v4.hpp"

void ExplorationPlannerV4::calculateNbvCandidates(NbvCandidates & nbv_candidates) {
    /*
       nbv_candidates serves as the candidates archive, keeping all non zero utility.
       nbv_candidates_ are the valid trajectory candidates from this pose. 
    */
    log_ = EpLog{};
    nbv_candidates_.clear();
    
    auto s_eval = std::chrono::high_resolution_clock::now();
    // pass over arcive candididates
    if(!nbv_candidates.empty()){
        std::cout << " archive candidates has size : " << nbv_candidates.size() << std::endl;
        evaluateNbvCandidates(nbv_candidates);
        removeZeroGain(nbv_candidates);
        std::cout << " After removing zero gain candidates : " << nbv_candidates.size() << std::endl;
        nbv_candidates_ = nbv_candidates; 

        if(!nbv_candidates_.empty()){
            std::cout << " candidates has size : " << nbv_candidates_.size() << std::endl;
            filterInvalidPlans(nbv_candidates_);
            std::cout << "After filtering valid paths: : " << nbv_candidates_.size() << std::endl;
        }
        
    }
    auto e_eval = std::chrono::high_resolution_clock::now();
    
    auto s_gen = std::chrono::high_resolution_clock::now();
    generateCandidates(nbv_candidates);
    auto e_gen = std::chrono::high_resolution_clock::now();

    log_.n_candidates = nbv_candidates.size();

    auto d_eval = std::chrono::duration_cast<std::chrono::milliseconds>(e_eval - s_eval).count();
    auto d_gen = std::chrono::duration_cast<std::chrono::milliseconds>(e_gen - s_gen).count();
    log_.evaluate_t = d_eval; 
    log_.generate_t = d_gen; 
    
    


}


Nbv ExplorationPlannerV4::selectNbv(NbvCandidates & nbv_candidates){
    /*
        Get the next Nbv candidate, Plans are alraeady calculated.
        Select the one with highes information gain. 
    */

    if (nbv_candidates_.empty()) {
        //throw std::runtime_error("The nbv_candidates vector is empty");
        std::cout << " No more candidates, should not happen. Generate new candidates" << std::endl;
        //generateCandidates(nbv_candidates);
    }

    auto s = std::chrono::high_resolution_clock::now();

    NbvCandidates paretoForntiers = findParetoFrontiers(nbv_candidates_);

    

    if (paretoForntiers.empty()) {
        std::cout << " No pareto frontiers, can this happen??. Generate new candidates" << std::endl;
        //generateCandidates(nbv_candidates);
        //paretoForntiers = findParetoFrontiers(nbv_candidates);
        //throw std::runtime_error("The nbv_candidates vector is empty");
    }

    Nbv nbv = Nbv();
    double best_ratio = -1;

    for (const auto& candidate : paretoForntiers) {
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

    /// Delete chosen frontier
    removeNbvFromCandidates(nbv_candidates, nbv);

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

    //removeZeroGain(nbv_candidates);

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
    //nbv_candidates.clear();

    // replan to prev poses. 

    auto start_time = std::chrono::high_resolution_clock::now();
    //clusters_ = computeClusters(octo_map_);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    log_.cluster_t = duration;

    WorkspaceBounds bounds = WORK_SPACE;
    OrigoOffset origo = ORIGO;

    double r_min = 0.0, r_max = 0.85; 
    double phi_min = 0.0, phi_max = 2 * M_PI;
    double theta_min = 0.0, theta_max = 3*M_PI/4;

    double roll_min = -M_PI/4, roll_max = M_PI/4;
    double pitch_min = -M_PI, pitch_max = M_PI;
    double yaw_min = -M_PI, yaw_max = M_PI;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_r(r_min, r_max);
    std::uniform_real_distribution<> dis_phi(phi_min, phi_max);
    std::uniform_real_distribution<> dis_theta(theta_min, theta_max);

    std::uniform_real_distribution<> dis_roll(roll_min, roll_max);
    std::uniform_real_distribution<> dis_pitch(pitch_min, pitch_max);
    std::uniform_real_distribution<> dis_yaw(yaw_min, yaw_max);


    int i = 0;
    int max_attempts = 100;
    int N = 10;
    while(nbv_candidates_.size() < N ){//|| !(i < max_attempts)){ //Try at least 100 times.
        ++i;
        
        std::cout << " Attempt number: " << i << std::endl;
        std::cout << " Candidates size: " << nbv_candidates_.size() << std::endl;
        Nbv nbv;


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

        Eigen::Quaternion q_r = Eigen::Quaterniond::UnitRandom();
        

        nbv.pose = Eigen::Isometry3d::Identity();
        nbv.pose.translate(Eigen::Vector3d(x, y, z)); 
        nbv.pose.rotate(q_r);

        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto result = plan(nbv.pose);
        if (result) {
            Plan valid_plan = *result;  
            nbv.plan = valid_plan;
            nbv.pose = getFinalPoseFromPlan(valid_plan);
            nbv.cost = compute_traj_lenght(valid_plan); 
            nbv.gain = calculateRayView(nbv.pose, octo_map_).num_unknowns;
            nbv_candidates_.push_back(nbv); 
            nbv_candidates.push_back(nbv); // add to archive also
        }
    

        // // Position at sampled position
        // nbv.pose = Eigen::Isometry3d::Identity();
        // nbv.pose.translate(Eigen::Vector3d(x, y, z)); 
       
        // for(Cluster & cluster : clusters_){
        //     if(nbv_candidates.size() >= N){
        //         continue;
        //     }
        //     ++i;
        //     // Calculate the direction vector towards the target
        //     Eigen::Vector3d position(x, y, z);
        //     Eigen::Vector3d target_pos(cluster.target.x(), cluster.target.y(), cluster.target.z());

        //     // 1. Calculate the x-axis direction pointing toward the target
        //     Eigen::Vector3d direction_to_target = (target_pos - position).normalized();
        //     Eigen::Vector3d x_axis = direction_to_target;

        //     // 2. Define the global up vector as the approximate z-axis
        //     Eigen::Vector3d global_up(0, 0, 1);

        //     // 3. Compute the z-axis by projecting global up onto the plane orthogonal to x_axis
        //     Eigen::Vector3d z_axis = (global_up - global_up.dot(x_axis) * x_axis).normalized();

        //     // 4. Compute the y-axis as the cross product of z_axis and x_axis
        //     Eigen::Vector3d y_axis = z_axis.cross(x_axis);

        //     // 5. Construct the rotation matrix from x, y, and z axes
        //     Eigen::Matrix3d rotation_matrix;
        //     rotation_matrix.col(0) = x_axis;  // x-axis points to the target
        //     rotation_matrix.col(1) = y_axis;  // y-axis is perpendicular to both x and z
        //     rotation_matrix.col(2) = z_axis;  // z-axis points approximately upward

        //     // 6. Apply the rotation matrix to the pose
        //     nbv.pose.linear() = rotation_matrix;

        //     std::this_thread::sleep_for(std::chrono::milliseconds(10));

        //     auto result = plan(nbv.pose);
        //     if (result) {
        //         RayView ray_view = calculateRayView(nbv.pose, octo_map_);
        //         double gain = ray_view.num_unknowns;
        //         if(gain != 0){
        //             Plan valid_plan = *result;  
        //             nbv.plan = valid_plan;
        //             nbv.gain = gain;
        //             nbv.cost = compute_traj_lenght(valid_plan);
        //             nbv_candidates.push_back(nbv);

        //         }

        //     }
            
        // }
        
    }
    /// TODO: handle if no candidates can be generated.. Save last trajectory or somthing. 
    
    log_.attempts = i;
    std::cout << " Number of itertions to generate candidates was: " << i << std::endl;

}




NbvCandidates ExplorationPlannerV4::findParetoFrontiers(const NbvCandidates & nbv_candidates){

    // First sort by cost
    NbvCandidates sorted_candidates = nbv_candidates;

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
    

    return paretoFrontiers;


}