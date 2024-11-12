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

void ExplorationPlannerV4::calculateNbvCandidates() {
    /*
        initial dummy random generator planner 
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


Nbv ExplorationPlannerV4::selectNbv(){
    /*
        Get the next Nbv candidate, Plans are alraeady calculated.
        Select the one with highes information gain. 
    */

    if (nbv_candidates_.nbv_candidates.empty()) {
        throw std::runtime_error("The nbv_candidates vector is empty");
    }

    /// TODO: create cost() and gain() functions, or utility()
    // AND check if no errors are made here.. 
    auto s = std::chrono::high_resolution_clock::now();

    const Nbv* nbv = &nbv_candidates_.nbv_candidates.at(0);
    double gain = nbv->ray_view.num_unknowns;
    double cost = nbv->cost;
    double best_utility = - cost;

    for (const auto& candidate : nbv_candidates_.nbv_candidates) {
        double cand_gain = candidate.ray_view.num_unknowns;
        double cand_cost = candidate.cost;
        double utility = - cand_cost;

        if (utility > best_utility) {
            
            best_utility = utility;

            nbv = &candidate;
        }
    }

    auto e = std::chrono::high_resolution_clock::now();
    auto d = std::chrono::duration_cast<std::chrono::milliseconds>(e - s).count();
    
    std::cout << "Cost of Nbv is: " << best_utility << std::endl;
    log_.select_t = d;
    log_.est_gain = static_cast<double>(nbv->ray_view.num_unknowns);
    log_.utility_score = best_utility;

    return *nbv;
    
   
}

void ExplorationPlannerV4::evaluateNbvCandidates(){
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
        std::cout << nbv.cost << std::endl;
    }

}



void ExplorationPlannerV4::generateCandidates()
/*
    Noraml distribution random generate candidates, aim at a cluster center. Append if plan exists. 
*/
{
    clusters_.clear();
    nbv_candidates_.nbv_candidates.clear();

    clusters_ = computeClusters(octo_map_);

    WorkspaceBounds bounds = WORK_SPACE;
    OrigoOffset origo = ORIGO;

    /// TODO: Order clusters after closeness
    /// TODO: remove clusters with no frontiers

    /// TODO: Create distributions

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(bounds.min_x, bounds.max_x);
    std::uniform_real_distribution<> dis_y(bounds.min_y, bounds.max_y);
    std::uniform_real_distribution<> dis_z(bounds.min_z+ origo.z, bounds.max_z+origo.z);


    int i = 0;
    int max_attempts = 1000;
    while(nbv_candidates_.nbv_candidates.size() < 4 && i < max_attempts){
        
        std::cout << " Attempt number: " << i << std::endl;
        std::cout << " Candidates size: " << nbv_candidates_.nbv_candidates.size() << std::endl;
        Nbv nbv;
        
        double x = dis_x(gen);
        double y = dis_y(gen);
        double z = dis_z(gen);

        octomap::OcTreeNode* node = octo_map_->search(x, y, z); 

        if (!node || octo_map_->isNodeOccupied(node)) {
            continue;
        }

        // Position at sampled position
        nbv.pose = Eigen::Isometry3d::Identity();
        nbv.pose.translate(Eigen::Vector3d(x, y, z)); 
       
        for(Cluster & cluster : clusters_){
            ++i;
            // Calculate the direction vector towards the target
            Eigen::Vector3d position(x, y, z);
            Eigen::Vector3d target_pos(cluster.target.x(), cluster.target.y(), cluster.target.z());

            // 1. Calculate the x-axis direction pointing toward the target
            Eigen::Vector3d direction_to_target = (target_pos - position).normalized();
            Eigen::Vector3d x_axis = direction_to_target;

            // 2. Define the global up vector as the approximate z-axis
            Eigen::Vector3d global_up(0, 0, 1);

            // 3. Compute the z-axis by projecting global up onto the plane orthogonal to x_axis
            Eigen::Vector3d z_axis = (global_up - global_up.dot(x_axis) * x_axis).normalized();

            // 4. Compute the y-axis as the cross product of z_axis and x_axis
            Eigen::Vector3d y_axis = z_axis.cross(x_axis);

            // 5. Construct the rotation matrix from x, y, and z axes
            Eigen::Matrix3d rotation_matrix;
            rotation_matrix.col(0) = x_axis;  // x-axis points to the target
            rotation_matrix.col(1) = y_axis;  // y-axis is perpendicular to both x and z
            rotation_matrix.col(2) = z_axis;  // z-axis points approximately upward

            // 6. Apply the rotation matrix to the pose
            nbv.pose.linear() = rotation_matrix;

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            auto result = plan(nbv.pose);
            if (result) {
                Plan valid_plan = *result;  
                nbv.plan = valid_plan;

                nbv.cost = compute_traj_lenght(valid_plan);
                nbv_candidates_.nbv_candidates.push_back(nbv);
            }
            
        }
        
    }
    /// TODO: handle if no candidates can be generated.. Save last trajectory or somthing. 
    if(i >= max_attempts){
        nbv_candidates_.nbv_candidates.push_back(Nbv());
    }
    
    log_.attempts = i;
    std::cout << " Number of itertions to generate candidates was: " << i << std::endl;

}




