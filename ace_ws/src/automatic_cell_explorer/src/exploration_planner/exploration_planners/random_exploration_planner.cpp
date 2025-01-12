#include <random>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/math/Vector3.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pluginlib/class_loader.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "automatic_cell_explorer/constants.hpp"
#include "automatic_cell_explorer/exploration_planner/nbv.hpp"
#include "automatic_cell_explorer/exploration_planner/raycast.hpp"
#include "automatic_cell_explorer/exploration_planner/exploration_planners/random_exploration_planner.hpp"

void RandomExplorationPlanner::calculateNbvCandidates() {

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


Nbv RandomExplorationPlanner::selectNbv(){
    /*
        Get the next Nbv candidate, Plans are alraeady calculated.
        Select the one with highes information gain. 
    */
    if (nbv_candidates_.empty()) {
        throw std::runtime_error("The nbv_candidates vector is empty");
    }

    auto s = std::chrono::high_resolution_clock::now();


    Nbv nbv = Nbv();
    double best_ratio = -1000;

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

    std::cout << "Cost of Nbv is: " << nbv.gain << std::endl;

    auto e = std::chrono::high_resolution_clock::now();
    auto d = std::chrono::duration_cast<std::chrono::milliseconds>(e - s).count();
    
    log_.select_t = d;
    log_.est_gain = static_cast<double>(nbv.gain);
    log_.utility_score = best_ratio;

    return nbv;
    
   
}

void RandomExplorationPlanner::evaluateNbvCandidates(){
    /*
        Evaluates the candidates with rycasting. 
    */

    
    std::cout << "updating ray view " << std::endl;

    double total_time = 0.0;

    for(Nbv &nbv: nbv_candidates_){
        Eigen::Isometry3d sensor_pose = nbv.pose;

        auto start_time = std::chrono::high_resolution_clock::now();

        RayView ray_view = calculateRayView(sensor_pose, octo_map_);
        nbv.ray_view = ray_view;
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


void RandomExplorationPlanner::generateCandidatesJointSpace(){

    nbv_candidates_.clear();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_j(-M_PI, M_PI);


    int i = 0;
    while(nbv_candidates_.size() != N_SAMPLES){
        ++i;
        double a = dis_j(gen);
        double b = dis_j(gen);
        double c = dis_j(gen);
        double d = dis_j(gen);
        double e = dis_j(gen);
        double f = dis_j(gen);


        Nbv nbv;
        std::vector<double> q = {a,b,c,d,e,f};
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto result = plan(q);
        if (result) {
            nbv.plan = *result;
            nbv.pose = getFinalPoseFromPlan(nbv.plan);//forward_kinematics(q);
            nbv.cost = compute_traj_lenght(nbv.plan);
            nbv_candidates_.push_back(nbv);

        }
    
    }
    log_.attempts = i;
    std::cout << " Number of itertions to generate candidates was: " << i << std::endl;
}


void RandomExplorationPlanner::generateCandidates()
/*
    Random generate candidates. Append if plan exists. 
*/
{
    nbv_candidates_.clear();

    WorkspaceBounds bounds = WORK_SPACE;
    OrigoOffset origo = ORIGO;

    double r_min = 0.0, r_max = 0.85; 
    double phi_min = 0.0, phi_max = 2 * M_PI;
    double theta_min = 0.0, theta_max = 3*M_PI/4;

    double roll_min = -M_PI, roll_max = M_PI;
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

    int i =0;
    while(nbv_candidates_.size() != N_SAMPLES){
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

        // If the voxel is unknown space or is occupied, skip to the next iteration
        if (!node || octo_map_->isNodeOccupied(node)) {
            std::cout << "Node occupied " << std::endl;
            continue;
        }
        
        Eigen::Quaternion q_r = Eigen::Quaterniond::UnitRandom();
        

        nbv.pose = Eigen::Isometry3d::Identity();
        nbv.pose.translate(Eigen::Vector3d(x, y, z)); 
        nbv.pose.rotate(q_r);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto result = plan(nbv.pose);
        if (result) {
            Plan valid_plan = *result;  
            nbv.plan = valid_plan;
            nbv.pose = getFinalPoseFromPlan(nbv.plan);
            nbv.cost = compute_traj_lenght(valid_plan); 
            nbv_candidates_.push_back(nbv);
        }
    
    
    }
    log_.attempts = i;
    std::cout << " Number of itertions to generate candidates was: " << i << std::endl;

}


void RandomExplorationPlanner::generateCandidatesTarget()
/*
    Noraml distribution random generate candidates, aim at a cluster center. Append if plan exists. 
*/
{
    nbv_candidates_.clear();
    clusters_.clear();

    auto start_time = std::chrono::high_resolution_clock::now();
    clusters_ = computeClusters(octo_map_);

    for(Cluster & cluster : clusters_){
        std::cout << "points in cluster " << cluster.points.size() << std::endl;
        std::cout << "Frontiers in cluster " << cluster.frontiers.size() << std::endl;
    }


    std::cout<< "Number of clusters: " << clusters_.size() << std::endl;
    removeClustersWhithoutFrontiers(clusters_);
    std::cout<< "Number of clusters: " << clusters_.size() << std::endl;

    if( clusters_.size() == 0){
        std::cout << "Terminate, no more clusters that can be seen " << std::endl;
        return;
    }

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
    int max_attempts = 100;
    int N = 10;
    while(nbv_candidates_.size() < N ){//|| !(i < max_attempts)){ //Try at least 100 times.
        
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
            std::cout << "Node occupied, skipping" << std::endl;
            ++i;
            continue;
        }

        nbv.pose = Eigen::Isometry3d::Identity();
        nbv.pose.translate(Eigen::Vector3d(x, y, z)); 
       
        for(Cluster & cluster : clusters_){
            if(nbv_candidates_.size() >= N){
                std::cout << "Candidate size is large enough. " << std::endl;
                continue;
            }
            ++i;
            
            Eigen::Vector3d position(x, y, z);
            Eigen::Vector3d target_pos(cluster.center.x(), cluster.center.y(), cluster.center.z());

            Eigen::Vector3d direction_to_target = (target_pos - position).normalized();
            Eigen::Vector3d x_axis = direction_to_target;
            
            Eigen::Vector3d global_up(0, 0, 1);
            
            Eigen::Vector3d z_axis = (global_up - global_up.dot(x_axis) * x_axis).normalized();

            Eigen::Vector3d y_axis = z_axis.cross(x_axis);

            Eigen::Matrix3d rotation_matrix;
            rotation_matrix.col(0) = x_axis;  // x-axis points to the target
            rotation_matrix.col(1) = y_axis;  // y-axis is perpendicular to both x and z
            rotation_matrix.col(2) = z_axis;  // z-axis points approximately upward

            nbv.pose.linear() = rotation_matrix;

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            auto result = plan(nbv.pose);
            if (result) {
                Plan valid_plan = *result;  
                nbv.plan = valid_plan;
                nbv.pose = getFinalPoseFromPlan(valid_plan);
                RayView ray_view = calculateRayView(nbv.pose, octo_map_);
                double gain = ray_view.num_unknowns;
                //if(gain != 0){
                    
                    nbv.gain = gain;
                    nbv.cost = compute_traj_lenght(valid_plan);
                    nbv_candidates_.push_back(nbv);

                //}

            }
            
        }
        
    }
    
    log_.attempts = i;
    std::cout << " Number of itertions to generate candidates was: " << i << std::endl;

}

