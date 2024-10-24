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
#include "automatic_cell_explorer/exploration_planner/exploration_planners/exploration_planner_v4.hpp"

void ExplorationPlannerV4::calculateNbvCandidates() {
    /*
        initial dummy random generator planner 
    */

    generateCandidates();
    evaluateNbvCandidates();

}


Nbv ExplorationPlannerV4::selectNbv(){
    /*
        Get the next Nbv candidate, Plans are alraeady calculated.
        Select the one with highes information gain. 
    */
    if (nbv_candidates_.nbv_candidates.empty()) {
        throw std::runtime_error("The nbv_candidates vector is empty");
    }

    Nbv highest_cost_nbv = nbv_candidates_.nbv_candidates.at(0);

    for (const auto& nbv : nbv_candidates_.nbv_candidates) {
        if (nbv.ray_view.num_unknowns > highest_cost_nbv.ray_view.num_unknowns) {
            highest_cost_nbv = nbv;
        }
    }
    std::cout << "Cost of Nbv is: " << highest_cost_nbv.ray_view.num_unknowns << std::endl;

    return highest_cost_nbv;
    
   
}

void ExplorationPlannerV4::evaluateNbvCandidates(){
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



void ExplorationPlannerV4::generateCandidates()
/*
    Random generate candidates. Append if plan exists. 
*/
{

    // Extract frontiers - create workcell sized frontier octomap
    OctreePtr frontier_tree = extractFrontierOctreeInBounds(octo_map_);
    // Cluster frontiers, return the frontier centers 
    // generate candidates with orientation_ z up, x = robot origo - cluster center

    // cluster center with noraml?, k-means cluster orientatin (pos of free and unknown) and point. 

    nbv_candidates_.nbv_candidates.clear();

    WorkspaceBounds bounds = WORK_SPACE;
    OrigoOffset origo = ORIGO;

    double roll_min = -M_PI, roll_max = M_PI;
    double pitch_min = -M_PI, pitch_max = M_PI;
    double yaw_min = -M_PI, yaw_max = M_PI;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(bounds.min_x, bounds.max_x);
    std::uniform_real_distribution<> dis_y(bounds.min_y, bounds.max_y);
    std::uniform_real_distribution<> dis_z(bounds.min_z+ origo.z, bounds.max_z+origo.z);
    std::uniform_real_distribution<> dis_roll(roll_min, roll_max);
    std::uniform_real_distribution<> dis_pitch(pitch_min, pitch_max);
    std::uniform_real_distribution<> dis_yaw(yaw_min, yaw_max);

    int i = 0;
    while(nbv_candidates_.nbv_candidates.size() != N_SAMPLES){
        ++i;
        std::cout << " Attempt number: " << i << std::endl;
        Nbv nbv;
        
        double x = dis_x(gen);
        double y = dis_y(gen);
        double z = dis_z(gen);

        octomap::OcTreeNode* node = octo_map_->search(x, y, z); 

        // If the voxel is unknown space or is occupied, skip to the next iteration
        if (!node || octo_map_->isNodeOccupied(node)) {
            continue;
        }
        
        Eigen::Quaternion q_r = Eigen::Quaterniond::UnitRandom();
        

        nbv.pose = Eigen::Isometry3d::Identity();
        nbv.pose.translate(Eigen::Vector3d(x, y, z)); 
        nbv.pose.rotate(q_r);


        auto result = plan(nbv.pose);
        if (result) {
            Plan valid_plan = *result;  
            nbv.plan = valid_plan;

            nbv.cost = 0.0; 
            nbv_candidates_.nbv_candidates.push_back(nbv);
        }
        
    }
    std::cout << " Number of itertions to generate candidates was: " << i << std::endl;

}




