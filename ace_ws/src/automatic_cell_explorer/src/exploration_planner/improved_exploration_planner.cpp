#include <random>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/math/Vector3.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pluginlib/class_loader.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "automatic_cell_explorer/constants.hpp"
#include "automatic_cell_explorer/exploration_planner/evaluate_nbv.hpp"
#include "automatic_cell_explorer/exploration_planner/improved_exploration_planner.hpp"

void ImprovedExplorationPlanner::calculateNbvCandidates() {
    /*
        Improved Planner. 
    */

    generateCandidates();
    evaluateNbvCandidates();

}


Nbv ImprovedExplorationPlanner::selectNbv(){
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

    return highest_cost_nbv; //nbv is not deleted from the list, but does not matter if we regenerte the list every time. 
    
   
}
bool ImprovedExplorationPlanner::terminationCriteria() const {
    /*
        Termination Criteria from Improved planner: When nbv_candidates are empty.
    */
    return (nbv_candidates_.nbv_candidates.empty());
}

void ImprovedExplorationPlanner::evaluateNbvCandidates(){
    /*
        Evaluates the candidates with rycasting. 
    */
    updateRayViews(nbv_candidates_,octo_map_);

    std::cout << "Number of unknowns hit by each view candidate: " << std::endl;
    for(Nbv nbv: nbv_candidates_.nbv_candidates){
        std::cout << nbv.ray_view.num_unknowns << std::endl;
    }

}


void ImprovedExplorationPlanner::generateCandidates()
/*
    Random generate candidates. Append if plan exists. 
*/
{
    size_t N = 10;
    double offset = 0.72;
    nbv_candidates_.nbv_candidates.clear();

    double x_min = -1.0, x_max = 1.0;
    double y_min = -1.0, y_max = 1.0;
    double z_min = offset + 0.0, z_max = offset + 1.0;

    double roll_min = -M_PI, roll_max = M_PI;
    double pitch_min = -M_PI_2, pitch_max = M_PI_2;
    double yaw_min = -M_PI, yaw_max = M_PI;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(x_min, x_max);
    std::uniform_real_distribution<> dis_y(y_min, y_max);
    std::uniform_real_distribution<> dis_z(z_min, z_max);
    std::uniform_real_distribution<> dis_roll(roll_min, roll_max);
    std::uniform_real_distribution<> dis_pitch(pitch_min, pitch_max);
    std::uniform_real_distribution<> dis_yaw(yaw_min, yaw_max);

    while(nbv_candidates_.nbv_candidates.size() != N){

        Nbv nbv;
        
        // Generate random position (translation)
        double x = dis_x(gen);
        double y = dis_y(gen);
        double z = dis_z(gen);
        
        // Generate random orientation (rotation using Euler angles)
        double roll = dis_roll(gen);
        double pitch = dis_pitch(gen);
        double yaw = dis_yaw(gen);
        
        // Convert Euler angles to rotation matrix
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle; 

        nbv.pose = Eigen::Isometry3d::Identity();
        nbv.pose.translate(Eigen::Vector3d(x, y, z)); 
        nbv.pose.rotate(q);


        auto result = plan(nbv.pose);
        if (result) {
            Plan valid_plan = *result;  
            nbv.plan = valid_plan;

            nbv.cost = 0.0; 
            nbv_candidates_.nbv_candidates.push_back(nbv);
        }
        
    }
    
}




