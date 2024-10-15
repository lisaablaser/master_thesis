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
#include "automatic_cell_explorer/exploration_planner/random_exploration_planner_v2.hpp"

void RandomExplorationPlannerV2::calculateNbvCandidates() {
    /*
        initial dummy random generator planner (not v2..)
    */

    generateCandidates();
    evaluateNbvCandidates();

}


Nbv RandomExplorationPlannerV2::selectNbv(){
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
bool RandomExplorationPlannerV2::terminationCriteria() const {
    /*
        Termination Criteria: When nbv_candidates are empty.
    */
    return (nbv_candidates_.nbv_candidates.empty());
}

void RandomExplorationPlannerV2::evaluateNbvCandidates(){
    /*
        Evaluates the candidates with rycasting. 
    */
    updateRayViews(nbv_candidates_,octo_map_);

    std::cout << "Number of unknowns hit by each view candidate: " << std::endl;
    for(Nbv nbv: nbv_candidates_.nbv_candidates){
        std::cout << nbv.ray_view.num_unknowns << std::endl;
    }

}



void RandomExplorationPlannerV2::generateCandidates()
/*
    Random generate candidates. Append if plan exists. 
*/
{
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

    while(nbv_candidates_.nbv_candidates.size() != N_SAMPLES){

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

    // while(nbv_candidates_.nbv_candidates.size() != N_SAMPLES){
    //     // std::vector<double> random_joint_values;
    //     auto robot_model = mvt_interface_->getRobotModel();
    //     auto joint_model_group = robot_model->getJointModelGroup("ur_manipulator");
    //     auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);

    //     // robot_state->setToRandomPositions(joint_model_group);
    //     // robot_state->copyJointGroupPositions(joint_model_group, random_joint_values);

    //     //planning_scene_monitor::LockedPlanningSceneRO ls(plm_interface_);
    //     //auto pc = ls->getWorld();

    //     Nbv nbv;

    //     //Much slower 
    //     mvt_interface_->setRandomTarget();
        
    //     std::vector<double> target_joint_values;
    //     mvt_interface_->getJointValueTarget(target_joint_values);

    //     // Set the robot state to these target joint values
    //     robot_state->setJointGroupPositions(joint_model_group, target_joint_values);



    //     const Eigen::Isometry3d& target_pose_eigen = robot_state->getGlobalLinkTransform("rgbd_camera");
    //     Plan plan_p;  
    //     auto result = mvt_interface_->plan(plan_p);

    //     if (result) {
    //         nbv.pose = target_pose_eigen;
    //         nbv.plan = plan_p;
    //         nbv.cost = 0.0; 
    //         nbv_candidates_.nbv_candidates.push_back(nbv);
    //     }
        
    // }
    
}




