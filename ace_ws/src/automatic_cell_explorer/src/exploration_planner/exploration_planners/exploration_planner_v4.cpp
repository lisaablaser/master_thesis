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

    
    std::cout << "updating ray view " << std::endl;
    for(Nbv &nbv: nbv_candidates_.nbv_candidates){
        Eigen::Isometry3d sensor_pose = nbv.pose;
        RayView ray_view = calculateRayView(sensor_pose, octo_map_);
        nbv.ray_view = ray_view;
    
    }

    std::cout << "Number of unknowns hit by each view candidate: " << std::endl;
    for(Nbv nbv: nbv_candidates_.nbv_candidates){
        std::cout << nbv.ray_view.num_unknowns << std::endl;
    }

}


void ExplorationPlannerV4::findTargets(){
    /*
        Targets are cluster centers for now.
        target_normals: center to robot pose. Not used.  
    */
    
    std::vector<Cluster> clusters = computeClusters(octo_map_);

    //auto current_pose = mvt_interface_->getCurrentPose();
    //Eigen::Vector3d end_effector_position(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);


    for(Cluster & cluster: clusters){
        TargetPatch new_target;
        new_target.target = cluster.center;
        new_target.target_normal = Eigen::Vector3d(0,0,1);//end_effector_position - Eigen::Vector3d(new_target.target.x(), new_target.target.y(), new_target.target.z());
        targets_.push_back(new_target);
    }

}


void ExplorationPlannerV4::generateCandidates()
/*
    Random generate candidates, aim at a cluster center. Append if plan exists. 
*/
{
    targets_.clear();
    findTargets();

    nbv_candidates_.nbv_candidates.clear();
    
    // use target pathces to guide sampling

    WorkspaceBounds bounds = WORK_SPACE;
    OrigoOffset origo = ORIGO;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(bounds.min_x, bounds.max_x);
    std::uniform_real_distribution<> dis_y(bounds.min_y, bounds.max_y);
    std::uniform_real_distribution<> dis_z(bounds.min_z+ origo.z, bounds.max_z+origo.z);


    int i = 0;
    while(nbv_candidates_.nbv_candidates.size() <= 3){
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

        // Position at sampled position
        nbv.pose = Eigen::Isometry3d::Identity();
        nbv.pose.translate(Eigen::Vector3d(x, y, z)); 
        
       
        for(TargetPatch & target : targets_){
            // Calculate the direction vector towards the target
            Eigen::Vector3d position(x, y, z);
            Eigen::Vector3d target_pos(target.target.x(), target.target.y(), target.target.z());

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

            auto result = plan(nbv.pose);
            if (result) {
                Plan valid_plan = *result;  
                nbv.plan = valid_plan;

                nbv.cost = 0.0; 
                nbv_candidates_.nbv_candidates.push_back(nbv);
            }
            
        }
        
    }
    std::cout << " Number of itertions to generate candidates was: " << i << std::endl;

}




