#include <random>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/math/Vector3.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pluginlib/class_loader.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "automatic_cell_explorer/constants.hpp"
#include "automatic_cell_explorer/exploration_planner/raycast.hpp"
#include "automatic_cell_explorer/exploration_planner/exploration_planners/exploration_planner_v2.hpp"

void ExplorationPlannerV2::calculateNbvCandidates() {
    /*
        Improved random generator Planner. 
    */

    // Update and remove old poses
    update_ray_views();             // Gain updated
    remove_weak_candidates();       //
    update_trajectories();          // Traj updated
    calculateCost();                // Cost updated

    // Generate new candidates
    generateCandidates();
    //evaluateNbvCandidates();

}


Nbv ExplorationPlannerV2::selectNbv(){
    /*
        Get the next Nbv candidate, Plans are alraeady calculated.
        Select the one with highes information gain. 
    */
    if (nbv_candidates_.nbv_candidates.empty()) {
        throw std::runtime_error("The nbv_candidates vector is empty");
    }

    auto nbv = nbv_candidates_.nbv_candidates.begin();
    float highest_cost = nbv->cost;
    float highest_gain = nbv->ray_view.num_unknowns;
    float highest_utility = -INFINITY;
    //should be deleted after selected

    for (auto it = nbv_candidates_.nbv_candidates.begin(); it != nbv_candidates_.nbv_candidates.end(); ++it )
    {
        float cost = it->cost;
        float gain = it->ray_view.num_unknowns;
        // Define som waights

        float utility = gain;

        if (utility > highest_utility) {

            highest_cost = cost;
            highest_gain = gain;
            highest_utility = utility;
            nbv = it;

        }
    }
    std::cout << "Cost of Nbv is: " << highest_cost << std::endl;
    std::cout << "Gain of Nbv is: " << highest_gain << std::endl;
    std::cout << "Utility of Nbv is: " << highest_utility << std::endl;
    nbv_candidates_.nbv_candidates.erase(nbv);

    std::cout << "Nbv was removed from list, length is now: " << nbv_candidates_.nbv_candidates.size() << std::endl; 

    // could attemt to fint a shorter path:
        //maybe try to shorten the chosen NBV path 
        /** \brief Set the number of times the motion plan is to be computed from scratch before the shortest solution is
    * returned. The default value is 1.*/
    //void setNumPlanningAttempts(unsigned int num_planning_attempts);

    return *nbv;
   
}


void ExplorationPlannerV2::evaluateNbvCandidates(){
    /*
        Evaluates the candidates with rycasting. 
    */

    //update ray views should only modify ray_views
    //update_ray_views();
    //calculate nbv.utility = gain - cost

    

    std::cout << "Number of unknowns hit by each view candidate: " << std::endl;
    for(Nbv nbv: nbv_candidates_.nbv_candidates){
        std::cout << nbv.ray_view.num_unknowns << std::endl;
    }

}

void ExplorationPlannerV2::calculateCost(){
    /*
        calculate time of trajectory

    */
    for (auto it = nbv_candidates_.nbv_candidates.begin(); it != nbv_candidates_.nbv_candidates.end(); ++it )
    {
        auto traj = it->plan.trajectory.joint_trajectory;
        size_t num_points = traj.points.size();
        std::cout << "Number of points in trajectory: " << num_points << std::endl;

        double total_time = traj.points.back().time_from_start.sec;
        std::cout << "Total duration of the trajectory: " << total_time << " seconds" << std::endl;

        it->cost = total_time;

    }


}

double ExplorationPlannerV2::getCost(Nbv & nbv){
    auto traj = nbv.plan.trajectory.joint_trajectory;
    size_t num_points = traj.points.size();
    std::cout << "Number of points in trajectory: " << num_points << std::endl;

    double total_time = traj.points.back().time_from_start.sec;
    std::cout << "Total duration of the trajectory: " << total_time << " seconds" << std::endl;

    return total_time;
}

void ExplorationPlannerV2::update_ray_views(){
    /*
        updates ray_views from current map. Also updates Gain
    */

    std::cout << "updating ray views " << std::endl;
    for (auto it = nbv_candidates_.nbv_candidates.begin(); it != nbv_candidates_.nbv_candidates.end(); ++it )
    {
        it->ray_view = getRayView(*it);
    }

}
RayView ExplorationPlannerV2::getRayView(Nbv & nbv ){

    double max_range = 0.93;
    Eigen::Isometry3d& sensor_pose = nbv.pose;
    RayView ray_view = calculateRayView(sensor_pose, octo_map_, max_range);
    
    return ray_view;

}


void ExplorationPlannerV2::update_trajectories(){
    /*
        updates trajectories from new state, and removes the onec with no valid trajectory
    */

    for (auto it = nbv_candidates_.nbv_candidates.begin(); it != nbv_candidates_.nbv_candidates.end(); )
    {
        auto result = plan(it->pose);

        if (result) {
   
            Plan valid_plan = *result;
            it->plan = valid_plan;
            ++it;  
        }
        else {
            it = nbv_candidates_.nbv_candidates.erase(it);
        }
    }
}


void ExplorationPlannerV2::remove_weak_candidates(){
    /// Should update rayviews first maybe. 
    /// And gain_treshold should be dynamic. 
    int gain_trehsold = 1000;

    for (auto it = nbv_candidates_.nbv_candidates.begin(); it != nbv_candidates_.nbv_candidates.end(); )
    {
        if (it->ray_view.num_unknowns < gain_trehsold) {
   
            it = nbv_candidates_.nbv_candidates.erase(it);
            
        }
        else{
            ++it;
        }
    }

}


void ExplorationPlannerV2::generateCandidates()
/*
    Random generate candidates until we have N in total. Append if plan exists. 
*/
{   
    /// TODO: if max atempts are used, recover to home pose or somthing. 


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
            nbv.plan = *result;
            nbv.ray_view = getRayView(nbv);
            nbv.cost = getCost(nbv); 

            nbv_candidates_.nbv_candidates.push_back(nbv);
        }
        
    }
    std::cout << " Number of itertions to generate candidates was: " << i << std::endl;
    
}




