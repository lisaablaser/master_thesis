#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/math/Vector3.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematic_constraints/utils.h>
#include <pluginlib/class_loader.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "automatic_cell_explorer/exploration_planner/exploration_planner.hpp"
#include "automatic_cell_explorer/exploration_planner/evaluate_nbv.hpp"




ExplorationPlanner::ExplorationPlanner(MoveGrpPtr mvt_interface, std::shared_ptr<octomap::OcTree> octo_map)
    : 
    mvt_interface_(mvt_interface),
    octo_map_(octo_map),
    nbv_candidates_()
{

}
void ExplorationPlanner::generateCandidates(){
    generate_nvb_candidates_circle(0.5, 1.5, 10);
}

ExecuteReq ExplorationPlanner::get_nbv_demo(){
/*
    Demo Planner
*/

    /// TODO: Further process octomap as method requires. 

    if(nbv_candidates_.nbv_candidates.empty()){
        generate_nvb_candidates_circle(0.5, 1.5, 10);
    }

    Nbv nbv = popFirstNbv();

    
    auto result = plan(nbv.pose);

    if (result) {
        Plan valid_plan = *result;  
        nbv.plan = valid_plan;

        ExecuteReq req;
        req.start_state = nbv.plan.start_state;
        req.trajectory = nbv.plan.trajectory;
        return req;

    } else {
        std::cerr << "No valid plan was generated." << std::endl;
        return get_nbv_demo();
    }

}

void ExplorationPlanner::evaluateNbvCandidates(){
    updteRayViews(nbv_candidates_,octo_map_);
}


Nbv ExplorationPlanner::getNbv() const{
    /*
        Get the next Nbv candidate.
    */
     if (nbv_candidates_.nbv_candidates.empty()) {
        throw std::runtime_error("No candidates available to pop.");
    }

    Nbv first_nbv = nbv_candidates_.nbv_candidates.front();

    return first_nbv;

}

Nbv ExplorationPlanner::popFirstNbv() {
    /// TODO: move to samle_nbv.h
    /*
        Returns and deletes the nbv. 
    */

    if (nbv_candidates_.nbv_candidates.empty()) {
        throw std::runtime_error("No candidates available to pop.");
    }

    Nbv first_nbv = nbv_candidates_.nbv_candidates.front();

    nbv_candidates_.nbv_candidates.erase(nbv_candidates_.nbv_candidates.begin());

    return first_nbv;
}

std::optional<Plan> ExplorationPlanner::plan(const Eigen::Isometry3d& pose){

    planning_interface::MotionPlanRequest req;

    req.group_name = "ur_manipulator";

    req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
    req.workspace_parameters.min_corner.z = -5.0;
    req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
    req.workspace_parameters.max_corner.z = 5.0;

    mvt_interface_->setPoseTarget(pose);


    auto const [success, plan] = [&mvt_interface = mvt_interface_]{
      Plan p;
      auto const ok = static_cast<bool>(mvt_interface->plan(p));
      return std::make_pair(ok, p);
    }();

    if(success) {
      std::cout << "Planning succeeded. Executing..." << std::endl;
      return plan;

    } else {
      std::cout << "Planning failed!" << std::endl;
      return std::nullopt; 

    }

    return plan;
}



void ExplorationPlanner::generate_nvb_candidates_circle(double radius, double height, int resolution_degrees)
{
    /// TODO: move to generate_nbv.h
    nbv_candidates_.nbv_candidates.clear();
    for (int angle_deg = 0; angle_deg < 360; angle_deg += resolution_degrees)
    {
        Nbv nbv;
        
        double angle_rad = angle_deg * M_PI / 180.0;

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Vector3d(radius * std::cos(angle_rad), 
                                             radius * std::sin(angle_rad),  
                                             height);                       

        
        Eigen::AngleAxisd yaw_rotation(angle_rad, Eigen::Vector3d::UnitZ());
        pose.rotate(yaw_rotation);

        nbv.pose = pose;
        nbv.cost = 0.0; 
        nbv.plan = Plan();

        nbv_candidates_.nbv_candidates.push_back(nbv);
    }
}

double ExplorationPlanner::calculate_occupied_volume() const
{
    /// TODO: move to evaluate_nbv.h
    double volume = 0.0;
    for (auto it = octo_map_->begin_leafs(), end = octo_map_->end_leafs(); it != end; ++it) {
        if (octo_map_->isNodeOccupied(*it)) {
            volume += compute_node_volume(octo_map_->getResolution());
        }
    }
    return volume;
}

double ExplorationPlanner::compute_node_volume(double resolution) const
{
    return resolution * resolution * resolution;
}


