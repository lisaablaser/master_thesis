#include "automatic_cell_explorer/exploration_planner.hpp"
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/math/Vector3.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematic_constraints/utils.h>
#include <pluginlib/class_loader.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include "automatic_cell_explorer/raycast.hpp"



ExplorationPlanner::ExplorationPlanner(MoveGrpPtr mvt_interface, std::shared_ptr<octomap::OcTree> octo_map)
    : 
    mvt_interface_(mvt_interface),
    octo_map_(octo_map)
{

}


RayView ExplorationPlanner::getCurrentRayView(double max_range){

    //update current state
    auto robot_state = mvt_interface_->getCurrentState();
    auto joint_positions = mvt_interface_->getCurrentJointValues();
    for(double j: joint_positions){

        std::cout << j << std::endl;
    }
    robot_state->printStatePositions();
    robot_state->update();
    const Eigen::Isometry3d& sensor_state = robot_state->getGlobalLinkTransform("rgbd_camera");

    robot_state->printStatePositions();

    //modify since robot pose update does not work
    Eigen::Isometry3d modified_sensor_state = sensor_state;
    modified_sensor_state.translation().x() += -1;
    modified_sensor_state.translation().y() += 0.5;
    modified_sensor_state.translation().z() += 1.1;

    return calculateRayView(sensor_state, octo_map_, max_range);
}



ExecuteReq ExplorationPlanner::calculate_nbv(){
    std::cout << "Calculating NBV function" << std::endl;

    geometry_msgs::msg::PoseStamped pose = generate_nvb_candidate();
    Plan p = plan(pose);

    ExecuteReq req;
    req.start_state = p.start_state;
    req.trajectory = p.trajectory;

    return req;

}

Plan ExplorationPlanner::plan(geometry_msgs::msg::PoseStamped pose){

    planning_interface::MotionPlanRequest req;

    double tolerance_pose = 0.01;
    double tolerance_angle = 0.01;

    req.group_name = "ur_manipulator";

    req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
    req.workspace_parameters.min_corner.z = -5.0;
    req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
    req.workspace_parameters.max_corner.z = 5.0;

    //update current pose??
    mvt_interface_->setPoseTarget(pose);


    auto const [success, plan] = [&mvt_interface = mvt_interface_]{
      Plan p;
      auto const ok = static_cast<bool>(mvt_interface->plan(p));
      return std::make_pair(ok, p);
    }();

    if(success) {
      std::cout << "Planning succeeded. Executing..." << std::endl;
      /// TODO: Return sucess or failure
      return plan;

    } else {
      std::cout << "Planning failed!" << std::endl;
    }

    return plan;
    
}


geometry_msgs::msg::PoseStamped ExplorationPlanner::generate_nvb_candidate()

{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = 0.4;
    pose.pose.position.y = 0.4;
    pose.pose.position.z = 1.5;
    pose.pose.orientation.w = 1.0;

    return pose;
}

double ExplorationPlanner::calculate_occupied_volume() const
{
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


