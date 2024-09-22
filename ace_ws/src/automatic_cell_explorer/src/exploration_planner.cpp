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



robot_trajectory::RobotTrajectory ExplorationPlanner::calculate_nbv(){
    planning_interface::MotionPlanRequest req = generate_nvb_candidate();

    robot_trajectory::RobotTrajectory traj = plan(req);
    std::cout << "Counting waypoints in calculate_nbv func: " << traj.getWayPointCount() << std::endl;
    std::cout << "Calculating NBV function" << std::endl;
    // return traj;
    //robot_trajectory::RobotTrajectory trajectory = plan(req);
    
    return robot_trajectory::RobotTrajectory(mvt_interface_->getRobotModel());

}

robot_trajectory::RobotTrajectory ExplorationPlanner::plan(planning_interface::MotionPlanRequest req){

    planning_interface::MotionPlanResponse res;

    // Corde dumped, crashes
    // planning_interface::PlanningContextPtr context =
    // planner_instance_->getPlanningContext(planning_scene_, req, res.error_code);


    // context->solve(res);

    // if (res.error_code.val != res.error_code.SUCCESS)
    // {
    //     RCLCPP_ERROR(node_->get_logger(), "Could not compute plan successfully");
    //     return robot_trajectory::RobotTrajectory(robot_model_); 
  
    // }

    // if (!res.trajectory) {
    //     RCLCPP_ERROR(node_->get_logger(), "No valid trajectory found");
    //     return robot_trajectory::RobotTrajectory(robot_model_); 
    // }

   
    // return robot_trajectory::RobotTrajectory(*res.trajectory); 
    return robot_trajectory::RobotTrajectory(mvt_interface_->getRobotModel());
    
}


planning_interface::MotionPlanRequest ExplorationPlanner::generate_nvb_candidate()

{
    planning_interface::MotionPlanRequest req;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = 0.09;
    pose.pose.position.y = 0.22;
    pose.pose.position.z = 1.41;
    pose.pose.orientation.w = 1.0;

    double tolerance_pose = 0.01;
    double tolerance_angle = 0.01;

    req.group_name = "ur_manipulator";

    req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
    req.workspace_parameters.min_corner.z = -5.0;
    req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
    req.workspace_parameters.max_corner.z = 5.0;
   
    return req;
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


