#include "automatic_cell_explorer/exploration_planner.hpp"
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/math/Vector3.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematic_constraints/utils.h>
#include <pluginlib/class_loader.hpp>

const std::string PLANNING_GROUP = "ur_manipulator";

ExplorationPlanner::ExplorationPlanner(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<octomap::OcTree> octo_map)
    : 
    node_(node),
    octo_map_(octo_map),
    robot_model_(nullptr),
    robot_state_(nullptr),
    planning_scene_(nullptr),
    planner_instance_(nullptr)
{
    // Load RobotModel
    robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");
    robot_model_ = robot_model_loader.getModel();

    if (!robot_model_) {
        throw std::runtime_error("Failed to load robot model");
    }
    std::cout << "Robot Name of the kinematic model: " << robot_model_->getName() << std::endl;


    // Create the planner instance
    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;

   
    // planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    

    // planner_instance_.reset(planner_plugin_loader->createUnmanagedInstance("ompl_interface/OMPLPlanner")); // Core dumped
    // RCLCPP_INFO(node_->get_logger(), "Using planning interface '%s'", planner_instance_->getDescription().c_str());




    std::cout << "Initialized Explortion planner sucessfully " << std::endl;
}


robot_trajectory::RobotTrajectory ExplorationPlanner::calculate_nbv(){
    planning_interface::MotionPlanRequest req = generate_request();


    // robot_trajectory::RobotTrajectory traj = plan(req);
    // std::cout << "Counting waypoints in calculate_nbv func: " << traj.getWayPointCount() << std::endl;
    // std::cout << "Calculating NBV function" << std::endl;
    // return traj;
    //robot_trajectory::RobotTrajectory trajectory = plan(req);
    
    return robot_trajectory::RobotTrajectory(robot_model_);

}

robot_trajectory::RobotTrajectory ExplorationPlanner::plan(planning_interface::MotionPlanRequest req){

    // planning_interface::MotionPlanResponse res;

    // if (!planner_instance_) {
    //     RCLCPP_ERROR(node_->get_logger(), "Planner instance is not initialized");
    //     return robot_trajectory::RobotTrajectory(robot_model_);
    // }

    // planning_interface::PlanningContextPtr context =
    // planner_instance_->getPlanningContext(planning_scene_, req, res.error_code);

    // if (!context)
    // {
    //     RCLCPP_ERROR(node_->get_logger(), "Failed to create planning context");
    //     return robot_trajectory::RobotTrajectory(robot_model_); 

    // }

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
    return robot_trajectory::RobotTrajectory(robot_model_);
    
}


planning_interface::MotionPlanRequest ExplorationPlanner::generate_request()

{
    planning_interface::MotionPlanRequest req;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = PLANNING_GROUP;
    pose.pose.position.x = 0.09;
    pose.pose.position.y = 0.22;
    pose.pose.position.z = 1.41;
    pose.pose.orientation.w = 1.0;
    

    double tolerance_pose = 0.01;
    double tolerance_angle = 0.01;
    

    //Core dumped. 
    //moveit_msgs::msg::Constraints pose_goal =
    //kinematic_constraints::constructGoalConstraints("tool0", pose, tolerance_pose, tolerance_angle);
 

    req.group_name = PLANNING_GROUP;
    //req.goal_constraints.push_back(pose_goal);

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
    // Assuming each occupied node is a cube with side length equal to the resolution
    return resolution * resolution * resolution;
}