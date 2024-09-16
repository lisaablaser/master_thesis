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

    // Set Robot State
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);

    // Load Planning Scene
    const moveit::core::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(PLANNING_GROUP);

    // Error: alloc_consolidate(): unaligned fastbin chunk detected, but runs
    //planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);


    // // Configure a valid robot state
    //planning_scene_->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");


    // Create the planner instance
    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;

   
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    
    // Core dumped, but runs
    // planner_instance_.reset(planner_plugin_loader->createUnmanagedInstance("ompl_interface/OMPLPlanner")); 
    // RCLCPP_INFO(node_->get_logger(), "Using planning interface '%s'", planner_instance_->getDescription().c_str());


    std::cout << "Initialized Explortion Planner sucessfully " << std::endl;
}



std::pair<int, std::vector<RayInfo>> ExplorationPlanner::simulateInformationGain(
    const Eigen::Isometry3d& sensor_state, 
    octomap::OcTree& octree, 
    double max_range = 100.0) 
{

    const double horizontal_fov = 64.0 * M_PI / 180.0; 
    const double vertical_fov = 32.0 * M_PI / 180.0;   
    const int horizontal_rays = 100;                    // Number of rays horizontally
    const int vertical_rays = 50;                       // Number of rays vertically
    
    double horizontal_step = horizontal_fov / horizontal_rays; 
    double vertical_step = vertical_fov / vertical_rays;       


    Eigen::Vector3d sensor_origin = sensor_state.translation();

    std::vector<RayInfo> rays;
    
    int information_gain = 0;

    for (int i = 0; i < horizontal_rays; ++i) {
        for (int j = 0; j < vertical_rays; ++j) {
            
            // Compute the angles for the current ray
            double horizontal_angle = (i - horizontal_rays / 2) * horizontal_step;
            double vertical_angle = (j - vertical_rays / 2) * vertical_step;

            // Create the direction vector in the camera frame (assuming X-forward, Y-right, Z-down)
            // TODO: Debug when robot state is actually updated.. 
            Eigen::Vector3d ray_direction_camera(
                std::cos(vertical_angle) * std::cos(horizontal_angle), 
                std::cos(vertical_angle) * std::sin(horizontal_angle),  
                std::sin(vertical_angle)                              
            );

            Eigen::Vector3d ray_direction_world = sensor_state.rotation() * ray_direction_camera;

            // Define the endpoint of the ray 
            Eigen::Vector3d ray_end;
            bool hit_unknown = false;
            
            octomap::point3d hit_point;
            
            if (octree.castRay(octomap::point3d(sensor_origin.x(), sensor_origin.y(), sensor_origin.z()),
                               octomap::point3d(ray_direction_world.x(), ray_direction_world.y(), ray_direction_world.z()),
                               hit_point, true, max_range)) 
            {
                // Hit occupied point
                ray_end = Eigen::Vector3d(hit_point.x(), hit_point.y(), hit_point.z());
                
            }
            else {
                // Hit nothing, search if it us unknown or free
                octomap::OcTreeNode* node = octree.search(hit_point.x(), hit_point.y(), hit_point.z());
                if (node == nullptr) {
                    hit_unknown = true;
                    ray_end = Eigen::Vector3d(hit_point.x(), hit_point.y(), hit_point.z());
                    information_gain++;
                } else if (!octree.isNodeOccupied(node)) {
                    // whole ray is free
                    ray_end = Eigen::Vector3d(hit_point.x(), hit_point.y(), hit_point.z());
                    
                }
            }
            rays.push_back({sensor_origin, ray_end, hit_unknown});
        }
    }

    return {information_gain, rays};
}

std::pair<int, std::vector<RayInfo>> ExplorationPlanner::test_sim_view(octomap::OcTree& octree, 
    double max_range = 100.0){
    const Eigen::Isometry3d& sensor_state_before = robot_state_->getGlobalLinkTransform("rgbd_camera");
    std::cout << "Before update:" << std::endl;
    printTransform(sensor_state_before);
    robot_state_->setToDefaultValues();
    robot_state_->update();

    const Eigen::Isometry3d& sensor_state_after = robot_state_->getGlobalLinkTransform("rgbd_camera");
    std::cout << "After update:" << std::endl;
    printTransform(sensor_state_after);

    const Eigen::Isometry3d& sensor_state = robot_state_->getGlobalLinkTransform("rgbd_camera"); //w.r.t root link

    //modify to actually look at unknown area
    Eigen::Isometry3d modified_sensor_state = sensor_state;
    modified_sensor_state.translation().x() += 0;

    auto [information_gain, rays] = simulateInformationGain(modified_sensor_state, octree, max_range);
    
    return {information_gain, rays};
}




void ExplorationPlanner::update_states(std::shared_ptr<octomap::OcTree> octo_map){

    
}

robot_trajectory::RobotTrajectory ExplorationPlanner::calculate_nbv(){
    planning_interface::MotionPlanRequest req = generate_request();

    robot_trajectory::RobotTrajectory traj = plan(req);
    std::cout << "Counting waypoints in calculate_nbv func: " << traj.getWayPointCount() << std::endl;
    std::cout << "Calculating NBV function" << std::endl;
    // return traj;
    //robot_trajectory::RobotTrajectory trajectory = plan(req);
    
    return robot_trajectory::RobotTrajectory(robot_model_);

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
    //req.goal_constraints.push_back(pose_goal);

    req.group_name = PLANNING_GROUP;

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

