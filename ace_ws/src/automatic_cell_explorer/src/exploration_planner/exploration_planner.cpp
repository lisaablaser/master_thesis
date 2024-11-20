#include "automatic_cell_explorer/exploration_planner/exploration_planner.hpp"


std::optional<Plan> ExplorationPlanner::plan(const Eigen::Isometry3d& pose){

  planning_interface::MotionPlanRequest req;

  req.group_name = "ur_manipulator";


  mvt_interface_->setStartStateToCurrentState();
  mvt_interface_->clearPathConstraints();
  mvt_interface_->clearPoseTargets();
  mvt_interface_->setPoseTarget(pose);

  auto const [success, plan] = [&mvt_interface = mvt_interface_]{
    Plan p;
    auto const ok = static_cast<bool>(mvt_interface->plan(p));
    return std::make_pair(ok, p);
  }();

  if(success) {

    return plan;

  } else {

    return std::nullopt; 

  }

  return plan;
}
std::optional<Plan> ExplorationPlanner::plan(const std::vector<double> & joint_values){

  planning_interface::MotionPlanRequest req;

  req.group_name = "ur_manipulator";

  mvt_interface_->setStartStateToCurrentState();
  mvt_interface_->clearPathConstraints();

  mvt_interface_->clearPoseTargets();
  mvt_interface_->setJointValueTarget(joint_values);

  auto const [success, plan] = [&mvt_interface = mvt_interface_]{
    Plan p;
    auto const ok = static_cast<bool>(mvt_interface->plan(p));
    return std::make_pair(ok, p);
  }();

  if(success) {

    return plan;

  } else {

    return std::nullopt; 

  }

  return plan;
}

Eigen::Isometry3d ExplorationPlanner::forward_kinematics(std::vector<double> joint_values){

  auto robot_model = mvt_interface_->getRobotModel();
  auto joint_model_group = robot_model->getJointModelGroup("ur_manipulator");
  
  moveit::core::RobotState robot_state(robot_model);
  robot_state.setToDefaultValues();

  robot_state.setJointGroupPositions(joint_model_group, joint_values);
  robot_state.update();

  //Compute the forward kinematics
  Eigen::Isometry3d pose = robot_state.getGlobalLinkTransform("rgbd_camera"); 

  return pose;
}

bool ExplorationPlanner::isPoseValid(const Eigen::Isometry3d& pose) {
  auto robot_model = mvt_interface_->getRobotModel();
  auto joint_model_group = robot_model->getJointModelGroup("ur_manipulator");
  
  moveit::core::RobotState robot_state(robot_model);

  // Try to compute the inverse kinematics for the pose
  bool found_ik = robot_state.setFromIK(joint_model_group, pose);
  return found_ik;
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



double ExplorationPlanner::compute_traj_lenght(Plan plan) const{
  auto robot_model = mvt_interface_->getRobotModel();
  robot_trajectory::RobotTrajectory trajectory(robot_model, "ur_manipulator");
  trajectory.setRobotTrajectoryMsg(robot_model, plan.trajectory);

  auto trajectory_length = 0.0;
  for (std::size_t index = 1; index < trajectory.getWayPointCount(); ++index)
    {
      const auto& first = trajectory.getWayPoint(index - 1);
      const auto& second = trajectory.getWayPoint(index);
      trajectory_length += first.distance(second);
    }
  return trajectory_length;

}


void ExplorationPlanner::removeNbvFromCandidates(NbvCandidates& nbv_candidates, const Nbv& nbv_to_remove) {
    nbv_candidates.erase(
        std::remove_if(
            nbv_candidates.begin(),
            nbv_candidates.end(),
            [&nbv_to_remove](const Nbv& candidate) {
                return (candidate==nbv_to_remove); /// TODO:check if this worked.
            }),
        nbv_candidates.end());
}

void ExplorationPlanner::filterInvalidPlans(NbvCandidates& nbv_candidates) {
    /*
        Filters and updates the plan
    */
    nbv_candidates.erase(
        std::remove_if(nbv_candidates.begin(), nbv_candidates.end(),
                       [this](Nbv& candidate) { // Lambda captures `this` to access `plan()`
                           auto result = plan(candidate.pose); // Attempt to plan
                           if (result) {
                               candidate.plan = *result; // Store the plan if valid
                               candidate.pose = getFinalPoseFromPlan(candidate.plan); //obs, maybe exhausts mvt_interface??
                               candidate.cost = compute_traj_lenght(candidate.plan);
                               return false; // Keep this candidate
                           }
                           return true; // Remove candidates with invalid plans
                       }),
        nbv_candidates.end()
    );
}

void ExplorationPlanner::removeZeroGain(NbvCandidates& nbv_candidates) {
    /*
        Remove zero gain candiodates
    */
    nbv_candidates.erase(
        std::remove_if(nbv_candidates.begin(), nbv_candidates.end(),
                       [](const Nbv& candidate) {
                           return candidate.gain == 0; // Remove candidates with zero cost
                       }),
        nbv_candidates.end()
    );
}

Eigen::Isometry3d ExplorationPlanner::getFinalPoseFromPlan(const Plan & plan)
    {
    // Extract the final trajectory point
    const auto& trajectory = plan.trajectory;
    // Get the last point in the trajectory
    const auto& last_point = trajectory.joint_trajectory.points.back();

    // Get the current robot model and joint group
    auto robot_model = mvt_interface_->getRobotModel();
    auto joint_model_group = robot_model->getJointModelGroup("ur_manipulator");

    // Create a RobotState and update it to the last point in the trajectory
    moveit::core::RobotState robot_state(robot_model);
    robot_state.setToDefaultValues();  // Initialize to default
    robot_state.setJointGroupPositions(joint_model_group, last_point.positions);

    // Get the end-effector link name
    std::string ee_link = mvt_interface_->getEndEffectorLink();

    // Convert the final joint state to a Cartesian pose
    Eigen::Isometry3d end_effector_state = robot_state.getGlobalLinkTransform(ee_link);

    return end_effector_state;
}