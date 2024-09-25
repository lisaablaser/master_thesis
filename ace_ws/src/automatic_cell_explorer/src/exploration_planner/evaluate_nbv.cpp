#include "automatic_cell_explorer/exploration_planner/evaluate_nbv.hpp"

Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose& pose);

std::vector<RayView> getAllRayViews(const NbvCandidates& nbvs,  std::shared_ptr<octomap::OcTree> octo_map){
    double max_range = 10.0;

    std::vector<RayView> ray_views;

    for(const Nbv &nbv: nbvs.nbv_candidates){
        Eigen::Isometry3d sensor_pose = poseToIsometry(nbv.pose.pose);
        RayView ray_view = calculateRayView(sensor_pose, octo_map, max_range);
        ray_views.push_back(ray_view);
    }

    return ray_views;
}


#include <Eigen/Geometry>  
#include <geometry_msgs/msg/pose_stamped.hpp> 
Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose& pose) {
    Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();

    // Set the translation (position)
    isometry.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);

    // Set the rotation (quaternion to rotation matrix)
    Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    isometry.rotate(quaternion);

    return isometry;
}