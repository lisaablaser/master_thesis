#ifndef RAYCAST_HPP
#define RAYCAST_HPP

#include <eigen3/Eigen/Geometry>


enum class NodeState {Free, Unknown, Occupied, Error};

struct RayInfo { //Ray
    
    Eigen::Vector3d start;  // Ray start (sensor origin)
    Eigen::Vector3d end;    // Ray end (hit point or max range)
    NodeState node_state;       // Whether the ray hit an unknown region
};

struct RayView{
    std::vector<RayInfo> rays; 
    int num_unknowns;
    Eigen::Isometry3d pose;
};

struct RayViews{
    std::vector<RayView> ray_views;
};



RayView calculateRayView(
    const Eigen::Isometry3d& sensor_state, 
    std::shared_ptr<octomap::OcTree> octo_map);
RayInfo castRay(std::shared_ptr<octomap::OcTree> & octo_map, const Eigen::Vector3d & sensor_origin, const Eigen::Vector3d & ray_direction_world);



#endif // RAYCAST_HPP