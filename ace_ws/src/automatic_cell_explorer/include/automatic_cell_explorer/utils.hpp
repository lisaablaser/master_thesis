#ifndef UTILS_HPP
#define UTILS_HPP

#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <eigen3/Eigen/Geometry>

struct RayInfo {
    Eigen::Vector3d start;  // Ray start (sensor origin)
    Eigen::Vector3d end;    // Ray end (hit point or max range)
    bool hit_unknown;       // Whether the ray hit an unknown region
};

sensor_msgs::msg::PointCloud2 convertOctomapToPointCloud2(const std::shared_ptr<octomap::OcTree>& octree); 
void publishRays(const std::vector<RayInfo>& rays,
                 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher);

void printTransform(const Eigen::Isometry3d& transform); 
void publish_fov_marker(const Eigen::Isometry3d& camera_pose, double fov_x, double fov_y, double range,std::shared_ptr<rclcpp::Node> node_); 


#endif // UTILS_HPP