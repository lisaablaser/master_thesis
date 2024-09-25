#ifndef VISUALIZE_HPP
#define VISUALIZE_HPP

#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <eigen3/Eigen/Geometry>

#include "automatic_cell_explorer/exploration_planner_utils/raycast.hpp"

sensor_msgs::msg::PointCloud2 convertOctomapToPointCloud2(const std::shared_ptr<octomap::OcTree>& octree); 
void publishRays(const std::vector<RayInfo>& rays,
                 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher);

void printTransform(const Eigen::Isometry3d& transform); 
void publish_fov_marker(std::shared_ptr<rclcpp::Node> node_, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher, const Eigen::Isometry3d& camera_pose, double fov_x, double fov_y); 


#endif // VISUALIZE_HPP