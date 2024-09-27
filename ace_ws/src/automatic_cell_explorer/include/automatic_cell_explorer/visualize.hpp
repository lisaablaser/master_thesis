#ifndef VISUALIZE_HPP
#define VISUALIZE_HPP

#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <eigen3/Eigen/Geometry>

#include "automatic_cell_explorer/exploration_planner/raycast.hpp"
#include "automatic_cell_explorer/exploration_planner/nbv.hpp"

using MarkerAPublisher = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr;
using MarkerPublisher = rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr;

void visualizeNbvFov(const Nbv & nbv, double fov_x, double fov_y, MarkerPublisher publisher); 
void visualizeNbvCandidatesPose( const NbvCandidates& nbv_candidates, MarkerAPublisher marker_pub);
void visualizeNbvRayView( const Nbv& nbv, MarkerAPublisher marker_pub);
void visualizeNbvCandidatesFOV( const NbvCandidates& nbv_candidates, MarkerAPublisher marker_pub);

sensor_msgs::msg::PointCloud2 convertOctomapToPointCloud2(const std::shared_ptr<octomap::OcTree>& octree); 
void publishRays(const std::vector<RayInfo>& rays, MarkerAPublisher publisher);

void printTransform(const Eigen::Isometry3d& transform); 

#endif // VISUALIZE_HPP