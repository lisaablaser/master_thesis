#ifndef VISUALIZE_HPP
#define VISUALIZE_HPP

#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "automatic_cell_explorer/exploration_planner/nbv.hpp"
#include "automatic_cell_explorer/exploration_planner/raycast.hpp"

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using MarkerPublisher = rclcpp::Publisher<Marker>::SharedPtr;
using MarkerAPublisher = rclcpp::Publisher<MarkerArray>::SharedPtr;

void visualizeRayDots(const Nbv & nbv, MarkerAPublisher publisher);
void visualizeNbvFov(const Nbv & nbv, MarkerPublisher publisher); 
void visualizeNbvCandidatesPose( const NbvCandidates& nbv_candidates, MarkerAPublisher marker_pub);
void visualizeNbvRayView( const Nbv& nbv, MarkerAPublisher marker_pub);
void visualizeNbvCandidatesFOV( const NbvCandidates& nbv_candidates, MarkerAPublisher marker_pub);

sensor_msgs::msg::PointCloud2 convertOctomapToPointCloud2(const std::shared_ptr<octomap::OcTree>& octree); 
void printTransform(const Eigen::Isometry3d& transform); 

#endif // VISUALIZE_HPP