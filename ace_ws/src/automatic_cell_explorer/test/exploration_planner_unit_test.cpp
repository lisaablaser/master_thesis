#include <gtest/gtest.h>
#include <iostream>
#include <filesystem>  
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <eigen3/Eigen/Geometry>
#include "automatic_cell_explorer/exploration_planner.hpp"

sensor_msgs::msg::PointCloud2 convertOctomapToPointCloud2(const std::shared_ptr<octomap::OcTree>& octree); 
void publishRays(const std::vector<RayInfo>& rays,
                 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher);
TEST(ExplorationPlannerTest, InitialState)
{
    rclcpp::init(0, nullptr);
    std::filesystem::path filename = "/home/lisa/master_thesis/ace_ws/src/automatic_cell_explorer/test/data/test_octomap.bt";
    std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(filename);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("test_node");
    std::shared_ptr<ExplorationPlanner> exploration_planner = std::make_shared<ExplorationPlanner>(node, octree);
    auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/octomap_point_cloud_centers", 10);
    auto rviz_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("ray_visualization", 10);


    // Raycast
    
    auto [information_gain, rays] = exploration_planner->test_sim_view(*octree, 10.0);
    
    publishRays(rays, rviz_publisher);



    double volume = exploration_planner->calculate_occupied_volume();
    
    std::cout << "occupied volume" << volume << std::endl;

    publisher->publish(convertOctomapToPointCloud2(octree));


    rclcpp::shutdown();
    
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);


    int result = RUN_ALL_TESTS();

    return result;
}

void publishRays(const std::vector<RayInfo>& rays,
                 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher) 
{
    // Create a MarkerArray message
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < rays.size(); ++i) {
        // Create a Marker for each ray
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "rgbd_camera"; 
        marker.ns = "raycasting";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set scale for the line (width of the ray)
        marker.scale.x = 0.01;

        // Color the ray depending on whether it hit unknown space
        if (rays[i].hit_unknown) {
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0; // Blue for unknown
        } else {
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0; // Green for free space
        }

        // Define the start and end points of the ray
        geometry_msgs::msg::Point start, end;
        start.x = rays[i].start.x();
        start.y = rays[i].start.y();
        start.z = rays[i].start.z();
        end.x = rays[i].end.x();
        end.y = rays[i].end.y();
        end.z = rays[i].end.z();

        // Add start and end points to the marker
        marker.points.push_back(start);
        marker.points.push_back(end);

        // Add the marker to the MarkerArray
        marker_array.markers.push_back(marker);
    }

    // Publish the MarkerArray
    publisher->publish(marker_array);
}

sensor_msgs::msg::PointCloud2 convertOctomapToPointCloud2(const std::shared_ptr<octomap::OcTree>& octree) {
    //OBS: everything is stretched, not good representation. 

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "world";  
    cloud_msg.header.stamp = rclcpp::Clock().now();
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    size_t point_count = 0;
    for (auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
        if (octree->isNodeOccupied(*it)) {
            
            if (point_count >= modifier.size()) {
                modifier.resize(point_count + 1);
            }
    
            const octomap::point3d& point = it.getCoordinate();

            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

            iter_x += point_count;
            iter_y += point_count;
            iter_z += point_count;

            *iter_x = point.x();
            *iter_y = point.y();
            *iter_z = point.z();

            ++point_count;
        }
    }
    cloud_msg.width = point_count;  
    cloud_msg.height = 1;  
    cloud_msg.is_dense = true; 

    return cloud_msg;
}