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
#include "automatic_cell_explorer/utils.hpp"


TEST(ExplorationPlannerTest, InitialState)
{
    rclcpp::init(0, nullptr);
    std::filesystem::path filename = "/home/lisa/master_thesis/ace_ws/src/automatic_cell_explorer/test/data/test_octomap.bt";
    std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(filename);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("test_node");
    //std::shared_ptr<ExplorationPlanner> exploration_planner = std::make_shared<ExplorationPlanner>(node, octree);
    auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/octomap_point_cloud_centers", 10);
    auto rviz_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("ray_visualization", 10);


    // Raycast
    
    // auto [information_gain, rays] = exploration_planner->test_sim_view(*octree, 100.0);
    
    // publishRays(rays, rviz_publisher);



    // double volume = exploration_planner->calculate_occupied_volume();
    
    // std::cout << "occupied volume" << volume << std::endl;
    // std::cout << "information gain from view" << information_gain << std::endl;

    // publisher->publish(convertOctomapToPointCloud2(octree));


    rclcpp::shutdown();
    
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);


    int result = RUN_ALL_TESTS();

    return result;
}

