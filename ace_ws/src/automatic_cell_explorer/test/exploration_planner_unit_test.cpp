#include <gtest/gtest.h>
#include <iostream>
#include <filesystem>  
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>
#include "automatic_cell_explorer/exploration_planner.hpp"

sensor_msgs::msg::PointCloud2 convertOctomapToPointCloud2(const std::shared_ptr<octomap::OcTree>& octree); 
TEST(ExplorationPlannerTest, InitialState)
{
    rclcpp::init(0, nullptr);
   
    std::filesystem::path filename = "/home/lisa/master_thesis/ace_ws/src/automatic_cell_explorer/test/data/octomap.bt";
    std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(filename);

    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("test_node");

    auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/octomap_point_cloud_centers", 10);

    publisher->publish(convertOctomapToPointCloud2(octree));

    std::shared_ptr<ExplorationPlanner> exploration_planner = std::make_shared<ExplorationPlanner>(node, octree);

    double volume = exploration_planner->calculate_occupied_volume();
    
    std::cout << "occupied volume" << volume << std::endl;
    // Add assertions to test initial state
    //ASSERT_TRUE(state_machine.isInitialized());
    //ASSERT_GT(volume, 0.0);  // Example assertion: volume should be greater than 0
    
    // Shut down ROS 2 at the end of the test
    rclcpp::shutdown();
    
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);


    int result = RUN_ALL_TESTS();

    return result;
}

sensor_msgs::msg::PointCloud2 convertOctomapToPointCloud2(const std::shared_ptr<octomap::OcTree>& octree) {

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