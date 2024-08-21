#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <octomap_msgs/msg/octomap.hpp>  
#include <octomap_msgs/conversions.h>  
#include <octomap/OcTree.h>  
#include <std_msgs/msg/bool.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class OctomapProcessor : public rclcpp::Node
{
public:
  OctomapProcessor()
  : Node("octomap_processor")
  {
   
    subscription_ =
      this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_full", 10, std::bind(&OctomapProcessor::octomap_callback, this, std::placeholders::_1));
    pointcloud_publisher_ = 
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/octomap_pointcloud", 10);
    camera_trigger_ = 
        this->create_publisher<std_msgs::msg::Bool>("/trigger", 10);

    auto trigger = std_msgs::msg::Bool();
    trigger.data = true;
    camera_trigger_->publish(trigger);

    RCLCPP_INFO(this->get_logger(), "Octomap_processor node intilized, trigger sent.");
  }

private:
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_trigger_;


  void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) const {
    // Convert the message to an octomap::OcTree object
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg));
    
    if (octree) {
      RCLCPP_INFO(this->get_logger(), "Received Octomap with %zu nodes", octree->size());

      // Here, you can further process the octomap as needed.
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

    for (auto it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
        if (octree->isNodeOccupied(*it)) {
        pcl_cloud.push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
        }
    }

    // Convert PCL PointCloud to ROS PointCloud2
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(pcl_cloud, output);
    output.header.frame_id = "world";  
    output.header.stamp = this->get_clock()->now();

    // Publish the point cloud
    pointcloud_publisher_->publish(output);

    auto trigger = std_msgs::msg::Bool();
    trigger.data = true;
    camera_trigger_->publish(trigger);
    RCLCPP_INFO(this->get_logger(), "Trigger sent.");


    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to convert Octomap message to OcTree.");
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapProcessor>());
  rclcpp::shutdown();
  return 0;
}