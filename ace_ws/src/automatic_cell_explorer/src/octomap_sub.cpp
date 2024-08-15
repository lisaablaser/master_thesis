#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <octomap_msgs/msg/octomap.hpp>  
#include <octomap_msgs/conversions.h>  
#include <octomap/OcTree.h>  

class OctomapSubscriber : public rclcpp::Node
{
public:
  OctomapSubscriber()
  : Node("octomap_subscriber")
  {
   
    subscription_ =
      this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_full", 10, std::bind(&OctomapSubscriber::octomap_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscription_;
  void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) const {
    // Convert the message to an octomap::OcTree object
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg));
    
    if (octree) {
      RCLCPP_INFO(this->get_logger(), "Received Octomap with %zu nodes", octree->size());
      // Here, you can further process the octomap as needed.
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to convert Octomap message to OcTree.");
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapSubscriber>());
  rclcpp::shutdown();
  return 0;
}