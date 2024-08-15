#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_srvs/srv/set_bool.hpp>
#include "automatic_cell_explorer/srv/move_to_nbv.hpp" 

class Planner : public rclcpp::Node
{
public:
  Planner()
  : Node("planner_node")
  {
    // Subscribe to the PointCloud2 topic
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/octomap_pointcloud", 10, std::bind(&Planner::pointcloud_callback, this, std::placeholders::_1));

    // Create a service client to trigger robot movement
    move_robot_client_ = this->create_client<automatic_cell_explorer::srv::MoveToNbv>("/move_robot_to_pose");

    // Wait for the service to be available
    while (!move_robot_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for the /move_robot_to_pose service to be available...");
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Client<automatic_cell_explorer::srv::MoveToNbv>::SharedPtr move_robot_client_;
  std::atomic<bool> processing_;

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
     if (processing_.load()) {
      RCLCPP_WARN(this->get_logger(), "Previous request still in process. Ignoring new point cloud.");
      return;
    }
    processing_.store(true);

    // Convert the PointCloud2 message to a PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // Perform calculations to determine the next best view (NBV)
    geometry_msgs::msg::PoseStamped nbv_pose = calculate_nbv(pcl_cloud);

    // Trigger the robot movement by calling the service
    call_move_robot_service(nbv_pose);

    processing_.store(false);
  }

  geometry_msgs::msg::PoseStamped calculate_nbv(pcl::PointCloud<pcl::PointXYZ>::Ptr /*_pcl_cloud*/)
  {
    // Your NBV calculation logic goes here
    geometry_msgs::msg::PoseStamped nbv_pose;
    nbv_pose.header.frame_id = "world";
    nbv_pose.header.stamp = this->get_clock()->now();

    // Example: Set NBV pose to a fixed point for simplicity
    nbv_pose.pose.position.x = 0.09;
    nbv_pose.pose.position.y = 0.22;
    nbv_pose.pose.position.z = 1.41;

    // Set the orientation (this example just sets it to identity)
    nbv_pose.pose.orientation.w = 1.0;
    nbv_pose.pose.orientation.x = 0.0;
    nbv_pose.pose.orientation.y = 0.0;
    nbv_pose.pose.orientation.z = 0.0;

    return nbv_pose;
  }

  void call_move_robot_service(const geometry_msgs::msg::PoseStamped& nbv_pose)
  {
    auto request = std::make_shared<automatic_cell_explorer::srv::MoveToNbv::Request>();
    request->pose = nbv_pose;  // This flag could be used to indicate that the robot should move

    // Call the service and wait for the response
    auto result_future = move_robot_client_->async_send_request(request);

    // Optional: handle the result (this could be used to check if the robot moved successfully)
    try {
      auto result = result_future.get();
      if (result->success) {
        RCLCPP_INFO(this->get_logger(), "Robot successfully moved to NBV pose.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to move the robot to NBV pose.");
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Planner>());
  rclcpp::shutdown();
  return 0;
}
