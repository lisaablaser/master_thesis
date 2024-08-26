#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "automatic_cell_explorer/srv/move_to_nbv.hpp" 

class Planner : public rclcpp::Node
{
public:
  Planner()
  : Node("planner_node"),current_angle_(0.0)
  {
    // Subscribe to the PointCloud2 topic (processed octomap), calculate nbv
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/octomap_pointcloud", 10, std::bind(&Planner::plan_path, this, std::placeholders::_1));

 
    move_robot_client_ = this->create_client<automatic_cell_explorer::srv::MoveToNbv>("/move_robot_to_pose");

    // Wait for the service to be available
    while (!move_robot_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for the /move_robot_to_pose service to be available...");
    }

    RCLCPP_INFO(this->get_logger(), "Planner node initialized");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Client<automatic_cell_explorer::srv::MoveToNbv>::SharedPtr move_robot_client_;

  double current_angle_;
  

  void plan_path(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

    // Convert the PointCloud2 message to a PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // Perform calculations to determine the next best view (NBV)
    geometry_msgs::msg::PoseStamped nbv_pose = calculate_nbv(pcl_cloud);


    // Trigger the robot movement by calling the service
    call_move_robot_service(nbv_pose);

    RCLCPP_INFO(this->get_logger(), "Move robot service finished");
 
  }

  geometry_msgs::msg::PoseStamped calculate_nbv(pcl::PointCloud<pcl::PointXYZ>::Ptr /*_pcl_cloud*/)
  {
    double radius = 0.5;  // Set the desired radius of the circle
    double height = 0.6 + 0.755; // Set the desired height (z)
    
    // Calculate the x and y position based on the current angle
    double x = radius * std::cos(current_angle_);
    double y = radius * std::sin(current_angle_);

    current_angle_ += M_PI / 18.0;
    if (current_angle_ >= 2 * M_PI) {
      current_angle_ -= 2 * M_PI;
    }

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, current_angle_);  // Facing outwards

    geometry_msgs::msg::PoseStamped nbv_pose;
    nbv_pose.header.frame_id = "world";
    nbv_pose.header.stamp = this->get_clock()->now();

    nbv_pose.pose.position.x = x;
    nbv_pose.pose.position.y = y;
    nbv_pose.pose.position.z = height;

    nbv_pose.pose.orientation.x = quaternion.x();
    nbv_pose.pose.orientation.y = quaternion.y();
    nbv_pose.pose.orientation.z = quaternion.z();
    nbv_pose.pose.orientation.w = quaternion.w();

    return nbv_pose;
  }

  void call_move_robot_service(const geometry_msgs::msg::PoseStamped& nbv_pose)
  {
    auto request = std::make_shared<automatic_cell_explorer::srv::MoveToNbv::Request>();
    request->pose = nbv_pose;  // This flag could be used to indicate that the robot should move

    // Call the service and wait for the response
    auto result_future = move_robot_client_->async_send_request(request);


    // Optional: handle the result (this could be used to check if the robot moved successfully)
    // TODO 
    // node freezes sometimes. wait for sucess, handle failure. 
    
    
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Planner>());
  rclcpp::shutdown();
  return 0;
}
