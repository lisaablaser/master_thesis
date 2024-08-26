#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "automatic_cell_explorer/srv/move_to_nbv.hpp" 

//Class (Node) to offer move service 
//remove camrea trigger from here
//change to recieve a trajectory

class MoveRobotNode : public rclcpp::Node{
public:
  MoveRobotNode()
  : Node("move_robot_service_node"), move_group_interface_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), "ur_manipulator")
  {
   
    move_robot_service_ = this->create_service<automatic_cell_explorer::srv::MoveToNbv>(
      "/move_robot_to_pose", std::bind(&MoveRobotNode::move_robot_callback, this, std::placeholders::_1, std::placeholders::_2));


  }

private:
  rclcpp::Service<automatic_cell_explorer::srv::MoveToNbv>::SharedPtr move_robot_service_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;


  void move_robot_callback(const std::shared_ptr<automatic_cell_explorer::srv::MoveToNbv::Request> request,
                           std::shared_ptr<automatic_cell_explorer::srv::MoveToNbv::Response> response)
  {
    
    RCLCPP_INFO(this->get_logger(), "Requested Pose:");
    RCLCPP_INFO(this->get_logger(), "Position - x: %.2f, y: %.2f, z: %.2f",
                request->pose.pose.position.x, 
                request->pose.pose.position.y, 
                request->pose.pose.position.z);

    RCLCPP_INFO(this->get_logger(), "Orientation - x: %.2f, y: %.2f, z: %.2f, w: %.2f",
                request->pose.pose.orientation.x, 
                request->pose.pose.orientation.y, 
                request->pose.pose.orientation.z, 
                request->pose.pose.orientation.w);
    
    move_group_interface_.setPoseTarget(request->pose.pose);

    // Plan to the target pose
    auto const [success, plan] = [&move_group_interface = move_group_interface_]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan if successful
    if(success) {
      RCLCPP_INFO(this->get_logger(), "Planning succeeded. Executing...");
      move_group_interface_.execute(plan);
      response->success = true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
      response->success = false;
      //maybe request another nbv? Or try planning again. 
    }

    //report back  status, not send a trigger. SM decides what to do next. 

  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
   
  auto move_robot_node = std::make_shared<MoveRobotNode>();

  rclcpp::spin(move_robot_node);
  rclcpp::shutdown();
  return 0;
}
