
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "automatic_cell_explorer/move_robot_service.hpp"

//Class (Node) to offer move service 
//remove camrea trigger from here
//change to recieve a trajectory


  MoveRobotNode::MoveRobotNode(MoveGrpPtr mvt_interface)
  : Node("move_robot_service_node")
  {
    mvt_interface_ =mvt_interface;
    move_robot_service_ = this->create_service<MvToNbv>(
      "/move_robot_to_pose", std::bind(&MoveRobotNode::move_robot_callback, this, std::placeholders::_1, std::placeholders::_2));

  }


  void MoveRobotNode::move_robot_callback(const std::shared_ptr<MvToNbv::Request> request,
                           std::shared_ptr<MvToNbv::Response> response)
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
    
    mvt_interface_->setPoseTarget(request->pose.pose);
    const auto &jp = mvt_interface_->getCurrentJointValues();
    std::cout<<"current jp" << std::endl;
    for (double j:jp){
      std::cout<<j<<std::endl;
    }
    

    // Plan to the target pose
    auto const [success, plan] = [&mvt_interface = mvt_interface_]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(mvt_interface->plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan if successful
    if(success) {
      RCLCPP_INFO(this->get_logger(), "Planning succeeded. Executing...");
      mvt_interface_->execute(plan);
      response->success = true;

      const auto &jp_after = mvt_interface_->getCurrentJointValues();
      std::cout<<"current jp" << std::endl;
      for (double j:jp_after){
        std::cout<<j<<std::endl;
    }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
      response->success = false;
      //maybe request another nbv? Or try planning again. 
    }

    //report back  status, not send a trigger. SM decides what to do next. 

  }

/*int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
   
  auto move_robot_node = std::make_shared<MoveRobotNode>();

  rclcpp::spin(move_robot_node);
  rclcpp::shutdown();
  return 0;
}
*/
