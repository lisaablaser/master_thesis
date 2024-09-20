#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "automatic_cell_explorer/srv/move_to_nbv.hpp"
#include "automatic_cell_explorer/moveit_interface.hpp"

//Class (Node) to offer move service 
//remove camrea trigger from here
//change to recieve a trajectory

class MoveRobotNode : public rclcpp::Node{
public:
  using MvToNbv = automatic_cell_explorer::srv::MoveToNbv;
  MoveRobotNode(MvtInterfacePtr mvt_interface);

private:
  rclcpp::Service<MvToNbv>::SharedPtr move_robot_service_;
  MvtInterfacePtr mvt_interface_;


  void move_robot_callback(const std::shared_ptr<MvToNbv::Request> request,
                           std::shared_ptr<MvToNbv::Response> response);

};
