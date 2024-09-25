#ifndef MOVE_ROBOT_SERVICE_HPP
#define MOVE_ROBOT_SERVICE_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>


#include "automatic_cell_explorer/moveit_interface.hpp"
#include "automatic_cell_explorer/execute_srv.hpp"




class MoveRobotService : public rclcpp::Node{
public:

  MoveRobotService(MoveGrpPtr mvt_interface);

private:
  rclcpp::Service<Execute>::SharedPtr move_robot_service_;
  MoveGrpPtr mvt_interface_;


  void move_robot_callback(const std::shared_ptr<ExecuteReq> request, std::shared_ptr<ExecuteRes> response);

};

#endif // MOVE_ROBOT_SERVICE_HPP