
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "automatic_cell_explorer/move_robot_service.hpp"

//Class (Node) to offer move service 
//remove camrea trigger from here
//change to recieve a trajectory


  MoveRobotService::MoveRobotService(MoveGrpPtr mvt_interface)
  : Node("move_robot_service_node")
  {
    mvt_interface_ =mvt_interface;
    move_robot_service_ = this->create_service<Execute>(
      "/move_robot", std::bind(&MoveRobotService::move_robot_callback, this, std::placeholders::_1, std::placeholders::_2));

  }

  void MoveRobotService::move_robot_callback(const std::shared_ptr<ExecuteReq> request, std::shared_ptr<ExecuteRes> response)
  {
  Plan plan;
  plan.start_state = request->start_state;
  plan.trajectory = request->trajectory;

  moveit::core::MoveItErrorCode execute_result = mvt_interface_->execute(plan);

  if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Execution succeeded.");
        response->success = true;
    }
  else
    {
        RCLCPP_ERROR(this->get_logger(), "Execution failed with error code: %d", execute_result.val);
        response->success = false;
    }

  }


