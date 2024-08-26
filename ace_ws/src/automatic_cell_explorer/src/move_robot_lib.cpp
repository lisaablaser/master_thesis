//#include "automatic_cell_explorer/srv/move_to_nbv.hpp" 

#include "automatic_cell_explorer/move_robot_lib.hpp"


MoveRobot::MoveRobot(): current_angle_(0.0)
  {

  }


  
void MoveRobot::plan_path()
  {

  }

void MoveRobot::calculate_nbv()
  {
  


  }

void MoveRobot::call_move_robot_service()
  {
    //auto request = std::make_shared<automatic_cell_explorer::srv::MoveToNbv::Request>();
    //request->pose = nbv_pose;  // This flag could be used to indicate that the robot should move

    // Call the service and wait for the response
    //auto result_future = move_robot_client_->async_send_request(request);


    // Optional: handle the result (this could be used to check if the robot moved successfully)
    // TODO 
    // node freezes sometimes. wait for sucess, handle failure. 

    
  }

