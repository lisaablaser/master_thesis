#ifndef MOVE_ROBOT_LIB_HPP
#define MOVE_ROBOT_LIB_HPP



class MoveRobot 
{
public:
  MoveRobot();

private:

  double current_angle_;
  
  void plan_path();
  
  void calculate_nbv();
  

  void call_move_robot_service();
  
};

#endif // MOVE_ROBOT_LIB_HPP