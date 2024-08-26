#include <iostream>
#include "automatic_cell_explorer/state_machine.hpp"
#include "automatic_cell_explorer/move_robot_lib.hpp"
#include "automatic_cell_explorer/calculate_NBV_lib.hpp"


StateMachineNode::StateMachineNode() : Node("state_machine_node"), current_state_(State::Initialize)
{
    //camera_trigger_ = 
    //    this->create_publisher<std_msgs::msg::Bool>("/trigger", 10);
    
    //pointcloud_publisher_ = 
    //    this->create_publisher<sensor_msgs::msg::PointCloud2>("/octomap_pointcloud", 10);

    //octomap_subscriber_ =
    //  this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_full", 10, std::bind(&StateMachineNode::octomap_callback, this, std::placeholders::_1));

    //move_robot_client_ = this->create_client<automatic_cell_explorer::srv::MoveToNbv>("/move_robot_to_pose");

    // Wait for the service to be available
    //while (!move_robot_client_->wait_for_service(std::chrono::seconds(1))) {
    //  RCLCPP_INFO(this->get_logger(), "Waiting for the /move_robot_to_pose service to be available...");
    //}

    //handle_initialize();
    std::cout << "Testing if State machine inits" << std::endl;
    
}



void StateMachineNode::handle_initialize(){
    //RCLCPP_INFO(this->get_logger(), "State: Initialize");

    // Transition to the Capture state
    current_state_ = State::Capture;
    handle_capture();

}

void StateMachineNode::handle_capture(){
    //RCLCPP_INFO(this->get_logger(), "State: Capture");

    //Trigger to start the loop. Should verify it is recieved somehow. 
    //auto trigger = std_msgs::msg::Bool();
    //trigger.data = true;
    //camera_trigger_->publish(trigger);

    //RCLCPP_INFO(this->get_logger(), "Trigger sent.");

    //process octomap here??
    //update planning scene
    //maybe read the newest octomap_full topic??

    // Transition to 
    current_state_ = State::Calculate_NBV;
    handle_calculate_nbv();

}

void StateMachineNode::handle_calculate_nbv(){
    //RCLCPP_INFO(this->get_logger(), "State: Calculate_NBV");

    //read the newes octomap_full message, 

    //CalculateNBV calculate_nbv = CalculateNBV();


    // Transition to 
    current_state_ = State::Move_robot;
    handle_move_robot();

}

void StateMachineNode::handle_move_robot(){
    //RCLCPP_INFO(this->get_logger(), "State: Move_robot");

    //MoveRobot move_robot = MoveRobot();
    
    // Transition to 
    current_state_ = State::Capture;
    handle_capture();

}


