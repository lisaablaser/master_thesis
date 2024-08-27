#include <iostream>
#include <octomap_msgs/conversions.h>
#include "automatic_cell_explorer/state_machine.hpp"
#include "automatic_cell_explorer/move_robot_lib.hpp"
#include "automatic_cell_explorer/calculate_NBV_lib.hpp"


StateMachineNode::StateMachineNode() : Node("state_machine_node"), current_state_(State::Initialize), finished_(false)
{
    camera_trigger_ = 
        this->create_publisher<std_msgs::msg::Bool>("/trigger", 10);
    
    //pointcloud_publisher_ = 
    //    this->create_publisher<sensor_msgs::msg::PointCloud2>("/octomap_pointcloud", 10);

    octomap_subscriber_ =
      this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_full", 10, std::bind(&StateMachineNode::update_planning_scene, this, std::placeholders::_1));

    planning_scene_ = std::make_shared<octomap::OcTree>(0.1);
    //move_robot_client_ = this->create_client<automatic_cell_explorer::srv::MoveToNbv>("/move_robot_to_pose");

    // Wait for the service to be available
    //while (!move_robot_client_->wait_for_service(std::chrono::seconds(1))) {
    //  RCLCPP_INFO(this->get_logger(), "Waiting for the /move_robot_to_pose service to be available...");
    //}

    
    std::cout << "initialize finihshed" << std::endl;
   
    
}



void StateMachineNode::handle_initialize(){
    //RCLCPP_INFO(this->get_logger(), "State: Initialize");
    std::cout << "Handle init function" << std::endl;
    // Transition to the Capture state
    current_state_ = State::Capture;
    

}

void StateMachineNode::handle_capture(){
    //RCLCPP_INFO(this->get_logger(), "State: Capture");
    std::cout << "Handle capture function" << std::endl;

    //Trigger to start the loop. Should verify it is recieved somehow. 
    auto trigger = std_msgs::msg::Bool();
    trigger.data = true;
    camera_trigger_->publish(trigger);

    RCLCPP_INFO(this->get_logger(), "Trigger sent.");

    current_state_ = State::WaitingForCallback;

    //process octomap here??
    //update planning scene
    //maybe read the newest octomap_full topic??

    // Transition to 
    //calback triggers next step
    

}

void StateMachineNode::handle_calculate_nbv(){
    //RCLCPP_INFO(this->get_logger(), "State: Calculate_NBV");
    std::cout << "Hanle calculate NBV function" << std::endl;
    //read the newes octomap_full message, 

    //CalculateNBV calculate_nbv = CalculateNBV();


    // Transition to 
    current_state_ = State::Move_robot;
    

}

void StateMachineNode::handle_move_robot(){
    //RCLCPP_INFO(this->get_logger(), "State: Move_robot");
    std::cout << "Hanle move robot function" << std::endl;
    //MoveRobot move_robot = MoveRobot();
    
    // Transition to 
    current_state_ = State::Finished;
    //handle_capture();

}


void StateMachineNode::update_planning_scene(const octomap_msgs::msg::Octomap::SharedPtr msg)
{

    octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
    if (abstract_tree) {
        octomap::OcTree* received_tree = dynamic_cast<octomap::OcTree*>(abstract_tree);

        if (received_tree) {
            // You can now use the received_tree to update your planning_scene_
            planning_scene_->swapContent(*received_tree);
            RCLCPP_INFO(this->get_logger(), "Planning scene updated with new Octomap data.");
            current_state_ = State::Calculate_NBV;
            

        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert message to OcTree.");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Received empty or invalid Octomap message.");
    }

}

void StateMachineNode::execute_state_machine()
{
    rclcpp::Rate loop_rate(10); // Adjust rate as needed

    while (rclcpp::ok()) {
        switch (current_state_) {
            case State::Initialize:
                handle_initialize();
                break;

            case State::Capture:
                handle_capture();
                break;
                
            case State::WaitingForCallback:
                RCLCPP_INFO(this->get_logger(), "waiting for callback");

                while (current_state_ == State::WaitingForCallback && rclcpp::ok()) {
                    RCLCPP_INFO(this->get_logger(), "waiting for callback");
                    rclcpp::spin_some(this->shared_from_this());  // Spin to process callbacks
                    loop_rate.sleep();  // Sleep to avoid tight loop spinning
                }
                break;

            case State::Calculate_NBV:
                handle_calculate_nbv();
                break;

            case State::Move_robot:
                handle_move_robot();
                break;
            
            case State::Finished:
                RCLCPP_INFO(this->get_logger(), "Process finished cleanly...");
                finished_ = true; 
                return;
                break;

            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown state.");
                rclcpp::shutdown();
                return;
                break;
        }

        
    }
}