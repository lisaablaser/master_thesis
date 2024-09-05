#include <iostream>
#include <octomap_msgs/conversions.h>
#include "automatic_cell_explorer/state_machine.hpp"


StateMachineNode::StateMachineNode() 
    : Node("state_machine_node"), 
    node_(nullptr),
    current_state_(State::Initialise), 
    finished_(false),
    octomap_(std::make_shared<octomap::OcTree>(0.1)),
    exploration_planner_(nullptr)
{
    camera_trigger_ = 
        this->create_publisher<std_msgs::msg::Bool>("/trigger", 10);

    octomap_subscriber_ =
      this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_full", rclcpp::QoS(1), std::bind(&StateMachineNode::update_planning_scene, this, std::placeholders::_1));
   
}



void StateMachineNode::handle_initialise(){
    std::cout << "--State Initialise--" << std::endl;
    node_ = shared_from_this();
    exploration_planner_ = std::make_shared<ExplorationPlanner>(node_,octomap_);

    current_state_ = State::Capture;
}

void StateMachineNode::handle_capture(){
    std::cout << "--State Capture--" << std::endl;

    auto trigger = std_msgs::msg::Bool();
    trigger.data = true;
    camera_trigger_->publish(trigger);
    RCLCPP_INFO(this->get_logger(), "Trigger sent.");

    current_state_ = State::WaitingForOctomap;
}

void StateMachineNode::handle_calculate_nbv(){
    
    std::cout << "--State Calculate Nbv--" << std::endl;

    //exploration_planner.update_states(octomap_);
    robot_trajectory::RobotTrajectory traj = exploration_planner_->calculate_nbv();
    std::cout << "Number of waypoints in the trajectory: " << traj.getWayPointCount() << std::endl;

    current_state_ = State::Move_robot;
}

void StateMachineNode::handle_move_robot(){

    std::cout << "--State MoveRobot--" << std::endl;

    current_state_ = State::Finished;
}


void StateMachineNode::update_planning_scene(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
    if (abstract_tree) {
        octomap::OcTree* received_tree = dynamic_cast<octomap::OcTree*>(abstract_tree);

        if (received_tree) {
            
            octomap_->swapContent(*received_tree);
            RCLCPP_INFO(this->get_logger(), "Planning scene updated with new Octomap data.");
                
            
            std::string file_path = "src/automatic_cell_explorer/test/data/test_octomap.bt";

            // Save the octomap to a .bt file
            if (octomap_->writeBinary(file_path)) {
                std::cout << "Octomap successfully saved to " << file_path << std::endl;
            } else {
                std::cerr << "Failed to save octomap to " << file_path << std::endl;
            }


            current_state_ = State::Calculate_NBV;

        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert message to OcTree.");
            current_state_ = State::Error;
        }
        
    } else {
        RCLCPP_ERROR(this->get_logger(), "Received empty or invalid Octomap message.");
        current_state_ = State::Error;
    }
    delete abstract_tree;
}

void StateMachineNode::execute_state_machine()
{
    rclcpp::Rate loop_rate(10); 
    while (rclcpp::ok()) {
        switch (current_state_) {

            case State::Initialise:
                handle_initialise();
                break;

            case State::Capture:
                handle_capture();
                break;
                
            case State::WaitingForOctomap:
                {
                    auto start_time = std::chrono::steady_clock::now();
                    while (current_state_ == State::WaitingForOctomap && rclcpp::ok()) {
                        rclcpp::spin_some(this->shared_from_this());  
                        loop_rate.sleep();  

                        auto elapsed_time = std::chrono::steady_clock::now() - start_time;
                        if (std::chrono::duration_cast<std::chrono::seconds>(elapsed_time).count() > 5) {
                            RCLCPP_WARN(this->get_logger(), "Callback not received within timeout, triggering new capture");
                            current_state_ = State::Capture;
                            break;
                        }
                    }
                }
                break;

            case State::Calculate_NBV:
                handle_calculate_nbv();
                break;

            case State::Move_robot:
                handle_move_robot();
                break;
            
            case State::Finished:
                RCLCPP_INFO(this->get_logger(), "Process finished...");
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