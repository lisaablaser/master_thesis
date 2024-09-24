#include <iostream>
#include <octomap_msgs/conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include "automatic_cell_explorer/state_machine.hpp"
#include "automatic_cell_explorer/utils.hpp"



StateMachineNode::StateMachineNode(MoveGrpPtr mvt_interface) 
    : Node("state_machine_node"), 
    node_(nullptr),
    mvt_interface_(mvt_interface),
    current_state_(State::Initialise), 
    finished_(false),
    octomap_(std::make_shared<octomap::OcTree>(0.1)),
    exploration_planner_(std::make_shared<ExplorationPlanner>(mvt_interface_, octomap_))
{
    camera_trigger_ = 
        this->create_publisher<std_msgs::msg::Bool>("/trigger", 10);

    octomap_subscriber_ =
      this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_full", rclcpp::QoS(1), std::bind(&StateMachineNode::update_planning_scene, this, std::placeholders::_1));

    world_publisher_ =
      this->create_publisher<moveit_msgs::msg::PlanningSceneWorld>("/planning_scene_world",rclcpp::SystemDefaultsQoS());
   
    move_client_ = 
        this->create_client<automatic_cell_explorer::srv::MoveToNbv>("move_robot_to_pose");

    while (!move_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for the move_robot service to be available...");
        }

    std::cout << "Initializing state machine with node name: " << this->get_name() << std::endl;

}

void StateMachineNode::handle_initialise(){
    std::cout << "--State Initialise--" << std::endl;

    node_ = shared_from_this();

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
    
    auto rviz_publisher = node_->create_publisher<visualization_msgs::msg::MarkerArray>("ray_visualization", 10);
    static auto pose_pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>("sensor_pose", 10);
    static auto marker_pub = node_->create_publisher<visualization_msgs::msg::Marker>("camera_fov_marker", 10);
    

    std::cout << "--State Calculate Nbv--" << std::endl;
    
    RayView ray_view = exploration_planner_->getCurrentRayView();
    std::cout << "information gain from view: " << ray_view.num_unknowns << std::endl;

    std::shared_ptr<automatic_cell_explorer::srv::MoveToNbv::Request> request = std::make_shared<automatic_cell_explorer::srv::MoveToNbv::Request>(exploration_planner_->calculate_nbv());

    handle_move_robot(request);

    // Vizualize Result

    Eigen::Isometry3d&  sensor_state = ray_view.pose;

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = node_->now();  
    pose_msg.header.frame_id = "world";     
    pose_msg.pose = tf2::toMsg(sensor_state);

    pose_pub->publish(pose_msg);
    publish_fov_marker(node_, marker_pub, sensor_state, 64.0, 36.0);
    publishRays(ray_view.rays, rviz_publisher);

    current_state_ = State::Move_robot;
}

void StateMachineNode::handle_move_robot(std::shared_ptr<automatic_cell_explorer::srv::MoveToNbv::Request> request){
/// Call service and wait
    std::cout << "--State MoveRobot--" << std::endl;

    auto future_result = move_client_->async_send_request(request);

    // Block and wait until the response is received
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future_result.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Service call succeeded. Robot moved successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed. Execution was unsuccessful.");
        }
    } 
    else 
    {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: No response from the service.");
    }
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

            moveit_msgs::msg::PlanningSceneWorld msg_out;
            //msg_out.octomap.octomap.id = msg ->id;
            //msg_out.octomap.octomap.binary = msg->binary;
            //msg_out.octomap.octomap.data = msg->data;
            //msg_out.octomap.octomap.header = msg->header;
            msg_out.octomap.header = msg->header;
            msg_out.octomap.set__octomap(*msg);
            world_publisher_->publish(msg_out);

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
                //call handle function here?
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

