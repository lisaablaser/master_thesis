#include <iostream>
#include <octomap_msgs/conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "automatic_cell_explorer/state_machine.hpp"
#include "automatic_cell_explorer/visualize.hpp"


StateMachineNode::StateMachineNode(MoveGrpPtr mvt_interface, RvizToolPtr rviz_tool) 
    : Node("state_machine_node"), 
    node_(nullptr),
    mvt_interface_(mvt_interface),
    rviz_tool_(rviz_tool),
    current_state_(State::Initialise), 
    finished_(false),
    octomap_(std::make_shared<octomap::OcTree>(0.1)),
    exploration_planner_(std::make_shared<ExplorationPlanner>(mvt_interface_, octomap_)),
    current_req_(ExecuteReq())
{
    camera_trigger_ = 
        this->create_publisher<std_msgs::msg::Bool>("/trigger", 10);

    octomap_subscriber_ =
      this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_full", rclcpp::QoS(1), std::bind(&StateMachineNode::update_planning_scene, this, std::placeholders::_1));

    world_publisher_ =
      this->create_publisher<moveit_msgs::msg::PlanningSceneWorld>("/planning_scene_world",rclcpp::SystemDefaultsQoS());
   
    move_client_ = 
        this->create_client<Execute>("move_robot");

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
    auto nbv_candidates_pose_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("nbv_candidates", 10);
    auto nbv_candidates_ray_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("nbv_candidate_rays", 10);
    auto nbv_candidates_fov_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("nbv_candidate_fov", 10);
    auto nbv_ray_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("nbv_ray", 10);


    std::cout << "--State Calculate Nbv--" << std::endl;
    
    /// TODO: also return all nbv_candidates, costs, traj etc. for viz
    //exploration_planner_->generateCandidates();
   
    

    ExecuteReq request = exploration_planner_->get_nbv_demo();
    current_req_ = request;

    //Nbv nbv = exploration_planner_->getNbv();
    //visualizeNbvRayView(nbv, nbv_ray_pub); //test from view samled in free space. 


    NbvCandidates nbv_candidates = exploration_planner_->getNbvCandidates();
    visualizeNbvCandidatesPose(nbv_candidates, nbv_candidates_pose_pub);

    exploration_planner_->evaluateNbvCandidates();
    visualizeNbvFOV(nbv_candidates, nbv_candidates_fov_pub);
    //visualizeNbvCandidatesRayViews(nbv_candidates, nbv_candidates_ray_pub); //too hevy for rviz
    

    rviz_tool_->prompt("Press next");




    //pose_pub->publish(pose_msg);
    //publish_fov_marker(marker_pub, sensor_state, 64.0, 36.0);
    //publishRays(ray_view.rays, rviz_publisher);



    current_state_ = State::Move_robot;
}

void StateMachineNode::handle_move_robot(){
/// Call service and wait
    std::cout << "--State MoveRobot--" << std::endl;

    auto request_ptr = std::make_shared<ExecuteReq>(current_req_);
    auto future_result = move_client_->async_send_request(request_ptr);

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
    RCLCPP_INFO(this->get_logger(), "Should capture again now");


    // If success
    current_state_ = State::Capture;
    // If not sucess, get NBV
}


void StateMachineNode::update_planning_scene(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
    if (abstract_tree) {
        octomap::OcTree* received_tree = dynamic_cast<octomap::OcTree*>(abstract_tree);

        if (received_tree) {

            /// TODO: Set initial free space in octomap. 

            octomap_->swapContent(*received_tree);
            RCLCPP_INFO(this->get_logger(), "Planning scene updated with new Octomap data.");

            /// TODO: process octomap before published to planning scene. 
 
            moveit_msgs::msg::PlanningSceneWorld msg_out;
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

