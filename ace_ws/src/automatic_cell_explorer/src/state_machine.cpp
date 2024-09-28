#include <iostream>
#include <tf2_eigen/tf2_eigen.hpp>
#include <octomap_msgs/conversions.h>

#include "automatic_cell_explorer/visualize.hpp"
#include "automatic_cell_explorer/state_machine.hpp"
#include "automatic_cell_explorer/exploration_planner/demo_exploration_planner.hpp"


StateMachineNode::StateMachineNode(MoveGrpPtr mvt_interface, RvizToolPtr rviz_tool) 
    : Node("state_machine_node"), 
    node_(nullptr),
    mvt_interface_(mvt_interface),
    rviz_tool_(rviz_tool),
    current_state_(State::Initialise), 
    finished_(false),
    octomap_(std::make_shared<octomap::OcTree>(0.1)),
    exploration_planner_(std::make_shared<DemoExplorationPlanner>(mvt_interface_, octomap_)),
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
    std::cout << "--State Calculate Nbv--" << std::endl;
    
    auto rviz_publisher = node_->create_publisher<visualization_msgs::msg::MarkerArray>("ray_visualization", 10);
    auto marker_pub = node_->create_publisher<visualization_msgs::msg::Marker>("nbv_fov", 10);
    auto nbv_ray_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("nbv_ray", 10);
    auto nbv_candidates_pose_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("nbv_candidates", 10);
    auto nbv_candidates_fov_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("nbv_candidates_fov", 10);
    
  
    

    exploration_planner_->calculateNbvCandidates();
    NbvCandidates nbv_candidates = exploration_planner_->getNbvCandidates();
    Nbv nbv = exploration_planner_->selectNbv();

    if (exploration_planner_->terminationCriteria()){
        std::cout << "Planner reached termination criteria" << std::endl;
        finished_ = true;
        current_state_ = State::Finished;
        return;

    }
    
    // Visualizations
    visualizeNbvRayView(nbv, nbv_ray_pub); /// TODO: improve ray visualization
    visualizeNbvFov(nbv, 64.0, 36.0, marker_pub);
    visualizeNbvCandidatesPose(nbv_candidates, nbv_candidates_pose_pub);
    visualizeNbvCandidatesFOV(nbv_candidates, nbv_candidates_fov_pub);
    publishRays(nbv.ray_view.rays, rviz_publisher);


    // Progress
    rviz_tool_->prompt("Press next");
    ExecuteReq req;
    req.start_state = nbv.plan.start_state;
    req.trajectory = nbv.plan.trajectory;
    current_req_ = req;
    current_state_ = State::Move_robot;
}

void StateMachineNode::handle_move_robot(){
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


    
    current_state_ = State::Capture;
    // If not sucess, chnge to calculateNbv state. 
}


void StateMachineNode::update_planning_scene(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "New octompa recieved, updating planning scene.");
    auto unknown_space_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("unknown_space", 10);
    auto free_space_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("free_space", 10);
    auto frontiers_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("frontiers", 10);

    octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);

    // If this is too slow, waitForCallback timer is activsated, and new trigger is sent...
    if (abstract_tree) {
        octomap::OcTree* received_tree = dynamic_cast<octomap::OcTree*>(abstract_tree);

        if (received_tree) {

            RCLCPP_INFO(this->get_logger(), "Tree recieved.");
            // Update octo map
            createInitialSafeSpace(received_tree, 1.5, 1.0, 2.0, 0.01);
            octomap_ = std::make_shared<octomap::OcTree>(*received_tree);

            // Visualize
            OctrePtr unknown_tree = extractUnknownOctree(received_tree);
            OctrePtr free_tree = extractFreeOctree(received_tree);
            OctrePtr frontier_tree = extractFrontierOctree(received_tree);
            sensor_msgs::msg::PointCloud2 unknown_pc = convertOctomapToPointCloud2(unknown_tree);
            sensor_msgs::msg::PointCloud2 free_pc = convertOctomapToPointCloud2(free_tree);
            sensor_msgs::msg::PointCloud2 frontiers_pc = convertOctomapToPointCloud2(frontier_tree);
            unknown_space_pub->publish(unknown_pc);
            free_space_pub->publish(free_pc);
            frontiers_pub->publish(frontiers_pc);

            // Update PlanningScene
            markUnknownSpaceAsObstacles(received_tree, 2.0, 2.0, 2.0, 0.01);
            moveit_msgs::msg::PlanningSceneWorld msg_out;
            msg_out.octomap.header = msg->header;
            if (!octomap_msgs::fullMapToMsg(*received_tree, msg_out.octomap.octomap)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to convert modified OcTree back to Octomap message.");
            }
            world_publisher_->publish(msg_out);

            RCLCPP_INFO(this->get_logger(), "setting next state to CalculateNBV.");
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
                        if (std::chrono::duration_cast<std::chrono::seconds>(elapsed_time).count() > 10) {
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

