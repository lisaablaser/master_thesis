#include <iostream>
#include <tf2_eigen/tf2_eigen.hpp>
#include <octomap_msgs/conversions.h>
#include "std_msgs/msg/float64.hpp"

#include "automatic_cell_explorer/visualize.hpp"
#include "automatic_cell_explorer/state_machine.hpp"
#include "automatic_cell_explorer/constants.hpp"
#include "automatic_cell_explorer/exploration_planner/nbv.hpp"



StateMachineNode::StateMachineNode(MoveGrpPtr mvt_interface, planning_scene_monitor::PlanningSceneMonitorPtr plm_interface, RvizToolPtr rviz_tool) 
    : Node("state_machine_node"), 
    node_(nullptr),
    mvt_interface_(mvt_interface),
    plm_interface_(plm_interface),
    rviz_tool_(rviz_tool),
    current_state_(State::Initialise), 
    current_type_(PlannerType::Local),
    finished_(false),
    octomap_(std::make_shared<octomap::OcTree>(RES_LARGE)),
    exploration_planner_(createPlanner(current_type_, mvt_interface_, octomap_)),
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

    prev_progress_ = 0;


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
    
    auto nbv_ray_dots_pub = node_->create_publisher<MarkerArray>("nbv_ray_dots", 10);
    auto nbv_fov_pub = node_->create_publisher<Marker>("nbv_fov", 10);
    auto nbv_ray_pub = node_->create_publisher<MarkerArray>("nbv_ray", 10);
    auto nbv_candidates_pose_pub = node_->create_publisher<MarkerArray>("nbv_candidates", 10);
    auto nbv_candidates_fov_pub = node_->create_publisher<MarkerArray>("nbv_candidates_fov", 10);
  
    auto start_time = std::chrono::high_resolution_clock::now();

    exploration_planner_->updateOctomap(octomap_);
    exploration_planner_->calculateNbvCandidates();
    NbvCandidates nbv_candidates = exploration_planner_->getNbvCandidates();
    Nbv nbv = exploration_planner_->selectNbv();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(this->get_logger(), "----------------- Calculated NBV in %ld ms", duration.count());

    
    // Visualizations
    visualizeNbvCandidatesPose(nbv_candidates, nbv_candidates_pose_pub);
    visualizeNbvCandidatesFOV(nbv_candidates, nbv_candidates_fov_pub);
    visualizeNbvRayView(nbv, nbv_ray_pub);
    visualizeNbvFov(nbv, nbv_fov_pub);
    visualizeRayDots(nbv, nbv_ray_dots_pub);

    // Send Stats
    auto nbv_time_pub = this->create_publisher<std_msgs::msg::Float64>("/nbv_time", rclcpp::QoS(10).reliable());
    auto nbv_time_msg = std_msgs::msg::Float64();
    nbv_time_msg.data = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count(); //To seconds
    nbv_time_pub->publish(nbv_time_msg);

    rviz_tool_->prompt("Press next");

    // If no valid nbv was found, skip. N
    /// TODO: dont really need to capture, but should switch to global planner
    if(is_deafault(nbv)){
        std::cout << "Nbv was default, switching to capture to try again " << std::endl;

        current_state_ = State::Capture;
        return;
    }
    if(nbv_candidates.nbv_candidates.empty()){
        std::cout << "No Nbv candidates where calculate" << std::endl;

        current_state_ = State::Capture;
        return;
    }
    if(nbv.ray_view.num_unknowns == 0){
        std::cout << "No gain, skip this pose " << std::endl;
        current_state_ = State::Capture;
        return;
    }

    ExecuteReq req;
    req.start_state = nbv.plan.start_state;
    req.trajectory = nbv.plan.trajectory;
    current_req_ = req;
    current_state_ = State::Move_robot;
}

void StateMachineNode::handle_move_robot(){
    std::cout << "--State MoveRobot--" << std::endl;

    auto request_ptr = std::make_shared<ExecuteReq>(current_req_);
    auto result = move_client_->async_send_request(request_ptr);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();
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
    /// TODO: Error handle if execution fails.
}


void StateMachineNode::update_planning_scene(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "New octompa recieved, updating planning scene.");
    auto unknown_space_pub = this->create_publisher<octomap_msgs::msg::Octomap>("unknown_space", 10);
    auto free_space_pub = this->create_publisher<octomap_msgs::msg::Octomap>("free_space", 10);
    auto frontiers_pub = this->create_publisher<octomap_msgs::msg::Octomap>("frontiers", 10);
    auto map_pub = this->create_publisher<octomap_msgs::msg::Octomap>("map", 10);
    auto c_center_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("centers",10);
    auto cluster_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("clusters", 10);

    octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
    

    // sett/delete all nodes outisde to be free. 

    // If this is too slow, waitForCallback timer is activsated, and new trigger is sent...
    if (abstract_tree) {
        octomap::OcTree* received_tree = dynamic_cast<octomap::OcTree*>(abstract_tree);
        

        if (received_tree) {

            auto start_time = std::chrono::high_resolution_clock::now();

            
            // Update internal octomap
            createInitialSafeSpace(received_tree);
            octomap_ = std::make_shared<octomap::OcTree>(*received_tree);

            // Update PlanningScene
            markUnknownSpaceAsObstacles(received_tree);
            moveit_msgs::msg::PlanningSceneWorld msg_out;
            msg_out.octomap.header = msg->header;
            if (!octomap_msgs::fullMapToMsg(*received_tree, msg_out.octomap.octomap)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to convert modified OcTree back to Octomap message.");
            }
            world_publisher_->publish(msg_out);


            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            RCLCPP_INFO(this->get_logger(), "--------------- Octomap processed in %ld ms", duration.count());


            // Visualize
            OctreePtr unknown_tree = extractUnknownOctree(octomap_);
            OctreePtr free_tree = extractFreeOctree(octomap_);
            OctreePtr frontier_tree = extractFrontierOctreeInBounds(octomap_);

            std::vector<Cluster> clusters = computeClusters(octomap_);
            //std::vector<Cluster> clusters = findUnknownVoxelClusters(octomap_);

            visualizeClusters(clusters, cluster_pub);


            octomap_msgs::msg::Octomap unknown_msg;
            unknown_msg.header.frame_id = "world";  
            unknown_msg.header.stamp = this->get_clock()->now(); 
            octomap_msgs::fullMapToMsg(*unknown_tree, unknown_msg);
            unknown_space_pub->publish(unknown_msg);

            octomap_msgs::msg::Octomap free_msg;
            free_msg.header.frame_id = "world";  
            free_msg.header.stamp = this->get_clock()->now(); 
            octomap_msgs::fullMapToMsg(*free_tree, free_msg);
            free_space_pub->publish(free_msg);

            octomap_msgs::msg::Octomap frontiers_msg;
            frontiers_msg.header.frame_id = "world";  
            frontiers_msg.header.stamp = this->get_clock()->now(); 
            octomap_msgs::fullMapToMsg(*frontier_tree, frontiers_msg);
            frontiers_pub->publish(frontiers_msg);

            octomap_msgs::msg::Octomap map_msg;
            map_msg.header.frame_id = "world";  
            map_msg.header.stamp = this->get_clock()->now(); 
            octomap_msgs::fullMapToMsg(*octomap_, map_msg);
            map_pub->publish(map_msg);


            // Publish progress
            auto unknown_voxel_pub = this->create_publisher<std_msgs::msg::Float64>("/voxel_count", rclcpp::QoS(10).reliable());
            auto u_msg = std_msgs::msg::Float64();
            double unknown_voxel_count = calculateOccupiedVolume(unknown_tree);
            u_msg.data = unknown_voxel_count;
            unknown_voxel_pub->publish(u_msg);

            std::cout << "Unknown voxel count: " << unknown_voxel_count << std::endl;

            // Check if we should switch planners:
            if(current_type_ == PlannerType::Local){
                double progress_tresh = 0.01;
                if(abs(prev_progress_-unknown_voxel_count) < progress_tresh){
                    std::cout << "--Switching to Global Planner--" << std::endl;
                    current_type_ = PlannerType::Global;
                    updatePlanner(current_type_);
                }   
            }
            else if(current_type_ == PlannerType::Global){
                //switch at once. 
                std::cout << "--Switching to Local Planner--" << std::endl;
                current_type_ = PlannerType::Local;
                updatePlanner(current_type_);
                
            
            }
            prev_progress_ = unknown_voxel_count;
            

            // Check termination criteria
            double termination_treshold = 95.0;

            if (unknown_voxel_count > termination_treshold) {
                std::cout << "Planner reached termination criteria" << std::endl;
                finished_ = true;
                current_state_ = State::Finished;
                return;
            }



            RCLCPP_INFO(this->get_logger(), "Setting next state to CalculateNBV.");
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

