#include <iostream>
#include <tf2_eigen/tf2_eigen.hpp>
#include <octomap_msgs/conversions.h>
#include "std_msgs/msg/float64.hpp"

#include "logger.hpp"

#include "automatic_cell_explorer/visualize.hpp"
#include "automatic_cell_explorer/state_machine.hpp"
#include "automatic_cell_explorer/constants.hpp"
#include "automatic_cell_explorer/exploration_planner/nbv.hpp"
#include "automatic_cell_explorer/clustering.hpp"



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
    current_req_(ExecuteReq()),
    iteration_(0),
    logger_("runs/random_local_test.csv"),
    nbv_candidates_()
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
    logIteration();

}

void StateMachineNode::handle_capture(){
    std::cout << "--State Capture--" << std::endl;

    

    auto trigger = std_msgs::msg::Bool();
    trigger.data = true;
    camera_trigger_->publish(trigger);
    RCLCPP_INFO(this->get_logger(), "Trigger sent.");

    
}

void StateMachineNode::handle_calculate_nbv(){
    std::cout << "--State Calculate Nbv--" << std::endl;

    bool visualize = true;
    updatePlanner(current_type_);
    //plm_interface_->updateSceneWithCurrentState();

    auto start_time = std::chrono::high_resolution_clock::now();
    exploration_planner_->updateOctomap(octomap_);

    NbvCandidates nbv_candidates_visualize;
    Nbv nbv;

    //Logic for keeping nbv candidates in memory
    bool has_nbv_memory = false;
    if(has_nbv_memory && current_type_== PlannerType::Global){
        
        exploration_planner_->calculateNbvCandidates(nbv_candidates_);

        nbv_candidates_visualize = nbv_candidates_;

        nbv = exploration_planner_->selectNbv(nbv_candidates_); 

        auto cluster_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("clusters", 10);
        std::vector<Cluster> clusters = exploration_planner_->getClusters(); 
        visualizeClusters(clusters, cluster_pub);

    }
    else{

        exploration_planner_->calculateNbvCandidates();

        nbv_candidates_visualize = exploration_planner_->getNbvCandidates();

        nbv = exploration_planner_->selectNbv();

    }

    


    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(this->get_logger(), "----------------- Calculated NBV in %ld ms", duration.count());

    /// LOG:  
    log_.n_nbv_memory = nbv_candidates_.size();
    log_.nbv_calculation_t = duration.count();
    log_.planner = static_cast<int>(current_type_);
    log_.traj_lenght = nbv.cost;

    
    // Visualizations
    if(visualize){
        auto nbv_ray_dots_pub = node_->create_publisher<MarkerArray>("nbv_ray_dots", 10);
        auto nbv_fov_pub = node_->create_publisher<Marker>("nbv_fov", 10);
        auto nbv_ray_pub = node_->create_publisher<MarkerArray>("nbv_ray", 10);
        auto nbv_candidates_pose_pub = node_->create_publisher<MarkerArray>("nbv_candidates", 10);
        auto nbv_candidates_fov_pub = node_->create_publisher<MarkerArray>("nbv_candidates_fov", 10);
        

        visualizeNbvCandidatesPose(nbv_candidates_visualize, nbv_candidates_pose_pub);
        visualizeNbvCandidatesFOV(nbv_candidates_visualize, nbv_candidates_fov_pub);
        visualizeNbvRayView(nbv, nbv_ray_pub);
        visualizeNbvFov(nbv, nbv_fov_pub);
        visualizeRayDots(nbv, nbv_ray_dots_pub);  
    }


    //rviz_tool_->prompt("Press next");




    // Maybe check progress if raycast is wrong. (looking agains ceiling)
    if(nbv==Nbv() || nbv.gain == 0){
        std::cout << "Had no gain or was invalid. Trying again " << std::endl;
        if(current_type_ == PlannerType::Local){
            std::cout << "--Switching to Global Planner-- " << std::endl;
            current_type_ = PlannerType::Global;
        }   
        /// LOG:
        log_.progress = prev_progress_;
        log_.gain = 0;        
        logIteration();
        handle_calculate_nbv();
        return;
    }
    else if(current_type_ == PlannerType::Global){
        std::cout << "--Switching back to Local planner --- " << std::endl;
        current_type_ = PlannerType::Local;
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
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();
        if (response->success) {

            RCLCPP_INFO(this->get_logger(), "Service call succeeded. Robot moved successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed. Execution was unsuccessful.");
            log_.move_t = -1.0;
            log_.progress = prev_progress_;
            log_.gain = 0;
            logIteration();
            current_state_ = State::Calculate_NBV;
            return;
        }
    } 
    else 
    {
        /// TODO: Some timing issue here, robot seems to move sucessfully, and stil enters this loop somtimes. 
        RCLCPP_ERROR(this->get_logger(), "Service call failed: No response from the service. Will just move on to campture for now");
        //log_.move_t = -1.0;
        //current_state_ = State::Calculate_NBV;
        //return;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    log_.move_t = static_cast<double>(duration);
    
    current_state_ = State::Capture;
}


void StateMachineNode::update_planning_scene(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "New octompa recieved, updating planning scene.");


    octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
    

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
            
            log_.map_update_t = duration.count();

            OctreePtr unknown_tree = extractUnknownOctree(octomap_);

            bool visualize_map = true;
            // Visualize
            if(visualize_map){
                
                OctreePtr free_tree = extractFreeOctree(octomap_);
                OctreePtr frontier_tree = extractFrontierOctreeInBounds(octomap_);


                auto unknown_space_pub = this->create_publisher<octomap_msgs::msg::Octomap>("unknown_space", 10);
                auto free_space_pub = this->create_publisher<octomap_msgs::msg::Octomap>("free_space", 10);
                auto frontiers_pub = this->create_publisher<octomap_msgs::msg::Octomap>("frontiers", 10);
                auto map_pub = this->create_publisher<octomap_msgs::msg::Octomap>("map", 10);
                auto c_center_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("centers",10);
                //auto cluster_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("clusters", 10);
                auto targets_pub = node_->create_publisher<MarkerArray>("targets", 10);

                // only for visualizations. 
                //std::vector<Cluster> clusters = computeClusters(octomap_); //could use getClusters, but only for Gloabal planner
                //visualizeClusters(clusters, cluster_pub);

    


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

            }

          
           

            // LOG
            double unknown_voxel_count = calculateOccupiedVolume(unknown_tree);
            double gain = unknown_voxel_count - prev_progress_;
            log_.progress = unknown_voxel_count;
            log_.gain = gain;

            std::cout << "Previous Progress: " << prev_progress_ << std::endl;
            std::cout << "Current Unknown Voxel Count: " << unknown_voxel_count << std::endl;
            std::cout << "Calculated Gain: " << gain << std::endl;

            // Keep until utility measue is reliable
            // If no actual progress is made, we should witch to global planner. 
            if(current_type_ == PlannerType::Local){
                double progress_tresh = 0.01;
                if(abs(gain) < progress_tresh){
                    std::cout << "--Switching to Global Planner-- progess was: " << gain  << std::endl;
                    current_type_ = PlannerType::Global;
                }   
            }
        
            prev_progress_ = unknown_voxel_count;

            logIteration();

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
                current_state_ = State::Capture;
                break;

            case State::Capture:
                handle_capture();
                current_state_ = State::WaitingForOctomap;
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

