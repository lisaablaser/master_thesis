#include <filesystem>
#include "automatic_cell_explorer/moveit_interface.hpp"

MoveGrpPtr getMoveGroupInterface(rclcpp::Node::SharedPtr node){
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    auto optioins = moveit::planning_interface::MoveGroupInterface::Options("ur_manipulator", "robot_description", "");

    MoveGrpPtr interface = std::make_shared<MoveGrp>(node,optioins);
    interface->startStateMonitor();
    interface->setStartStateToCurrentState();

    /// TODO: configure planning stuff here.  
    interface->setPlanningPipelineId("ompl");
    interface->setPlannerId("RRTConnect");

    interface->setPlanningTime(0.1); 
    interface->setGoalTolerance(0.1); //OBS: scary getFinalPoseFromPlan function to achieve accuaracy in raycast. 

    //interface->setNumPlanningAttempts(5); //return shortest of x attempts, not sure if anything changed
    //interface->setReplanAttempts(10); //10-20, nothing happens?
    interface->setWorkspace(-1.0,-1.0,-1.0,1.0,1.0,1.0);
    std::map<std::string, std::string> rrt_connect_params;
    rrt_connect_params["range"] = "0.2";        // 0.1-0.3
    rrt_connect_params["goal_bias"] = "0.1";    // 0.05-0.2

    interface->setPlannerParams("RRTConnect", "ur_manipulator", rrt_connect_params, true);

    interface->setMaxVelocityScalingFactor(0.8);  //  OBS: joint tolerances are set to 0.5 from 0.2 in ur_controllers.yaml
    interface->setMaxAccelerationScalingFactor(0.8);  // 


    auto robot_model = interface->getRobotModel();
    auto joint_model_group = robot_model->getJointModelGroup("ur_manipulator");


    



    return interface;
}

PlnScnPtr getPlanningSceenePtr(){

    PlnScnPtr interface = std::make_shared<PlnScn>();
    
    return interface;
}


PlnScnMonPtr getPlanningSceeneMonitiorPtr(rclcpp::Node::SharedPtr node){
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    auto psm = std::make_shared<PlnScnMon>(node,"robot_description");
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();
    psm->startPublishingPlanningScene(PlnScnMon::UPDATE_SCENE,"/monitored_planning_scene");
    psm->providePlanningSceneService();

    return psm;
}

RvizToolPtr getRvizToolPtr(rclcpp::Node::SharedPtr node, PlnScnMonPtr plm_interface){

    RvizToolPtr moveit_visual_tools = std::make_shared<RvizTool>(
    node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    plm_interface);
    moveit_visual_tools->deleteAllMarkers();
    moveit_visual_tools->loadRemoteControl();

    return moveit_visual_tools;
}