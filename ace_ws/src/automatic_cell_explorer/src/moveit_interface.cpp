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