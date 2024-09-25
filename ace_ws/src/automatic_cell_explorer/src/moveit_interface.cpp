#include "automatic_cell_explorer/moveit_interface.hpp"



MoveGrpPtr getMoveGroupInterface(rclcpp::Node::SharedPtr node){
    MoveGrpPtr interface = std::make_shared<MoveGrp>(node,"ur_manipulator");
    interface->startStateMonitor();
    interface->setStartStateToCurrentState();
    return interface;
}

PlnScnPtr getPlanningSceenePtr(){
    PlnScnPtr interface = std::make_shared<PlnScn>();

    return interface;
}

//need a worldGeometryMonitor updated by OccupancyMapMpnitor updated by a pointCloudOccupancyMapUpdater. 

PlnScnMonPtr getPlanningSceeneMonitiorPtr(rclcpp::Node::SharedPtr node){


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