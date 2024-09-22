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

planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceeneMonitiorPtr(rclcpp::Node::SharedPtr node){
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


    auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node,"robot_description");
    //psm->startSceneMonitor();
    psm->startWorldGeometryMonitor("/rgbd_camera/points", psm->getRobotModel()->getModelFrame(), 0.5);
    //bool success = psm->requestPlanningSceneState("/get_planning_scene");
    psm->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY);

    return psm;
}