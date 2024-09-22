#ifndef MOVEIT_INTERFACE_HPP
#define MOVEIT_INTERFACE_HPP

#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rclcpp/rclcpp.hpp>



using MoveGrp = moveit::planning_interface::MoveGroupInterface;
using MoveGrpPtr = moveit::planning_interface::MoveGroupInterfacePtr; 
using PlnScn = moveit::planning_interface::PlanningSceneInterface;
using PlnScnPtr = moveit::planning_interface::PlanningSceneInterfacePtr;

MoveGrpPtr getMoveGroupInterface(rclcpp::Node::SharedPtr node);
PlnScnPtr getPlanningSceenePtr();
planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceeneMonitiorPtr(rclcpp::Node::SharedPtr node);

#endif // MOVEIT_INTERFACE_HPP