#ifndef MOVEIT_TYPES_HPP
#define MOVEIT_TYPES_HPP

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


using MoveGrp = moveit::planning_interface::MoveGroupInterface;
using MoveGrpPtr = moveit::planning_interface::MoveGroupInterfacePtr; 
using PlnScn = moveit::planning_interface::PlanningSceneInterface;
using PlnScnPtr = moveit::planning_interface::PlanningSceneInterfacePtr;
using PlnScnMon = planning_scene_monitor::PlanningSceneMonitor;
using PlnScnMonPtr = planning_scene_monitor::PlanningSceneMonitorPtr;
using RvizTool = moveit_visual_tools::MoveItVisualTools;
using RvizToolPtr = moveit_visual_tools::MoveItVisualToolsPtr;


using Plan = moveit::planning_interface::MoveGroupInterface::Plan;

#endif // MOVEIT_TYPES_HPP