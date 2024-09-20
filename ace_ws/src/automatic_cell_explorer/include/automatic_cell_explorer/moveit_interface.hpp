#ifndef MOVEIT_INTERFACE_HPP
#define MOVEIT_INTERFACE_HPP

#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>



using MvtInterface = moveit::planning_interface::MoveGroupInterface;
using MvtInterfacePtr = std::shared_ptr<MvtInterface>;

MvtInterfacePtr getInterface(std::shared_ptr<rclcpp::Node> node);

#endif // MOVEIT_INTERFACE_HPP