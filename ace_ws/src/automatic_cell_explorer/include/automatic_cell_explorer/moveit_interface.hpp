#ifndef MOVEIT_INTERFACE_HPP
#define MOVEIT_INTERFACE_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "automatic_cell_explorer/moveit_types.hpp"


MoveGrpPtr getMoveGroupInterface(rclcpp::Node::SharedPtr node);
PlnScnPtr getPlanningSceenePtr();
PlnScnMonPtr getPlanningSceeneMonitiorPtr(rclcpp::Node::SharedPtr node);
RvizToolPtr getRvizToolPtr(rclcpp::Node::SharedPtr rviz_node, PlnScnMonPtr plm_interface);

#endif // MOVEIT_INTERFACE_HPP