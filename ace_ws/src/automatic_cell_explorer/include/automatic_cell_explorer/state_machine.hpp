#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
//#include <sensor_msgs/msg/point_cloud2.hpp>
//#include <octomap_msgs/msg/octomap.hpp>  
//#include "automatic_cell_explorer/srv/move_to_nbv.hpp" 

enum class State { Initialize, Capture, Calculate_NBV, Move_robot };

class StateMachineNode : public rclcpp::Node
{
public:
    StateMachineNode();

private:
 
    State current_state_;

    //rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_trigger_;
    //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

    //rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscriber_;
    
    //rclcpp::Client<automatic_cell_explorer::srv::MoveToNbv>::SharedPtr move_robot_client_;

    // State handling methods
    void handle_initialize();
    void handle_capture();
    void handle_calculate_nbv();
    void handle_move_robot();

    // Helper methods


    // Callback methods

};

#endif // STATE_MACHINE_HPP