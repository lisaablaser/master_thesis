#include "automatic_cell_explorer/moveit_interface.hpp"


MvtInterfacePtr getInterface(std::shared_ptr<rclcpp::Node> node){
    MvtInterfacePtr interface = std::make_shared<MvtInterface>(node,"ur_manipulator");
    interface->startStateMonitor();
    interface->setStartStateToCurrentState();
    return interface;
}