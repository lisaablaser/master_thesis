
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "automatic_cell_explorer/utils.hpp" 

void publishRays(const std::vector<RayInfo>& rays,
                 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher) 
{
    
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < rays.size(); ++i) {
        
        visualization_msgs::msg::Marker start_marker, end_marker;

        
        start_marker.header.frame_id = end_marker.header.frame_id = "world"; 
        start_marker.ns = end_marker.ns = "raycasting";
        start_marker.action = end_marker.action = visualization_msgs::msg::Marker::ADD;
        
        start_marker.type = end_marker.type = visualization_msgs::msg::Marker::SPHERE;

        // Set the scale for the points (size of the sphere)
        start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.1;  
        end_marker.scale.x = end_marker.scale.y = end_marker.scale.z = 0.1;        

    
        start_marker.color.a = 1.0;
        start_marker.color.r = 1.0; // Red for the start point
        start_marker.color.g = 0.0;
        start_marker.color.b = 0.0;

        end_marker.color.a = 1.0;
        if (rays[i].hit_unknown) {
            end_marker.color.r = 0.0;
            end_marker.color.g = 1.0;
            end_marker.color.b = 0.0; // Green for unknown space
        } else {
            end_marker.color.r = 0.0;
            end_marker.color.g = 1.0;
            end_marker.color.b = 1.0; // For free and occupied space
        }

        // Set unique IDs for the markers
        start_marker.id = i * 2;        // Even ID for start point
        end_marker.id = i * 2 + 1;      // Odd ID for end point

        // Set the positions for the start and end points
        start_marker.pose.position.x = rays[i].start.x();
        start_marker.pose.position.y = rays[i].start.y();
        start_marker.pose.position.z = rays[i].start.z();

        end_marker.pose.position.x = rays[i].end.x();
        end_marker.pose.position.y = rays[i].end.y();
        end_marker.pose.position.z = rays[i].end.z();

        marker_array.markers.push_back(start_marker);
        marker_array.markers.push_back(end_marker);
    }

    publisher->publish(marker_array);
}


sensor_msgs::msg::PointCloud2 convertOctomapToPointCloud2(const std::shared_ptr<octomap::OcTree>& octree) {

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "world";  
    cloud_msg.header.stamp = rclcpp::Clock().now();
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    size_t point_count = 0;
    for (auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
        if (octree->isNodeOccupied(*it)) {
            
            if (point_count >= modifier.size()) {
                modifier.resize(point_count + 1);
            }
    
            const octomap::point3d& point = it.getCoordinate();

            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

            iter_x += point_count;
            iter_y += point_count;
            iter_z += point_count;

            *iter_x = point.x();
            *iter_y = point.y();
            *iter_z = point.z();

            ++point_count;
        }
    }
    cloud_msg.width = point_count;  
    cloud_msg.height = 1;  
    cloud_msg.is_dense = true; 

    return cloud_msg;
}

void printTransform(const Eigen::Isometry3d& transform) {
    // Extract the translation (position)
    Eigen::Vector3d translation = transform.translation();
    std::cout << "Translation (x, y, z): " 
              << translation.x() << ", " 
              << translation.y() << ", " 
              << translation.z() << std::endl;

    // Extract the rotation as a quaternion
    Eigen::Quaterniond rotation(transform.rotation());
    std::cout << "Rotation (quaternion): "
              << "w: " << rotation.w() << ", "
              << "x: " << rotation.x() << ", "
              << "y: " << rotation.y() << ", "
              << "z: " << rotation.z() << std::endl;

    // Optional: Print rotation as Euler angles if preferred
    Eigen::Vector3d euler_angles = transform.rotation().eulerAngles(0, 1, 2); // roll (X), pitch (Y), yaw (Z)
    std::cout << "Rotation (Euler angles): "
              << "roll: " << euler_angles.x() << ", "
              << "pitch: " << euler_angles.y() << ", "
              << "yaw: " << euler_angles.z() << std::endl;
}